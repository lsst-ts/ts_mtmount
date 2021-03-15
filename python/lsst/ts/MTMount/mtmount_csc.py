# This file is part of ts_MTMount.
#
# Developed for Vera C. Rubin Observatory Telescope and Site Systems.
# This product includes software developed by the LSST Project
# (https://www.lsst.org).
# See the COPYRIGHT file at the top-level directory of this distribution
# for details of code ownership.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

__all__ = ["MTMountCsc"]

import asyncio
import json
import math
import re
import signal
import subprocess

from lsst.ts import salobj
from lsst.ts.idl.enums.MTMount import DriveState
from .config_schema import CONFIG_SCHEMA
from . import constants
from . import command_futures
from . import commands
from . import enums
from . import limits
from . import __version__

# Extra time to wait for commands to be done (sec)
TIMEOUT_BUFFER = 5

# Maximum time (second) between rotator application telemetry topics,
# beyond which rotator velocity will not be estimated.
MAX_ROTATOR_APPLICATION_GAP = 1.0

# Timeout for starting the mock controller
# and seeing the "running" message (sec).
MOCK_CTRL_START_TIMEOUT = 30

# Timeout for seeing telemetry from the telemetry client (sec).
TELEMETRY_START_TIMEOUT = 30

# Maximum time to wait for rotator telemetry (seconds).
# Must be significantly greater than the interval between rotator
# telemetry updates, which should not be longer than 0.2 seconds.
# For minimum confusion when CCW following fails, this should also be
# significantly less than the maximum time the low-level controller waits
# for a tracking command, which is controlled by setting "Tracking Wait time
# for check setpoint"; on 2020-02-01 the value was 5 seconds.
ROTATOR_TELEMETRY_TIMEOUT = 1


class MTMountCsc(salobj.ConfigurableCsc):
    """MTMount CSC

    Parameters
    ----------
    initial_state : `salobj.State` or `int`, optional
        The initial state of the CSC. This is provided for unit testing,
        as real CSCs should start up in `lsst.ts.salobj.StateSTANDBY`,
        the default.
    simulation_mode : `int`, optional
        Simulation mode.
    run_mock_controller : `bool`, optional
        Run the mock controller (using random ports)?
        Ignored unless ``simulation_mode == 1``.
        False with ``simulation_mode == 1`` is for unit tests
        which run their own mock controller; many tests do this
        in order to monitor what is going on in the controller.
    mock_command_port : `int` or `None`, optional
        Port for mock controller TCP/IP interface. If `None` then use the
        standard value. Ignored unless running in simulation mode
        and ``run_mock_controller`` false. This supports unit tests
        which run their own mock controller with randomly chosen ports.
    mock_telemetry_port : `int` or `None`, optional
        Port for mock controller telemetry server. If `None` then use the
        standard value. Ignored unless running in simulation mode
        and ``run_mock_controller`` false. This supports unit tests
        which run their own mock controller with randomly chosen ports.

    Raises
    ------
    salobj.ExpectedError
        If initial_state or simulation_mode is invalid.

    Notes
    -----
    **Simulation Modes**

    Supported simulation modes:

    * 0: regular operation
    * 1: simulation mode: start a mock TCP/IP controller and talk to it

    **Error Codes**

    See `CscErrorCode`
    """

    valid_simulation_modes = (0, 1)
    version = __version__

    def __init__(
        self,
        config_dir=None,
        initial_state=salobj.State.STANDBY,
        simulation_mode=0,
        run_mock_controller=True,
        mock_command_port=None,
        mock_telemetry_port=None,
    ):
        if simulation_mode == 0:
            command_port = constants.CSC_COMMAND_PORT
            telemetry_port = constants.CSC_COMMAND_PORT
        else:
            if run_mock_controller:
                # Set this in start, when the mock controller is run
                command_port = None
                telemetry_port = None
            else:
                command_port = (
                    mock_command_port
                    if mock_command_port is not None
                    else constants.CSC_COMMAND_PORT
                )
                telemetry_port = (
                    mock_telemetry_port
                    if mock_telemetry_port is not None
                    else constants.TELEMETRY_PORT
                )
        self.command_port = command_port
        self.telemetry_port = telemetry_port

        self.run_mock_controller = run_mock_controller

        # Connection to the low-level controller, or None if not connected.
        self.reader = None  # asyncio.StreamReader if connected
        self.writer = None  # asyncio.StreamWriter if connected

        # Subprocess running the telemetry client
        self.telemetry_client_process = None

        # Subprocess running the mock controller
        self.mock_controller_process = None

        # Dict of command sequence-id: (ack_task, done_task).
        # ack_task is set to the timeout (sec) when command is acknowledged.
        # done_task is set to to None when the command is done.
        # Both are set to exceptions if the command fails.
        self.command_dict = dict()

        self.command_lock = asyncio.Lock()

        self.on_drive_states = set(
            (DriveState.MOVING, DriveState.STOPPING, DriveState.STOPPED)
        )

        # Does the CSC have control of the mount?
        # Once the low-level controller reports this in an event,
        # add a SAL event and get the value from that.
        self.has_control = False

        # Task that waits while connecting to the TCP/IP controller.
        self.connect_task = salobj.make_done_future()

        # Task to track enabling and disabling
        self.enable_task = salobj.make_done_future()
        self.disable_task = salobj.make_done_future()

        self.mock_controller_started_task = asyncio.Future()
        self.monitor_telemetry_client_task = salobj.make_done_future()

        # Tasks for camera cable wrap following the rotator
        self.camera_cable_wrap_follow_start_task = salobj.make_done_future()
        self.camera_cable_wrap_follow_loop_task = salobj.make_done_future()

        # Task for self.read_loop
        self.read_loop_task = salobj.make_done_future()

        # Is camera rotator actual position - demand position
        # greater than config.max_rotator_position_error?
        # Log a warning every time this transitions to True.
        self.rotator_position_error_excessive = False

        # Should the CSC be connected?
        # Used to decide whether disconnecting sends the CSC to a Fault state.
        # Set True when fully connected and False just before
        # intentionally disconnecting.
        self.should_be_connected = False

        super().__init__(
            name="MTMount",
            index=0,
            config_schema=CONFIG_SCHEMA,
            config_dir=config_dir,
            initial_state=initial_state,
            simulation_mode=simulation_mode,
        )

        self.rotator = salobj.Remote(
            domain=self.domain, name="MTRotator", include=["rotation"]
        )

        self.mtmount_remote = salobj.Remote(
            domain=self.domain, name="MTMount", include=["cameraCableWrap"]
        )

        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, self.signal_handler)

        # Needed so `start` sees it. The value has been checked at this point,
        # but the simulation_mode attribute is usually set by super().start().
        self.evt_simulationMode.set_put(mode=simulation_mode)

    async def begin_enable(self, data):
        """Take control of the mount and initialize devices.

        If taking control fails, raise an exception to leave the state
        as DISABLED.
        If initializing fails, go to FAULT state (in enable_devices).
        """
        self.disable_task.cancel()
        self.enable_task.cancel()
        await super().begin_enable(data)
        if not self.has_control:
            try:
                self.log.info("Ask for permission to command the mount.")
                await self.send_command(
                    commands.AskForCommand(commander=enums.Source.CSC)
                )
                self.has_control = True
            except Exception as e:
                raise salobj.ExpectedError(
                    f"The CSC was not allowed to command the mount: {e!r}"
                )

        self.enable_task = asyncio.create_task(self.enable_devices())
        try:
            await self.enable_task
        except Exception:
            self.log.warning(
                "Could not enable devices; disabling devices and giving up control."
            )
            self.disable_task = asyncio.create_task(self.disable_devices())
            await self.disable_task
            raise

    async def begin_disable(self, data):
        await super().begin_disable(data)
        self.disable_task.cancel()
        self.disable_task = asyncio.create_task(self.disable_devices())
        await self.disable_task

    @property
    def connected(self):
        return not (
            self.reader is None
            or self.writer is None
            or self.writer.is_closing()
            or self.reader.at_eof()
        )

    @staticmethod
    def get_config_pkg():
        return "ts_config_mttcs"

    async def close_tasks(self):
        """Shut down pending tasks. Called by `close`."""
        while self.command_dict:
            command = self.command_dict.popitem()[1]
            command.setnoack("Connection closed before command finished")
        await super().close_tasks()

        self.connect_task.cancel()
        self.enable_task.cancel()
        self.disable_task.cancel()
        self.monitor_telemetry_client_task.cancel()
        self.camera_cable_wrap_follow_start_task.cancel()
        self.camera_cable_wrap_follow_loop_task.cancel()
        self.read_loop_task.cancel()

        await self.disconnect()
        processes = self.terminate_background_processes()
        for process in processes:
            await process.wait()

    async def get_camera_cable_wrap_demand(self):
        """Get camera cable wrap tracking command data.

        Read the position and velocity of the camera rotator
        and compute an optimum demand for camera cable wrap.

        Returns
        -------
        position : `float`
            Desired camera cable wrap position (in degrees).
        velocity : `float`
            Desired camera cable wrap velocity (in degrees).
        tai : `float`
            Desired camera cable wrap time (TAI unix seconds).
            This will be ``config.camera_cable_wrap_advance_time``
            seconds in the future.

        Raises
        ------
        asyncio.TimeoutError
            If data not seen in ROTATOR_TELEMETRY_TIMEOUT seconds.
        """

        rot_data = await self.rotator.tel_rotation.next(
            flush=True, timeout=ROTATOR_TELEMETRY_TIMEOUT
        )

        desired_tai = salobj.current_tai() + self.config.camera_cable_wrap_advance_time
        dt = rot_data.timestamp - desired_tai

        if (
            abs(rot_data.demandPosition - rot_data.actualPosition)
            > self.config.max_rotator_position_error
        ):
            if not self.rotator_position_error_excessive:
                self.log.warning(
                    "Excessive rotator demand-actual position error; using actual. "
                    f"Demand={rot_data.demandPosition:0.3f}; "
                    f"actual={rot_data.actualPosition:0.3f}; "
                    f"max_rotator_position_error={self.config.max_rotator_position_error} deg."
                )
                self.rotator_position_error_excessive = True
            desired_position = rot_data.actualPosition
            desired_velocity = rot_data.actualVelocity
        else:
            self.rotator_position_error_excessive = False
            desired_position = rot_data.demandPosition
            desired_velocity = rot_data.demandVelocity

        max_velocity = limits.LimitsDict[enums.DeviceId.CAMERA_CABLE_WRAP].max_velocity

        if abs(desired_velocity) > max_velocity:
            excessive_desired_velocity = desired_velocity
            desired_velocity = math.copysign(max_velocity, desired_velocity)
            self.log.warning(
                f"Limiting desired velocity from {excessive_desired_velocity:0.2f} "
                f"to {desired_velocity:0.2f}"
            )

        adjusted_desired_position = desired_position + desired_velocity * dt
        return (adjusted_desired_position, desired_velocity, desired_tai)

    async def connect(self):
        """Connect to the low-level controller and start the telemetry client.
        """
        if self.config is None:
            raise RuntimeError("Not yet configured")
        if self.connected:
            raise RuntimeError("Already connected")

        if self.command_port is None or self.telemetry_port is None:
            raise RuntimeError(
                f"Bug: command_port={self.command_port} and/or "
                f"telemetry_port={self.telemetry_port} is None; "
                f"simulation_mode={self.simulation_mode}, "
                f"run_mock_controller={self.run_mock_controller}"
            )

        if self.simulation_mode == 0:
            command_host = self.config.host
            telemetry_host = self.config.telemetry_host
        else:
            command_host = salobj.LOCAL_HOST
            telemetry_host = salobj.LOCAL_HOST
        try:
            self.log.info(
                "Connecting to the low-level controller: "
                f"host={command_host}, port={self.command_port}"
            )
            connect_coro = asyncio.open_connection(
                host=command_host, port=self.command_port
            )
            self.reader, self.writer = await asyncio.wait_for(
                connect_coro, timeout=self.config.connection_timeout
            )
            self.should_be_connected = True
            self.read_loop_task = asyncio.create_task(self.read_loop())
            self.log.debug("Connected to the low-level controller")
        except Exception as e:
            err_msg = "Could not connect to the low-level controller: "
            f"host={command_host}, port={self.command_port}"
            self.log.exception(err_msg)
            self.fault(
                code=enums.CscErrorCode.COULD_NOT_CONNECT, report=f"{err_msg}: {e!r}"
            )
            return

        # Run the telemetry client as a background process.
        args = [
            "run_mtmount_telemetry_client.py",
            f"--host={telemetry_host}",
            f"--port={self.telemetry_port}",
            f"--loglevel={self.log.level}",
        ]
        self.log.info(f"Starting the telemetry client: {' '.join(args)!r}")
        try:
            self.telemetry_client_process = await asyncio.create_subprocess_exec(*args)
        except Exception as e:
            cmdstr = " ".join(args)
            err_msg = f"Could not start MTMount telemetry client with {cmdstr!r}"
            self.log.exception(err_msg)
            self.fault(
                code=enums.CscErrorCode.TELEMETRY_CLIENT_ERROR,
                report=f"{err_msg}: {e!r}",
            )
            return
        try:
            await self.mtmount_remote.tel_cameraCableWrap.next(
                flush=False, timeout=TELEMETRY_START_TIMEOUT
            )
        except asyncio.TimeoutError:
            err_msg = "The telemetry client is not producing telemetry"
            self.log.error(err_msg)
            self.fault(code=enums.CscErrorCode.TELEMETRY_CLIENT_ERROR, report=err_msg)
        self.monitor_telemetry_client_task = asyncio.create_task(
            self.monitor_telemetry_client()
        )

    async def _wait_for_mock_controller(self):
        """Wait for the mock controller to be running.

        Returns
        -------
        command_port : `int`
            The command port.
        telemetry_port : `int`
            The telemetry port.
        """
        # Sample data:
        # Mock TMA controller running: command_port=40827, telemetry_port=33133
        running_regex = re.compile(
            r"running: command_port=(\d+), telemetry_port=(\d+)$"
        )
        while True:
            if self.mock_controller_process.returncode is not None:
                raise RuntimeError("Mock controller process failed")
            line_bytes = await self.mock_controller_process.stdout.readline()
            line_str = line_bytes.decode().strip()
            self.log.debug("Mock controller: %s", line_str)
            match = running_regex.search(line_str)
            if match is not None:
                return int(match[1]), int(match[2])

    async def disconnect(self):
        """Disconnect from the low-level controller.

        Close the connection to the mock controller, if connected.
        Stop the telemetry process, if running.
        Do not stop the mock controller (if the CSC started it),
        because that should remain running until the CSC quits.
        """
        self.should_be_connected = False

        self.monitor_telemetry_client_task.cancel()

        if self.writer is not None:
            self.log.info("Disconnect from the low-level controller")
            writer = self.writer
            self.writer = None
            writer.close()
            # In Python 3.8.6 writer.wait_closed may hang indefinitely
            try:
                await asyncio.wait_for(writer.wait_closed(), timeout=1)
            except asyncio.TimeoutError:
                self.log.warning(
                    "Timed out waiting for the writer to close; continuing"
                )

        # Kill the telemetry process
        if (
            self.telemetry_client_process is not None
            and self.telemetry_client_process.returncode is None
        ):
            self.log.info("Terminate the telemetry subprocess")
            self.telemetry_client_process.terminate()
            await self.telemetry_client_process.wait()

    async def enable_devices(self):
        self.log.info("Enable devices")
        self.disable_task.cancel()
        try:
            for reset_command in [
                commands.TopEndChillerResetAlarm(),
                commands.MainPowerSupplyResetAlarm(),
                commands.MirrorCoverLocksResetAlarm(),
                commands.MirrorCoversResetAlarm(),
                commands.AzimuthAxisResetAlarm(),
                commands.ElevationAxisResetAlarm(),
                commands.CameraCableWrapResetAlarm(),
            ]:
                try:
                    await self.send_command(reset_command, do_lock=False)
                except Exception as e:
                    self.log.warning(
                        f"Command {reset_command} failed; continuing: {e!r}"
                    )

            power_on_commands = [
                commands.TopEndChillerPower(on=True),
                commands.TopEndChillerTrackAmbient(on=True, temperature=0),
                commands.MainPowerSupplyPower(on=True),
                commands.OilSupplySystemPower(on=True),
                commands.AzimuthAxisPower(on=True),
                commands.ElevationAxisPower(on=True),
                commands.CameraCableWrapPower(on=True),
            ]
            await self.send_commands(*power_on_commands)
        except Exception as e:
            self.log.error(f"Failed to power on one or more devices: {e!r}")
            raise
        self.camera_cable_wrap_follow_start_task = asyncio.create_task(
            self.camera_cable_wrap_start_following()
        )
        await self.camera_cable_wrap_follow_start_task

    async def disable_devices(self):
        """Disable all devices and yield control."""
        self.log.info("Disable devices")
        self.enable_task.cancel()
        self.camera_cable_wrap_follow_start_task.cancel()
        self.camera_cable_wrap_follow_loop_task.cancel()
        if not self.connected:
            return
        for command in [
            commands.BothAxesStop(),
            commands.CameraCableWrapStop(),
            commands.AzimuthAxisPower(on=False),
            commands.ElevationAxisPower(on=False),
            commands.CameraCableWrapPower(on=False),
        ]:
            try:
                await self.send_command(command, do_lock=False)
            except Exception as e:
                self.log.warning(f"Command {command} failed; continuing: {e!r}")

        self.evt_axesInPosition.set_put(azimuth=False, elevation=False)

        try:
            self.log.info("Give up command of the mount.")
            await self.send_command(commands.AskForCommand(commander=enums.Source.NONE))
            self.has_control = False
        except Exception as e:
            self.log.warning(
                f"The CSC was not able to give up command of the mount: {e!r}"
            )

    async def handle_summary_state(self):
        if self.disabled_or_enabled:
            if not self.connected:
                self.connect_task.cancel()
                self.connect_task = asyncio.create_task(self.connect())
                await self.connect_task
        else:
            await self.disconnect()

    async def send_command(self, command, do_lock=True):
        """Send a command to the operation manager and wait for it to finish.

        Parameters
        ----------
        command : `Command`
            Command to send.
        do_lock : `bool`, optional
            Lock the port while using it?
            Specify False for emergency commands
            or if being called by send_commands.

        Returns
        -------
        command_futures : `command_futures.CommandFutures`
            Futures that monitor the command.
        """
        try:
            if do_lock:
                async with self.command_lock:
                    return await self._basic_send_command(command=command)
            else:
                return await self._basic_send_command(command=command)
        except ConnectionResetError:
            raise
        except Exception as e:
            self.log.exception(f"Failed to send command {command}: {e!r}")
            raise

    async def _basic_send_command(self, command):
        """Implementation of send_command. Ignores the command lock.

        Parameters
        ----------
        command : `Command`
            Command to send.

        Returns
        -------
        command_futures : `command_futures.CommandFutures`
            Futures that monitor the command.
        """
        if not self.connected:
            raise salobj.ExpectedError("Not connected to the low-level controller.")
        if command.sequence_id in self.command_dict:
            raise RuntimeError(
                f"Bug! Duplicate sequence_id {command.sequence_id} in command_dict"
            )
        futures = command_futures.CommandFutures()
        self.command_dict[command.sequence_id] = futures
        self.writer.write(command.encode())
        await self.writer.drain()
        timeout = await asyncio.wait_for(futures.ack, self.config.ack_timeout)
        if timeout < 0:
            # This command only receives an Ack; mark it done.
            futures.done.set_result(None)
        elif not futures.done.done():
            try:
                await asyncio.wait_for(futures.done, timeout=timeout + TIMEOUT_BUFFER)
            except asyncio.TimeoutError:
                self.command_dict.pop(command.sequence_id, None)
                raise asyncio.TimeoutError(
                    f"Timed out after {timeout + TIMEOUT_BUFFER} seconds "
                    f"waiting for the Done reply to {command}"
                )
        return futures

    async def send_commands(self, *commands, do_lock=True):
        """Run a set of operation manager commands.

        Wait for each command to finish before issuing the next.

        Parameters
        ----------
        commands : `List` [``Command``]
            Commands to send. The sequence_id attribute is set.
        do_lock : `bool`, optional
            Lock the port while using it?
            Specify False for emergency commands.
        """
        future = None
        try:
            if do_lock:
                async with self.command_lock:
                    for command in commands:
                        future = await self.send_command(command, do_lock=False)
                        await future.done
            else:
                for command in commands:
                    future = await self.send_command(command, do_lock=False)
                    await future.done
        except ConnectionResetError:
            if future is not None:
                future.setnoack("Connection lost")
        except Exception as e:
            err_msg = f"send_commands failed: {e!r}"
            self.log.exception(err_msg)
            # The future is probably done, but in case not...
            if future is not None:
                future.setnoack(err_msg)
            raise

    def terminate_background_processes(self):
        """Terminate the background processes with SIGTERM.

        Return the processes that were terminated
        so you can await them.
        """
        processes = []
        if (
            self.mock_controller_process is not None
            and self.mock_controller_process.returncode is None
        ):
            self.log.info("Terminate the mock controller process")
            self.mock_controller_process.terminate()
            processes.append(self.mock_controller_process)
            self.mock_controller_process = None
        if (
            self.telemetry_client_process is not None
            and self.telemetry_client_process.returncode is None
        ):
            self.log.info("Terminate the telemetry client")
            self.telemetry_client_process.terminate()
            processes.append(self.telemetry_client_process)
            self.telemetry_client_process = None
        return processes

    async def camera_cable_wrap_start_following(self):
        """Make the camera cable wrap start following the camera rotator.

        Camera cable wrap following is divided into two tasks so that the
        enableCameraCableWrapFollowing command can succeed or fail
        if following is likely or unlikely to work.

        This method enables camera cable wrap tracking,
        waits for that command to succeed or fail,
        and if it succeeds, starts the camera cable wrap following loop.
        """
        self.camera_cable_wrap_follow_loop_task.cancel()
        try:
            await self.send_command(commands.CameraCableWrapEnableTracking(on=True))
            self.camera_cable_wrap_follow_loop_task = asyncio.create_task(
                self._camera_cable_wrap_follow_loop()
            )
            self.evt_cameraCableWrapFollowing.set_put(enabled=True, force_output=True)
        except asyncio.CancelledError:
            self.log.info("Camera cable wrap following canceled before it starts")
            self.evt_cameraCableWrapFollowing.set_put(enabled=False)
            raise
        except Exception as e:
            if isinstance(e, salobj.ExpectedError):
                self.log.error(
                    f"Camera cable wrap tracking could not be enabled: {e!r}"
                )
            else:
                self.log.exception("Camera cable wrap tracking could not be enabled")
            self.evt_cameraCableWrapFollowing.set_put(enabled=False)
            raise

    async def _camera_cable_wrap_follow_loop(self):
        """Implement the camera cable wrap following the camera rotator.

        This should be called by camera_cable_wrap_start_following.
        Camera cable wrap tracking must be enabled before this is called.
        """
        self.log.info("Camera cable wrap following begins")
        self.rotator_position_error_excessive = False
        paused = False
        try:
            while True:
                try:
                    position_velocity_tai = await self.get_camera_cable_wrap_demand()
                except asyncio.TimeoutError:
                    if not paused:
                        paused = True
                        self.log.warning(
                            "Rotator data not available; stopping the camera "
                            "cable wrap until rotator data is available"
                        )
                        await self.send_command(commands.CameraCableWrapStop())
                    continue
                if paused:
                    paused = False
                    self.log.info(
                        "Rotator data received; resume making "
                        "the camera cable wrap follow the rotator"
                    )
                    await self.send_command(
                        commands.CameraCableWrapEnableTracking(on=True)
                    )

                position, velocity, tai = position_velocity_tai
                command = commands.CameraCableWrapTrack(
                    position=position, velocity=velocity, tai=tai,
                )
                await self.send_command(command)
                self.evt_cameraCableWrapTarget.set_put(
                    position=position, velocity=velocity, taiTime=tai
                )
                await asyncio.sleep(self.config.camera_cable_wrap_interval)

        except asyncio.CancelledError:
            self.log.info("Camera cable wrap following ends")
        except salobj.ExpectedError as e:
            self.log.error(f"Camera cable wrap following failed: {e!r}")
        except Exception:
            self.log.exception("Camera cable wrap following failed")
        finally:
            self.evt_cameraCableWrapFollowing.set_put(enabled=False)

    async def configure(self, config):
        self.config = config

    async def monitor_telemetry_client(self):
        await self.telemetry_client_process.wait()
        self.fail("Telemetry process exited prematurely")

    async def read_loop(self):
        """Read and process replies from the low-level controller.
        """
        self.log.debug("Read loop begins")
        cmd_reply_codes = frozenset(
            (
                enums.ReplyCode.CMD_ACKNOWLEDGED,
                enums.ReplyCode.CMD_REJECTED,
                enums.ReplyCode.CMD_SUCCEEDED,
                enums.ReplyCode.CMD_FAILED,
                enums.ReplyCode.CMD_SUPERSEDED,
            )
        )
        failed_reply_codes = frozenset(
            (enums.ReplyCode.CMD_REJECTED, enums.ReplyCode.CMD_FAILED)
        )
        while self.should_be_connected and self.connected:
            try:
                read_bytes = await self.reader.readuntil(constants.LINE_TERMINATOR)
                try:
                    reply = json.loads(read_bytes)
                    self.log.debug("Read %s", reply)
                except Exception as e:
                    self.log.warning(f"Ignoring unparsable reply: {read_bytes}: {e!r}")

                reply_id = reply["id"]
                if reply_id in cmd_reply_codes:
                    sequence_id = reply["parameters"]["sequenceId"]
                    if reply_id == enums.ReplyCode.CMD_ACKNOWLEDGED:
                        futures = self.command_dict.get(sequence_id, None)
                    else:
                        futures = self.command_dict.pop(sequence_id, None)
                    if futures is None:
                        self.log.warning(
                            f"Got reply with code {enums.ReplyCode(reply_id)!r} "
                            f"for non-existent command {sequence_id}"
                        )
                        continue
                    if reply_id == enums.ReplyCode.CMD_ACKNOWLEDGED:
                        # Command acknowledged. Set timeout but leave
                        # futures in command_dict.
                        futures.setack(reply["parameters"]["timeout"])
                    elif reply_id in failed_reply_codes:
                        # Command failed (before or after being acknowledged).
                        # Pop the command_dict entry and report failure.
                        futures.setnoack(reply["parameters"]["explanation"])
                    elif reply_id == enums.ReplyCode.CMD_SUCCEEDED:
                        futures.setdone()
                    elif reply_id == enums.ReplyCode.CMD_SUPERSEDED:
                        futures.setnoack("Superseded")
                    else:
                        raise RuntimeError(f"Bug: unsupported reply code {reply_id}")
                elif reply_id == enums.ReplyCode.WARNING:
                    self.evt_warning.set_put(
                        code=reply["parameters"]["code"],
                        active=reply["parameters"]["active"],
                        text="\n".join(reply["parameters"]["description"]),
                        force_output=True,
                    )
                elif reply_id == enums.ReplyCode.ERROR:
                    self.evt_error.set_put(
                        code=reply["parameters"]["code"],
                        latched=reply["parameters"]["on"],
                        active=reply["parameters"]["active"],
                        text="\n".join(reply["parameters"]["description"]),
                        force_output=True,
                    )
                elif reply_id == enums.ReplyCode.IN_POSITION:
                    axis = reply["parameters"]["axis"]
                    in_position = reply["parameters"]["inPosition"]
                    if axis == 0:
                        self.evt_axesInPosition.set_put(azimuth=in_position)
                    elif axis == 1:
                        self.evt_axesInPosition.set_put(elevation=in_position)
                    else:
                        self.log.warning(
                            f"Unrecognized axis={axis} in IN_POSITION reply"
                        )
                elif reply_id == enums.ReplyCode.STATE_INFO:
                    pass
                else:
                    self.log.warning(f"Ignoring unrecognized reply: {reply}")
            except asyncio.CancelledError:
                return
            except Exception as e:
                if self.should_be_connected:
                    if self.connected:
                        err_msg = "Read loop failed; possibly a bug"
                        self.log.exception(err_msg)
                        self.fault(
                            code=enums.CscErrorCode.INTERNAL_ERROR,
                            report=f"{err_msg}: {e!r}",
                        )
                    else:
                        self.fault(
                            code=enums.CscErrorCode.CONNECTION_LOST,
                            report="Connection lost to low-level controller (noticed in read_loop)",
                        )
        self.log.debug("Read loop ends")

    def signal_handler(self):
        self.terminate_background_processes()
        asyncio.create_task(self.close())

    async def start(self):
        print(
            f"start: self.simulation_mode={self.simulation_mode!r}; "
            f"self.run_mock_controller={self.run_mock_controller}"
        )

        if self.simulation_mode == 1 and self.run_mock_controller:
            # Run the mock controller using random ports;
            # read the ports to set command_port and telemetry_port.
            args = [
                "run_mock_tma.py",
                f"--loglevel={self.log.level}",
                "--random-ports",
            ]
            self.log.info(f"Starting the mock controller: {' '.join(args)}")
            try:
                self.mock_controller_process = await asyncio.create_subprocess_exec(
                    *args, stdout=subprocess.PIPE, stdin=subprocess.DEVNULL
                )
            except Exception as e:
                cmdstr = " ".join(args)
                err_msg = f"Mock controller process command {cmdstr!r} failed"
                self.log.exception(err_msg)
                self.fault(
                    code=enums.CscErrorCode.MOCK_CONTROLLER_ERROR,
                    report=f"{err_msg}: {e!r}",
                )
                return

            self.log.info("Waiting for mock controller to start")
            t0 = salobj.current_tai()
            self.command_port, self.telemetry_port = await asyncio.wait_for(
                self._wait_for_mock_controller(), timeout=MOCK_CTRL_START_TIMEOUT,
            )
            dt = salobj.current_tai() - t0
            self.log.info(
                f"Mock controller running with command_port={self.command_port} "
                f"telemetry_port={self.telemetry_port}; {dt:0.1f} seconds to start"
            )

        await asyncio.gather(self.rotator.start_task, self.mtmount_remote.start_task)
        self.evt_cameraCableWrapFollowing.set_put(enabled=False)

        await super().start()

    async def do_clearError(self, data):
        self.assert_enabled()
        raise salobj.ExpectedError("Not yet implemented")

    async def do_closeMirrorCovers(self, data):
        self.assert_enabled()
        # Deploy the mirror cover locks/guides.
        await self.send_commands(
            commands.MirrorCoverLocksPower(on=True),
            commands.MirrorCoverLocksMoveAll(deploy=True),
            commands.MirrorCoverLocksPower(on=False),
        )
        # Deploy the mirror covers.
        await self.send_commands(
            commands.MirrorCoversPower(on=True),
            commands.MirrorCoversDeploy(),
            commands.MirrorCoversPower(on=False),
        )

    async def do_openMirrorCovers(self, data):
        self.assert_enabled()
        # Retract the mirror covers.
        await self.send_commands(
            commands.MirrorCoversPower(on=True),
            commands.MirrorCoversRetract(),
            commands.MirrorCoversPower(on=False),
        )
        # Retract the mirror cover locks/guides.
        await self.send_commands(
            commands.MirrorCoverLocksPower(on=True),
            commands.MirrorCoverLocksMoveAll(deploy=False),
            commands.MirrorCoverLocksPower(on=False),
        )

    async def do_disableCameraCableWrapFollowing(self, data):
        self.assert_enabled()
        self.camera_cable_wrap_follow_start_task.cancel()
        self.camera_cable_wrap_follow_loop_task.cancel()
        await self.send_command(commands.CameraCableWrapStop())

    async def do_enableCameraCableWrapFollowing(self, data):
        self.assert_enabled()
        if self.camera_cable_wrap_follow_loop_task.done():
            self.camera_cable_wrap_follow_start_task = asyncio.create_task(
                self.camera_cable_wrap_start_following()
            )
            await self.camera_cable_wrap_follow_start_task

    async def do_moveToTarget(self, data):
        self.assert_enabled()
        await self.send_command(
            commands.BothAxesMove(azimuth=data.azimuth, elevation=data.elevation),
        )

    async def do_trackTarget(self, data):
        self.assert_enabled()
        await self.send_command(
            commands.BothAxesTrack(
                azimuth=data.azimuth,
                azimuth_velocity=data.azimuthVelocity,
                elevation=data.elevation,
                elevation_velocity=data.elevationVelocity,
                tai=data.taiTime,
            ),
        )
        self.evt_target.set_put(
            azimuth=data.azimuth,
            elevation=data.elevation,
            azimuthVelocity=data.azimuthVelocity,
            elevationVelocity=data.elevationVelocity,
            taiTime=data.taiTime,
            trackId=data.trackId,
            tracksys=data.tracksys,
            radesys=data.radesys,
            force_output=True,
        )

    async def do_startTracking(self, data):
        self.assert_enabled()
        await self.send_commands(
            commands.ElevationAxisEnableTracking(),
            commands.AzimuthAxisEnableTracking(),
        )

    async def do_stop(self, data):
        self.assert_enabled()
        await self.send_commands(
            commands.BothAxesStop(),
            commands.CameraCableWrapStop(),
            commands.MirrorCoverLocksStop(),
            commands.MirrorCoversStop(),
            do_lock=False,
        )

    async def do_stopTracking(self, data):
        self.assert_enabled()
        await self.send_command(commands.BothAxesStop())
