# This file is part of ts_mtmount.
#
# Developed for Rubin Observatory Telescope and Site Systems.
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

__all__ = ["MTMountCsc", "run_mtmount"]

import asyncio
import functools
import inspect
import json
import math
import re
import signal
import subprocess
import types

from lsst.ts import utils
from lsst.ts import salobj
from lsst.ts.idl.enums.MTMount import AxisMotionState, PowerState, System
from .config_schema import CONFIG_SCHEMA
from .utils import truncate_value
from . import constants
from . import command_futures
from . import commands
from . import enums
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

# Maximum time (seconds) to fully deploy or retract the mirror covers,
# including dealing with the mirror cover locks.
# 60 is the maximum allowed by our requirements,
# so it is not necessarily accurate.
# If Tekniker provides a single command to handle the mirror covers
# then this will no longer be necessary; we can use the timeout provided
# by the low-level controller, which is likely to be more accurate.
MIRROR_COVER_TIMEOUT = 60


class SystemStateInfo:
    """Information about a system state topic.

    Constructing this object has a side effect:
    it initializes the fields of the topic to unknown/nan.

    Parameters
    ----------
    topic : `salobj.topics.ControllerEvent`
        System state topic.

    Attributes
    ----------
    num_elements_power_state : int
        Length of elementsPowerState; 0 if absent
    num_motion_controller_state : int
        Length of motionControllerState; 0 if absent
    num_thermal : int
        Length of trackAmbient and setTemperature; 0 if absent
    """

    def __init__(self, topic: salobj.topics.ControllerEvent) -> None:
        self.topic = topic
        data = topic.DataType()
        if hasattr(data, "powerState"):
            topic.set(powerState=PowerState.UNKNOWN)
        eps = getattr(data, "elementsPowerState", None)
        if eps is None:
            self.num_elements_power_state = 0
        else:
            self.num_elements_power_state = len(eps)
            topic.set(
                elementsPowerState=[PowerState.UNKNOWN] * self.num_elements_power_state
            )
        mcs = getattr(data, "motionControllerState", None)
        if mcs is not None:
            self.num_motion_controller_state = len(mcs)
            topic.set(
                motionControllerState=[PowerState.UNKNOWN]
                * self.num_motion_controller_state
            )
        else:
            self.num_motion_controller_state = 0
        ta = getattr(data, "trackAmbient", None)
        if ta is not None:
            if isinstance(ta, bool):
                self.num_thermal = 1
                topic.set(
                    trackAmbient=False,
                    setTemperature=math.nan,
                )
            else:
                self.num_thermal = len(ta)
                topic.set(
                    trackAmbient=[False] * self.num_thermal,
                    setTemperature=[math.nan] * self.num_thermal,
                )
        else:
            self.num_thermal = 0


class MTMountCsc(salobj.ConfigurableCsc):
    """MTMount CSC

    Parameters
    ----------
    initial_state : `salobj.State` or `int`, optional
        The initial state of the CSC. This is provided for unit testing,
        as real CSCs should start up in `lsst.ts.salobj.StateSTANDBY`,
        the maxMove.
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

    # For the rotator following loop: commanded position and velocity
    # must be within this margin of the command limits.
    # The value is based on the fact that the low-level controller
    # reports all limits rounded to 2 decimal places.
    limits_margin = 0.01

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
            telemetry_port = constants.TELEMETRY_PORT
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

        # TODO DM-36445: remove this flag and assume evt_connected is modern
        self.has_modern_connected_event = False

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

        # Reply IDs that this code does not handle.
        self.unknown_reply_ids = set()

        self.command_lock = asyncio.Lock()

        # Does the CSC have control of the mount?
        # Once the low-level controller reports this in an event,
        # add a SAL event and get the value from that.
        self.has_control = False

        # Task that waits while connecting to the TCP/IP controller.
        self.connect_task = utils.make_done_future()

        # Task to track enabling and disabling
        self.enable_task = utils.make_done_future()
        self.disable_task = utils.make_done_future()

        self.mock_controller_started_task = asyncio.Future()
        self.monitor_telemetry_client_task = utils.make_done_future()

        # Tasks for camera cable wrap following the rotator
        self.camera_cable_wrap_follow_start_task = utils.make_done_future()
        self.camera_cable_wrap_follow_loop_task = utils.make_done_future()

        self.read_loop_task = utils.make_done_future()
        self.send_heartbeat_loop_task = utils.make_done_future()

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

        # TODO DM-36879: remove this and rely on GetActualSettings
        # to return the data.
        self.evt_cameraCableWrapControllerSettings.set(
            maxCmdVelocity=5.6,
            minCmdPosition=-90,
            maxCmdPosition=90,
        )

        # Dict of system ID: SystemStateInfo;
        # this provides useful information for handling replies and also
        # initializes the relevant fields of the events. Initialization is
        # useful because most of these topics are set by more than one reply
        # (POWER_STATE plus CHILLER_STATE and/or MOTION_CONTROLLER_STATE),
        # so they will be output before all fields are known.
        self.system_state_dict = {}
        for system_id, topic in (
            (System.AZIMUTH, self.evt_azimuthSystemState),
            (System.ELEVATION, self.evt_elevationSystemState),
            (System.CAMERA_CABLE_WRAP, self.evt_cameraCableWrapSystemState),
            (System.BALANCE, self.evt_balanceSystemState),
            (System.MIRROR_COVERS, self.evt_mirrorCoversSystemState),
            (System.MIRROR_COVER_LOCKS, self.evt_mirrorCoverLocksSystemState),
            (System.AZIMUTH_CABLE_WRAP, self.evt_azimuthCableWrapSystemState),
            (System.LOCKING_PINS, self.evt_lockingPinsSystemState),
            (System.DEPLOYABLE_PLATFORMS, self.evt_deployablePlatformsSystemState),
            (System.OIL_SUPPLY_SYSTEM, self.evt_oilSupplySystemState),
            (System.AZIMUTH_DRIVES_THERMAL, self.evt_azimuthDrivesThermalSystemState),
            (
                System.ELEVATION_DRIVES_THERMAL,
                self.evt_elevationDrivesThermalSystemState,
            ),
            (System.AZ0101_CABINET_THERMAL, self.evt_az0101CabinetThermalSystemState),
            (
                System.MODBUS_TEMPERATURE_CONTROLLERS,
                self.evt_modbusTemperatureControllersSystemState,
            ),
            (System.MAIN_CABINET_THERMAL, self.evt_mainCabinetThermalSystemState),
            (System.MAIN_AXES_POWER_SUPPLY, self.evt_mainAxesPowerSupplySystemState),
            (System.TOP_END_CHILLER, self.evt_topEndChillerSystemState),
        ):
            self.system_state_dict[system_id] = SystemStateInfo(topic)

        # Dict of ReplyId: function to call.
        # The function receives one argument: the reply.
        # Set this after calling super().__init__ so the events are available.
        self.reply_dispatch = {
            enums.ReplyId.AVAILABLE_SETTINGS: self.handle_available_settings,
            enums.ReplyId.AXIS_MOTION_STATE: self.handle_axis_motion_state,
            enums.ReplyId.AZIMUTH_TOPPLE_BLOCK: self.handle_azimuth_topple_block,
            enums.ReplyId.CHILLER_STATE: self.handle_chiller_state,
            enums.ReplyId.CMD_ACKNOWLEDGED: self.handle_command_reply,
            enums.ReplyId.CMD_REJECTED: self.handle_command_reply,
            enums.ReplyId.CMD_SUCCEEDED: self.handle_command_reply,
            enums.ReplyId.CMD_FAILED: self.handle_command_reply,
            enums.ReplyId.CMD_SUPERSEDED: self.handle_command_reply,
            enums.ReplyId.COMMANDER: self.handle_commander,
            enums.ReplyId.DEPLOYABLE_PLATFORMS_MOTION_STATE: functools.partial(
                self.handle_deployable_motion_state,
                topic=self.evt_deployablePlatformsMotionState,
            ),
            enums.ReplyId.DETAILED_SETTINGS_APPLIED: self.handle_detailed_settings_applied,
            enums.ReplyId.ELEVATION_LOCKING_PIN_MOTION_STATE: self.handle_elevation_locking_pin_motion_state,
            enums.ReplyId.ERROR: self.handle_error,
            enums.ReplyId.IN_POSITION: self.handle_in_position,
            enums.ReplyId.LIMITS: self.handle_limits,
            enums.ReplyId.MIRROR_COVER_LOCKS_MOTION_STATE: functools.partial(
                self.handle_deployable_motion_state,
                topic=self.evt_mirrorCoverLocksMotionState,
            ),
            enums.ReplyId.MIRROR_COVERS_MOTION_STATE: functools.partial(
                self.handle_deployable_motion_state,
                topic=self.evt_mirrorCoversMotionState,
            ),
            enums.ReplyId.MOTION_CONTROLLER_STATE: self.handle_motion_controller_state,
            enums.ReplyId.POWER_STATE: self.handle_power_state,
            enums.ReplyId.SAFETY_INTERLOCKS: self.handle_safety_interlocks,
            enums.ReplyId.WARNING: self.handle_warning,
        }
        # Make sure all the dispatchers are coroutines, to catch future
        # code errors. The check for an instance of partial is needed with
        # Python 3.8 and older, based on https://stackoverflow.com/a/52422903.
        for key, value in self.reply_dispatch.items():
            if isinstance(value, functools.partial):
                coro = value.func
            else:
                coro = value
            if not inspect.iscoroutinefunction(coro):
                raise RuntimeError(
                    f"Bug: the dispatch function for ReplyId={key!r} is not a coroutine"
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
        self.evt_simulationMode.set(mode=simulation_mode)

    async def begin_enable(self, data):
        """Take control of the mount and initialize devices.

        If taking control fails, raise an exception in order to leave
        the state as DISABLED.

        If initializing fails, first try to disable the axis controllers,
        then raise an exception in order to leave the state as DISABLED.
        """
        self.disable_task.cancel()
        self.enable_task.cancel()
        await super().begin_enable(data)
        if not self.has_control:
            try:
                self.log.info("Ask for permission to command the mount.")
                await self.send_command(
                    commands.AskForCommand(commander=enums.Source.CSC), do_lock=True
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
        """Disable the axis controllers and yield control.

        Leave the top end chiller and oils supply system running.
        """
        await super().begin_disable(data)
        self.disable_task.cancel()
        self.disable_task = asyncio.create_task(self.disable_devices())
        await self.disable_task

    @property
    def connected(self):
        """Return True if connected to the low-level controller."""
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
        self.send_heartbeat_loop_task.cancel()

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

        desired_tai = utils.current_tai() + self.config.camera_cable_wrap_advance_time
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

        # List of warning strings about truncated position and velocity
        # (in that order).
        truncation_warnings = []

        ccw_settings_data = self.evt_cameraCableWrapControllerSettings.data
        desired_velocity, warning_message = truncate_value(
            value=desired_velocity,
            min_value=-ccw_settings_data.maxCmdVelocity + self.limits_margin,
            max_value=ccw_settings_data.maxCmdVelocity - self.limits_margin,
            descr="velocity",
        )
        if warning_message:
            truncation_warnings.append(warning_message)

        # Adjust the desired position for the time offset,
        # then truncate it to be in bounds.
        adjusted_desired_position = desired_position + desired_velocity * dt

        adjusted_desired_position, warning_message = truncate_value(
            value=adjusted_desired_position,
            min_value=ccw_settings_data.minCmdPosition + self.limits_margin,
            max_value=ccw_settings_data.maxCmdPosition - self.limits_margin,
            descr="position",
        )
        if warning_message:
            truncation_warnings.insert(0, warning_message)

        if truncation_warnings:
            self.log.warning(
                "Limiting camera cable wrap commanded "
                + " and ".join(truncation_warnings)
            )

        return (adjusted_desired_position, desired_velocity, desired_tai)

    async def connect(self):
        """Connect to the low-level controller and start the telemetry
        client.
        """
        if self.has_modern_connected_event:
            await self.evt_connected.set_write(connected=self.connected)

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
            self.send_heartbeat_loop_task = asyncio.create_task(
                self.send_heartbeat_loop()
            )
            self.read_loop_task = asyncio.create_task(self.read_loop())
            self.log.debug("Connection made; requesting current state")
            await self.send_command(commands.StateInfo(), do_lock=True)
            # TODO DM-36879: enable this when the TMA command works:
            if False:
                await self.send_command(commands.GetActualSettings(), do_lock=True)
            self.log.debug("Connected to the low-level controller")
        except Exception as e:
            err_msg = "Could not connect to the low-level controller: "
            f"host={command_host}, port={self.command_port}"
            self.log.exception(err_msg)
            await self.fault(
                code=enums.CscErrorCode.COULD_NOT_CONNECT, report=f"{err_msg}: {e!r}"
            )
            return

        if self.has_modern_connected_event:
            await self.evt_connected.set_write(connected=self.connected)

        # Run the telemetry client as a background process.
        args = [
            "run_mtmount_telemetry_client",
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
            await self.fault(
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
            await self.fault(
                code=enums.CscErrorCode.TELEMETRY_CLIENT_ERROR, report=err_msg
            )
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

        Close the connection to the low-level controller, if connected.
        Clear the target event.
        Stop the telemetry process, if running.
        Do not stop the mock controller (even if the CSC started it),
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

        if self.has_modern_connected_event:
            await self.evt_connected.set_write(connected=self.connected)

        await self.clear_target()

        # Kill the telemetry process
        if (
            self.telemetry_client_process is not None
            and self.telemetry_client_process.returncode is None
        ):
            self.log.info("Terminate the telemetry subprocess")
            self.telemetry_client_process.terminate()
            await self.telemetry_client_process.wait()
            self.log.info("Telemetry subprocess terminated")

    async def enable_devices(self):
        """Enable all devices.

        Call this when going to ENABLED state.
        """
        self.log.info("Enable devices")
        self.disable_task.cancel()
        try:
            # Reset commands will fail if the subsystem is already on.
            # Also they will claim to succeed if the subsystem is off
            # or in standby, EVEN IF THE RESET ACTUALLY FAILS.
            # Thus the only reason a reset command can fail is if
            # the subsystem is already on, so we can basically ignore
            # such failures. It's a bit skanky, but avoids a race condition
            # in checking the subsystem power state and using that
            # to decide whether to reset alarms.
            for command, must_succeed in (
                (commands.MainCabinetThermalResetAlarm(), False),
                # Disabled for 2022-10 commissioning
                # (commands.TopEndChillerResetAlarm(), False),
                # (commands.OilSupplySystemResetAlarm(), False),
                (commands.MainAxesPowerSupplyResetAlarm(), False),
                (commands.MirrorCoverLocksResetAlarm(), False),
                (commands.MirrorCoversResetAlarm(), False),
                (commands.CameraCableWrapResetAlarm(), False),
                # Disabled for 2022-10 commissioning
                # (commands.TopEndChillerPower(on=True), True),
                # (commands.TopEndChillerTrackAmbient(
                #     on=True, temperature=0), True),
                (commands.MainAxesPowerSupplyPower(on=True), True),
                # Disabled for 2022-10 commissioning
                # (commands.OilSupplySystemPower(on=True), True),
                (commands.BothAxesResetAlarm(), True),
                (commands.BothAxesPower(on=True), True),
                (commands.CameraCableWrapPower(on=True), True),
            ):
                try:
                    await self.send_command(command, do_lock=True)
                except Exception as e:
                    if must_succeed:
                        raise salobj.ExpectedError(f"Command {command} failed: {e!r}")
                    else:
                        self.log.info(
                            f"Reset command {command} failed; the subsystem is probably already on"
                        )
        except Exception as e:
            self.log.error(f"Failed to power on one or more devices: {e!r}")
            raise
        self.camera_cable_wrap_follow_start_task = asyncio.create_task(
            self.start_camera_cable_wrap_following()
        )
        await self.camera_cable_wrap_follow_start_task

    async def disable_devices(self):
        """Disable the axis controllers and yield control.

        Leave the top end chiller and oils supply system running.

        Call this when going to DISABLED state.
        """
        self.log.info("Disable devices")
        self.enable_task.cancel()
        await self.stop_camera_cable_wrap_following()
        if not self.connected:
            return
        for command in [
            commands.BothAxesStop(),
            commands.CameraCableWrapStop(),
            commands.AzimuthPower(on=False),
            commands.ElevationPower(on=False),
            commands.CameraCableWrapPower(on=False),
        ]:
            try:
                await self.send_command(command, do_lock=True)
            except Exception as e:
                self.log.warning(f"Command {command} failed; continuing: {e!r}")

        await self.evt_elevationInPosition.set_write(inPosition=False)
        await self.evt_azimuthInPosition.set_write(inPosition=False)

        try:
            self.log.info("Give up command of the mount.")
            await self.send_command(
                # Give control to EUI so the TMA keeps receiving heartbeats;
                # this prevents it from shutting off the oil supply system,
                # main axes power supply, and top-end chiller.
                # TODO DM-35194: if we get Tekniker to change the behavior when
                # there is no heartbeat, change this to enums.Source.NONE.
                commands.AskForCommand(commander=enums.Source.EUI),
                do_lock=True,
                wait_done=True,
            )
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

    async def send_command(self, command, do_lock, wait_done=True):
        """Send a command to the operation manager and wait for it to finish.

        Parameters
        ----------
        command : `Command`
            Command to send.
        do_lock : `bool`
            Lock the port while this method runs?
            The low-level controller has ill-defined limits on which commands
            commands can run simultaneously, so specify True when practical.
            Specify False for stop commands and the camera cable wrap
            tracking command (so rotator following is not blocked).
        wait_done : `bool`, optional
            Wait for the command to finish?
            If False then only wait for the command to be acknowledged;
            if ``do_lock`` true then release the lock when acknowledged.
            Note that a few commands are done when acknowledged;
            this argument has no effect for those.

        Returns
        -------
        command_futures : `command_futures.CommandFutures`
            Futures that monitor the command.
        """
        try:
            if do_lock:
                async with self.command_lock:
                    return await self._basic_send_command(
                        command=command, wait_done=wait_done
                    )
            else:
                return await self._basic_send_command(
                    command=command, wait_done=wait_done
                )
        except ConnectionResetError:
            raise
        except Exception as e:
            self.log.exception(f"Failed to send command {command}: {e!r}")
            raise

    async def _basic_send_command(self, command, wait_done=True):
        """Implementation of send_command. Ignores the command lock.

        Parameters
        ----------
        command : `Command`
            Command to send.
        wait_done : `bool`, optional
            Wait for the command to finish?
            If False then only wait for the command to be acknowledged.

        Returns
        -------
        command_futures : `command_futures.CommandFutures`
            Futures that monitor the command.
        """
        if not self.connected:
            if self.should_be_connected:
                await self.fault(
                    enums.CscErrorCode.CONNECTION_LOST,
                    report="Connection lost to low-level controller (noticed in _basic_send_command)",
                )
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
        elif not futures.done.done() and wait_done:
            try:
                await asyncio.wait_for(futures.done, timeout=timeout + TIMEOUT_BUFFER)
            except asyncio.TimeoutError:
                self.command_dict.pop(command.sequence_id, None)
                raise asyncio.TimeoutError(
                    f"Timed out after {timeout + TIMEOUT_BUFFER} seconds "
                    f"waiting for the Done reply to {command}"
                )
        return futures

    async def send_commands(self, *commands, do_lock):
        """Run a set of operation manager commands.

        Wait for each command to finish before issuing the next.
        Wait for all commands to finish before returning.

        Parameters
        ----------
        commands : `List` [``Command``]
            Commands to send. The sequence_id attribute is set.
        do_lock : `bool`
            Lock the port while this method runs?
            The low-level controller has ill-defined limits on which commands
            commands can run simultaneously, so specify True when practical.
            Specify False for stop commands and the camera cable wrap
            tracking command (so rotator following is not blocked).
        """
        future = None
        try:
            if do_lock:
                async with self.command_lock:
                    for command in commands:
                        future = await self.send_command(
                            command, do_lock=False, wait_done=True
                        )
                        await future.done
            else:
                for command in commands:
                    future = await self.send_command(
                        command, do_lock=False, wait_done=True
                    )
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

    async def start_camera_cable_wrap_following(self):
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
            await self.send_command(
                commands.CameraCableWrapEnableTracking(on=True), do_lock=False
            )
            self.camera_cable_wrap_follow_loop_task = asyncio.create_task(
                self._camera_cable_wrap_follow_loop()
            )
            await self.evt_cameraCableWrapFollowing.set_write(
                enabled=True, force_output=True
            )
        except asyncio.CancelledError:
            self.log.info("Camera cable wrap following canceled before it starts")
            await self.evt_cameraCableWrapFollowing.set_write(enabled=False)
            # Try to stop the camera cable wrap on a best-effort basis.
            await self.send_command(commands.CameraCableWrapStop(), do_lock=False)
            raise
        except Exception as e:
            err_msg = "Camera cable wrap following could not be started"
            if isinstance(e, salobj.ExpectedError):
                self.log.error(f"{err_msg}: {e!r}")
            else:
                self.log.exception(err_msg)
            await self.evt_cameraCableWrapFollowing.set_write(enabled=False)
            # Try to stop the camera cable wrap on a best-effort basis.
            # This should succeed if things are working, but they
            # may not be in this exception branch.
            await self.send_command(commands.CameraCableWrapStop(), do_lock=False)
            raise

    async def _camera_cable_wrap_follow_loop(self):
        """Implement the camera cable wrap following the camera rotator.

        This should be called by start_camera_cable_wrap_following.
        Camera cable wrap tracking must be enabled before this is called.
        """
        self.log.info("Camera cable wrap following begins")
        self.rotator_position_error_excessive = False
        if not self.evt_cameraCableWrapControllerSettings.has_data:
            raise RuntimeError(
                "cameraCableWrapControllerSettings event has not been published; "
                "camera cable wrap command limits unknown"
            )
        paused = False
        try:
            while self.camera_cable_wrap_following_enabled:
                try:
                    position_velocity_tai = await self.get_camera_cable_wrap_demand()
                except asyncio.TimeoutError:
                    if not paused:
                        paused = True
                        self.log.warning(
                            "Rotator data not available; stopping the camera "
                            "cable wrap until rotator data is available"
                        )
                        await self.send_command(
                            commands.CameraCableWrapStop(), do_lock=False
                        )
                    continue
                if paused:
                    paused = False
                    self.log.info(
                        "Rotator data received; resume making "
                        "the camera cable wrap follow the rotator"
                    )
                    await self.send_command(
                        commands.CameraCableWrapEnableTracking(on=True), do_lock=False
                    )

                position, velocity, tai = position_velocity_tai
                command = commands.CameraCableWrapTrackTarget(
                    position=position,
                    velocity=velocity,
                    tai=tai,
                )
                await self.send_command(command, do_lock=False)
                await self.evt_cameraCableWrapTarget.set_write(
                    position=position, velocity=velocity, taiTime=tai
                )
                if self.camera_cable_wrap_following_enabled:
                    await asyncio.sleep(self.config.camera_cable_wrap_interval)

        except asyncio.CancelledError:
            self.log.info("Camera cable wrap following ends")
        except salobj.ExpectedError as e:
            self.log.error(f"Camera cable wrap following failed: {e!r}")
        except Exception:
            self.log.exception("Camera cable wrap following failed")
        finally:
            await self.evt_cameraCableWrapFollowing.set_write(enabled=False)

    @property
    def camera_cable_wrap_following_enabled(self):
        """Return True if camera cable wrap following rotator is enabled."""
        return self.evt_cameraCableWrapFollowing.data.enabled

    async def clear_target(self):
        """Clear the target event."""
        await self.evt_target.set_write(
            elevation=math.nan,
            azimuth=math.nan,
            elevationVelocity=math.nan,
            azimuthVelocity=math.nan,
            taiTime=math.nan,
            trackId=0,
            tracksys="",
            radesys="",
        )

    async def configure(self, config):
        self.config = config

    async def do_clearError(self, data):
        """Handle the clearError command."""
        self.assert_enabled()
        raise salobj.ExpectedError("Not yet implemented")

    async def do_closeMirrorCovers(self, data):
        """Handle the closeMirrorCovers command."""
        self.assert_enabled()
        await self.cmd_openMirrorCovers.ack_in_progress(
            data=data, timeout=MIRROR_COVER_TIMEOUT
        )
        await self.send_commands(
            commands.MirrorCoverLocksPower(on=True),
            commands.MirrorCoversPower(on=True),
            commands.MirrorCoverSystemDeploy(),
            commands.MirrorCoverLocksPower(on=False),
            commands.MirrorCoversPower(on=False),
            do_lock=True,
        )

    async def do_disableCameraCableWrapFollowing(self, data):
        """Handle the disableCameraCableWrapFollowing command."""
        self.assert_enabled()
        await self.stop_camera_cable_wrap_following()

    async def do_enableCameraCableWrapFollowing(self, data):
        """Handle the enableCameraCableWrapFollowing command."""
        self.assert_enabled()
        if self.camera_cable_wrap_follow_loop_task.done():
            self.camera_cable_wrap_follow_start_task = asyncio.create_task(
                self.start_camera_cable_wrap_following()
            )
            await self.camera_cable_wrap_follow_start_task

    async def do_homeBothAxes(self, data):
        self.assert_enabled()
        await self.send_command(
            commands.BothAxesHome(),
            do_lock=True,
        )

    async def do_openMirrorCovers(self, data):
        """Handle the openMirrorCovers command."""
        self.assert_enabled()
        await self.cmd_openMirrorCovers.ack_in_progress(
            data=data, timeout=MIRROR_COVER_TIMEOUT
        )
        await self.send_commands(
            commands.MirrorCoverLocksPower(on=True),
            commands.MirrorCoversPower(on=True),
            commands.MirrorCoverSystemRetract(),
            commands.MirrorCoversPower(on=False),
            commands.MirrorCoverLocksPower(on=False),
            do_lock=True,
        )

    async def do_moveToTarget(self, data):
        """Handle the moveToTarget command."""
        self.assert_enabled()
        cmd_futures = await self.send_command(
            commands.BothAxesMove(azimuth=data.azimuth, elevation=data.elevation),
            do_lock=True,
        )
        timeout = cmd_futures.timeout + TIMEOUT_BUFFER
        await self.cmd_moveToTarget.ack_in_progress(data=data, timeout=timeout)
        await cmd_futures.done

    async def do_trackTarget(self, data):
        """Handle the trackTarget command."""
        self.assert_enabled()
        # Use do_lock=False to prevent a slow command
        # from interrrupting tracking
        await self.send_command(
            commands.BothAxesTrackTarget(
                azimuth=data.azimuth,
                azimuth_velocity=data.azimuthVelocity,
                elevation=data.elevation,
                elevation_velocity=data.elevationVelocity,
                tai=data.taiTime,
            ),
            do_lock=False,
        )
        await self.evt_target.set_write(
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
        """Handle the startTracking command."""
        self.assert_enabled()
        await self.send_command(commands.BothAxesEnableTracking(), do_lock=True)

    async def do_stop(self, data):
        """Handle the stop command."""
        self.assert_enabled()
        await self.send_commands(
            commands.BothAxesStop(),
            commands.CameraCableWrapStop(),
            commands.MirrorCoverLocksStop(),
            commands.MirrorCoversStop(),
            do_lock=False,
        )

    async def do_stopTracking(self, data):
        """Handle the stopTracking command."""
        self.assert_enabled()
        await self.send_command(commands.BothAxesStop(), do_lock=False)

    async def handle_available_settings(self, reply):
        """Handle a `ReplyId.AVAILABLE_SETTINGS` reply."""
        await self.evt_availableSettings.set_write(
            names=", ".join(item["name"] for item in reply.sets),
            descriptions=json.dumps([item["description"] for item in reply.sets]),
            createdDates=", ".join(item["createdDate"] for item in reply.sets),
            modifiedDates=", ".join(item["modifiedDate"] for item in reply.sets),
        )

    async def handle_axis_motion_state(self, reply):
        """Handle a `ReplyId.AXIS_MOTION_STATE` reply."""
        axis = reply.axis
        state = reply.state
        was_tracking = self.is_tracking()
        if axis == System.ELEVATION:
            await self.evt_elevationMotionState.set_write(state=state)
            if state == AxisMotionState.MOVING_POINT_TO_POINT:
                await self.evt_target.set_write(
                    elevation=reply.position,
                    elevationVelocity=0,
                    taiTime=utils.current_tai(),
                    trackId=0,
                    tracksys="LOCAL",
                    radesys="",
                )
        elif axis == System.AZIMUTH:
            await self.evt_azimuthMotionState.set_write(state=state)
            if state == AxisMotionState.MOVING_POINT_TO_POINT:
                await self.evt_target.set_write(
                    azimuth=reply.position,
                    azimuthVelocity=0,
                    taiTime=utils.current_tai(),
                    trackId=0,
                    tracksys="LOCAL",
                    radesys="",
                )
        elif axis == System.CAMERA_CABLE_WRAP:
            await self.evt_cameraCableWrapMotionState.set_write(state=state)
        else:
            self.log.warning(f"Unrecognized axis={axis} in handle_axis_motion_state")
        if was_tracking and not self.is_tracking():
            await self.clear_target()

    async def handle_azimuth_topple_block(self, reply):
        """Handle a `ReplyId.AZIMUTH_TOPPLE_BLOCK` reply."""
        await self.evt_azimuthToppleBlock.set_write(
            forward=reply.forward,
            reverse=reply.reverse,
        )

    async def handle_chiller_state(self, reply):
        """Andle a `ReplyId.CHILLER_STATE` reply."""
        topic_info = self.system_state_dict[reply.system]
        if topic_info.num_thermal == 1:
            await topic_info.topic.set_write(
                trackAmbient=reply.trackAmbient[0],
                setTemperature=reply.temperature[0],
            )
        else:
            await topic_info.topic.set_write(
                trackAmbient=reply.trackAmbient,
                setTemperature=reply.temperature,
            )

    async def handle_command_reply(self, reply):
        """Handle a ReplyId.CMD_x reply."""
        reply_id = reply.id
        sequence_id = reply.sequenceId
        if reply_id == enums.ReplyId.CMD_ACKNOWLEDGED:
            futures = self.command_dict.get(sequence_id, None)
        else:
            futures = self.command_dict.pop(sequence_id, None)
        if futures is None:
            self.log.warning(
                f"Got reply with code {enums.ReplyId(reply_id)!r} "
                f"for non-existent command {sequence_id}"
            )
            return
        if reply_id == enums.ReplyId.CMD_ACKNOWLEDGED:
            # Command acknowledged. Set timeout but leave
            # futures in command_dict.
            futures.setack(reply.timeout)
        elif reply_id in (enums.ReplyId.CMD_REJECTED, enums.ReplyId.CMD_FAILED):
            # Command failed (before or after being acknowledged).
            # Pop the command_dict entry and report failure.
            futures.setnoack(reply.explanation)
        elif reply_id == enums.ReplyId.CMD_SUCCEEDED:
            futures.setdone()
        elif reply_id == enums.ReplyId.CMD_SUPERSEDED:
            futures.setnoack("Superseded")
        else:
            raise RuntimeError(f"Bug: unsupported reply code {reply_id}")

    async def handle_commander(self, reply):
        """Handle a `ReplyId.COMMANDER` reply."""
        await self.evt_commander.set_write(commander=reply.actualCommander)

    async def handle_deployable_motion_state(self, reply, topic):
        """Handle a generic position event.

        Including:

        * `ReplyId.DEPLOYABLE_PLATFORMS_MOTION_STATE`
        * `ReplyId.MIRROR_COVER_LOCKS_MOTION_STATE`
        * `ReplyId.MIRROR_COVERS_MOTION_STATE`
        """
        await topic.set_write(state=reply.state, elementsState=reply.elementsState)

    async def handle_detailed_settings_applied(self, reply):
        """Handle a `ReplyId.DETAILED_SETTINGS_APPLIED` reply."""
        for axis_name, event in (
            ("Azimuth", self.evt_azimuthControllerSettings),
            ("Elevation", self.evt_elevationControllerSettings),
        ):
            axis_settings = reply.MainAxis[axis_name]
            min_operational_l1_limit_enabled = axis_settings[
                "LimitsNegativeAdjustableSoftwareLimitEnable"
            ]
            if min_operational_l1_limit_enabled:
                min_l1_limit = axis_settings[
                    "LimitsNegativeAdjustableSoftwareLimitValue"
                ]
            else:
                min_l1_limit = axis_settings["LimitsNegativeSoftwareLimitValue"]
            max_operational_l1_limit_enabled = axis_settings[
                "LimitsPositiveAdjustableSoftwareLimitEnable"
            ]
            if max_operational_l1_limit_enabled:
                max_l1_limit = axis_settings[
                    "LimitsPositiveAdjustableSoftwareLimitValue"
                ]
            else:
                max_l1_limit = axis_settings["LimitsPositiveSoftwareLimitValue"]

            if axis_name == "Elevation":
                op_l2_limit_enabled_kwargs = dict(
                    minOperationalL2LimitEnabled=axis_settings[
                        "LimitsNegativeOperationalLimitSwitchEnable"
                    ],
                    maxOperationalL2LimitEnabled=axis_settings[
                        "LimitsPositiveOperationalLimitSwitchEnable"
                    ],
                )
            else:
                op_l2_limit_enabled_kwargs = {}
            await event.set_write(
                minCmdPositionEnabled=axis_settings["LimitsMinPositionEnable"],
                maxCmdPositionEnabled=axis_settings["LimitsMaxPositionEnable"],
                minL1LimitEnabled=axis_settings["LimitsNegativeSoftwareLimitEnable"],
                maxL1LimitEnabled=axis_settings["LimitsPositiveSoftwareLimitEnable"],
                minOperationalL1LimitEnabled=min_operational_l1_limit_enabled,
                maxOperationalL1LimitEnabled=max_operational_l1_limit_enabled,
                minL2LimitEnabled=axis_settings["LimitsNegativeLimitSwitchEnable"],
                maxL2LimitEnabled=axis_settings["LimitsPositiveLimitSwitchEnable"],
                minCmdPosition=axis_settings["LimitsMinPositionValue"],
                maxCmdPosition=axis_settings["LimitsMaxPositionValue"],
                minL1Limit=min_l1_limit,
                maxL1Limit=max_l1_limit,
                maxCmdVelocity=axis_settings["TcsMaxSpeed"],
                maxMoveVelocity=axis_settings["TcsDefaultVelocity"],
                maxMoveAcceleration=axis_settings["TcsDefaultAcceleration"],
                maxMoveJerk=axis_settings["TcsDefaultJerk"],
                maxTrackingVelocity=axis_settings["SoftmotionTrackingMaxSpeed"],
                maxTrackingAcceleration=axis_settings[
                    "SoftmotionTrackingMaxAcceleration"
                ],
                maxTrackingJerk=axis_settings["SoftmotionTrackingMaxJerk"],
                **op_l2_limit_enabled_kwargs,
            )
        ccw_settings = reply.CW["CCW"]
        await self.evt_cameraCableWrapControllerSettings.set_write(
            l1LimitsEnabled=ccw_settings["SoftwareLimitEnable"],
            l2LimitsEnabled=ccw_settings["LimitSwitchEnable"],
            minCmdPosition=ccw_settings["MinPosition"],
            maxCmdPosition=ccw_settings["MaxPosition"],
            minL1Limit=ccw_settings["MinSoftwareLimit"],
            maxL1Limit=ccw_settings["MaxSoftwareLimit"],
            maxCmdVelocity=ccw_settings["MaxSpeed"],
            maxMoveVelocity=ccw_settings["DefaultSpeed"],
            maxMoveAcceleration=ccw_settings["DefaultAcceleration"],
            maxMoveJerk=ccw_settings["DefaultJerk"],
            maxTrackingVelocity=ccw_settings["TrackingSpeed"],
            maxTrackingAcceleration=ccw_settings["TrackingAcceleration"],
            maxTrackingJerk=ccw_settings["TrackingJerk"],
        )

    async def handle_elevation_locking_pin_motion_state(self, reply):
        """Handle a `ReplyId.ELEVATION_LOCKING_PIN_MOTION_STATE` reply."""
        await self.evt_elevationLockingPinMotionState.set_write(
            state=reply.state, elementsState=reply.elementsState
        )

    async def handle_error(self, reply):
        """Handle a `ReplyId.ERROR` reply."""
        await self.evt_error.set_write(
            code=reply.code,
            latched=reply.latched,
            active=reply.active,
            text=reply.description,
            force_output=True,
        )

    async def handle_in_position(self, reply):
        """Handle a `ReplyId.IN_POSITION` reply."""
        topic = {
            System.ELEVATION: self.evt_elevationInPosition,
            System.AZIMUTH: self.evt_azimuthInPosition,
            System.CAMERA_CABLE_WRAP: self.evt_cameraCableWrapInPosition,
        }.get(reply.axis, None)
        if topic is None:
            self.log.warning(f"Unrecognized axis={reply.system} in handle_in_position")
        else:
            await topic.set_write(inPosition=reply.inPosition)

    async def handle_limits(self, reply):
        """Handle a `ReplyId.LIMITS` reply."""
        topic, nelts = {
            System.ELEVATION: (self.evt_elevationLimits, 1),
            System.AZIMUTH: (self.evt_azimuthLimits, 1),
            System.CAMERA_CABLE_WRAP: (self.evt_cameraCableWrapLimits, 1),
        }.get(reply.system, (None, 1))
        if topic is None:
            try:
                reply.system = System(reply.system)
            except ValueError:
                pass
            self.log.info(f"Unsupported system={reply.system!r} in handle_limits")
        else:
            if nelts > 1:
                await topic.set_write(limits=reply.limits)
            else:
                await topic.set_write(limits=reply.limits[0])

    async def handle_motion_controller_state(self, reply):
        """Handle a `ReplyId.MOTION_CONTROLLER_STATE` reply."""
        topic_info = self.system_state_dict[reply.system]
        if topic_info.num_motion_controller_state == 1:
            await topic_info.topic.set_write(
                motionControllerState=reply.motionControllerState[0],
            )
        else:
            await topic_info.topic.set_write(
                motionControllerState=reply.motionControllerState,
            )

    async def handle_power_state(self, reply):
        """Handle a `ReplyId.POWER_STATE` reply."""
        topic_info = self.system_state_dict[reply.system]
        if topic_info.num_elements_power_state > 0:
            await topic_info.topic.set_write(
                powerState=reply.powerState, elementsPowerState=reply.elementPowerState
            )
        else:
            await topic_info.topic.set_write(powerState=reply.powerState)

    async def handle_safety_interlocks(self, reply):
        """Handle a `ReplyId.SAFETY_INTERLOCKS` reply."""
        data_dict = vars(reply)
        del data_dict["id"]
        del data_dict["timestamp"]
        await self.evt_safetyInterlocks.set_write(**data_dict)

    async def handle_warning(self, reply):
        """Handle a `ReplyId.WARNING` reply."""
        await self.evt_warning.set_write(
            code=reply.code,
            active=reply.active,
            text=reply.description,
            force_output=True,
        )

    def is_tracking(self):
        """Return True if tracking according to the elevation and azimuth
        motion state events.
        """

        return all(
            [
                evt.data is not None and evt.data.state == AxisMotionState.TRACKING
                for evt in (self.evt_elevationMotionState, self.evt_azimuthMotionState)
            ]
        )

    async def monitor_telemetry_client(self):
        """Go to FAULT state if the telemetry client exits prematurely."""
        await self.telemetry_client_process.wait()
        await self.fault(
            code=enums.CscErrorCode.TELEMETRY_CLIENT_ERROR,
            report="Telemetry process exited prematurely",
        )

    async def read_loop(self):
        """Read and process replies from the low-level controller."""
        self.log.debug("Read loop begins")
        known_reply_ids = {int(item) for item in enums.ReplyId}
        while self.should_be_connected and self.connected:
            try:
                read_bytes = await self.reader.readuntil(constants.LINE_TERMINATOR)
                try:
                    reply_dict = json.loads(read_bytes)
                    self.log.debug("Read %s", reply_dict)
                    reply_id = reply_dict["id"]
                    if reply_id not in known_reply_ids:
                        if reply_id not in self.unknown_reply_ids:
                            self.unknown_reply_ids.add(reply_id)
                            self.log.warning(
                                f"Ignoring reply with unknown id={reply_id}: {read_bytes}:"
                            )
                        continue
                    reply = types.SimpleNamespace(
                        id=enums.ReplyId(reply_id),
                        timestamp=reply_dict["timestamp"],
                        **reply_dict["parameters"],
                    )
                except Exception as e:
                    self.log.warning(f"Ignoring unparsable reply: {read_bytes}: {e!r}")
                    continue

                handler = self.reply_dispatch.get(reply.id, None)
                if handler:
                    try:
                        await handler(reply)
                    except Exception as e:
                        self.log.error(f"Failed to handle reply: {reply}: {e!r}")
                else:
                    self.log.warning(f"Ignoring unrecognized reply: {reply}")
            except asyncio.CancelledError:
                return
            except Exception as e:
                if self.should_be_connected:
                    if self.connected:
                        err_msg = "Read loop failed; possibly a bug"
                        self.log.exception(err_msg)
                        await self.fault(
                            code=enums.CscErrorCode.INTERNAL_ERROR,
                            report=f"{err_msg}: {e!r}",
                        )
                    else:
                        await self.fault(
                            code=enums.CscErrorCode.CONNECTION_LOST,
                            report="Connection lost to low-level controller (noticed in read_loop)",
                        )
        self.log.debug("Read loop ends")

    async def send_heartbeat_loop(self):
        """Send a regular heartbeat "command" to the low-level controller.

        The command is not acknowledged in any way.
        """
        heartbeat_bytes = commands.Heartbeat().encode()
        try:
            while self.connected:
                self.writer.write(heartbeat_bytes)
                await self.writer.drain()
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass
        except Exception:
            self.log.exception("send heartbeat loop failed")

    def signal_handler(self):
        """Handle signals such as SIGTERM."""
        self.terminate_background_processes()
        asyncio.create_task(self.close())

    async def start(self):
        await super().start()
        self.has_modern_connected_event = hasattr(self.evt_connected.data, "connected")
        if self.has_modern_connected_event:
            await self.evt_connected.set_write(connected=self.connected)
        if self.simulation_mode == 1 and self.run_mock_controller:
            # Run the mock controller using random ports;
            # read the ports to set command_port and telemetry_port.
            args = [
                "run_mock_tma",
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
                await self.fault(
                    code=enums.CscErrorCode.MOCK_CONTROLLER_ERROR,
                    report=f"{err_msg}: {e!r}",
                )
                return

            self.log.info("Waiting for mock controller to start")
            t0 = utils.current_tai()
            self.command_port, self.telemetry_port = await asyncio.wait_for(
                self._wait_for_mock_controller(),
                timeout=MOCK_CTRL_START_TIMEOUT,
            )
            dt = utils.current_tai() - t0
            self.log.info(
                f"Mock controller running with command_port={self.command_port} "
                f"telemetry_port={self.telemetry_port}; {dt:0.1f} seconds to start"
            )

        await asyncio.gather(self.rotator.start_task, self.mtmount_remote.start_task)
        await self.evt_cameraCableWrapFollowing.set_write(enabled=False)
        await self.clear_target()

    async def stop_camera_cable_wrap_following(self):
        """Stop the camera cable wrap from following the rotator."""
        await self.evt_cameraCableWrapFollowing.set_write(enabled=False)
        self.camera_cable_wrap_follow_start_task.cancel()
        if self.camera_cable_wrap_follow_loop_task.done():
            return
        try:
            await asyncio.wait_for(
                self.camera_cable_wrap_follow_loop_task,
                timeout=self.config.camera_cable_wrap_interval + 0.1,
            )
        except asyncio.TimeoutError:
            self.camera_cable_wrap_follow_loop_task.cancel()
        finally:
            await self.send_command(commands.CameraCableWrapStop(), do_lock=False)


def run_mtmount() -> None:
    """Run the MTMount CSC."""
    asyncio.run(MTMountCsc.amain(index=None))
