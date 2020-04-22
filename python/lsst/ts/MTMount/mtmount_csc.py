# This file is part of ts_MTMount.
#
# Developed for the LSST Data Management System.
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
import pathlib

from lsst.ts import salobj
from . import command_futures
from . import commands
from . import enums
from . import replies
from . import communicator
from . import mock
from . import utils


# Extra time to wait for commands to be done (sec)
TIMEOUT_BUFFER = 5

# Maximum time (second) between rotator application telemetry topics,
# beyond which rotator velocity will not be estimated.
MAX_ROTATOR_APPLICATION_GAP = 1


AxisEnableCommands = (
    commands.AzimuthAxisDriveEnable,
    commands.ElevationAxisDriveEnable,
    commands.CameraCableWrapDriveEnable,
)


class MTMountCsc(salobj.ConfigurableCsc):
    """MTMount CSC

    Parameters
    ----------
    initial_state : `salobj.State` or `int` (optional)
        The initial state of the CSC. This is provided for unit testing,
        as real CSCs should start up in `lsst.ts.salobj.StateSTANDBY`,
        the default.
    simulation_mode : `int` (optional)
        Simulation mode.
    mock_reply_port : `int` (optional)
        Port for mock controller TCP/IP interface. If `None` then use the
        port specified by the configuration. Only used in simulation mode.

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

    def __init__(
        self,
        config_dir=None,
        initial_state=salobj.State.STANDBY,
        simulation_mode=0,
        mock_reply_port=None,
    ):
        schema_path = (
            pathlib.Path(__file__).resolve().parents[4] / "schema" / "MTMount.yaml"
        )
        self.mock_reply_port = mock_reply_port
        self.communicator = None
        self.mock_controller = None  # mock controller, or None of not constructed
        # Dict of command sequence-id: (ack_task, done_task).
        # ack_task is set to the timeout (sec) when command is acknowledged.
        # done_task is set to to None when the command is done.
        # Both are set to exceptions if the command fails.
        self.command_dict = dict()

        self.command_lock = asyncio.Lock()

        # Task that waits while connecting to the TCP/IP controller.
        self.connect_task = salobj.make_done_future()

        # Task to track enabling and disabling
        self.enable_task = salobj.make_done_future()
        self.disable_task = salobj.make_done_future()
        self.enable_state = enums.EnabledState.DISABLED

        # Task for self.camera_cable_wrap_loop
        self.camera_cable_wrap_task = salobj.make_done_future()

        # Task for self.read_loop
        self.read_loop_task = salobj.make_done_future()

        # Most recent Rotator Application telemetry sample,
        # or None if camera cable wrap not tracking.
        self.prev_rot_application = None

        # Should the CSC be connected?
        # Used to decide whether disconnecting sends the CSC to a Fault state.
        # Set True when fully connected and False just before
        # intentionally disconnecting.
        self.should_be_connected = False

        super().__init__(
            name="NewMTMount",
            index=0,
            schema_path=schema_path,
            config_dir=config_dir,
            initial_state=initial_state,
            simulation_mode=simulation_mode,
        )

        self.rotator = salobj.Remote(
            domain=self.domain, name="Rotator", include=["Application"]
        )

    @property
    def connected(self):
        if self.communicator is None:
            return False
        return self.communicator.connected

    @staticmethod
    def get_config_pkg():
        return "ts_config_mttcs"

    async def close_tasks(self):
        """Shut down pending tasks. Called by `close`."""
        await super().close_tasks()
        await self.close_communication()

    async def close_communication(self):
        """Close and delete the communicator and mock controller, if present.
        """
        self.should_be_connected = False
        if self.communicator is not None:
            await self.communicator.close()
            self.communicator = None
        if self.mock_controller is not None:
            await self.mock_controller.close()
            self.mock_controller = None

    def get_host(self):
        if self.simulation_mode:
            return salobj.LOCAL_HOST
        return self.config.host

    def get_reply_port(self):
        """Get the desired reply port.

        This will be self.config.reply_port unless in simulation mode
        and mock_reply_port specified.
        """
        if self.config is None:
            raise RuntimeError("Not yet configured")
        if self.simulation_mode != 0 and self.mock_reply_port is not None:
            return self.mock_reply_port
        else:
            return self.config.reply_port

    async def connect(self):
        """Connect to the Operation Manager.

        Start the mock controller, if simulating.
        """
        self.log.debug("connect")
        if self.config is None:
            raise RuntimeError("Not yet configured")
        if self.connected:
            raise RuntimeError("Already connected")
        client_host = self.config.host
        server_host = None
        reply_port = self.config.reply_port
        if self.simulation_mode:
            client_host = salobj.LOCAL_HOST
            server_host = salobj.LOCAL_HOST
            if self.mock_reply_port is not None:
                reply_port = self.mock_reply_port
        connect_tasks = []
        if self.simulation_mode:
            connect_tasks.append(self.start_mock_ctrl())
        if self.communicator is None:
            self.communicator = communicator.Communicator(
                name="communicator",
                client_host=client_host,
                client_port=reply_port + 1,
                server_host=server_host,
                server_port=reply_port,
                log=self.log,
                read_replies=True,
                connect_client=False,
                connect_callback=self.connect_callback,
            )
        try:
            if self.simulation_mode:
                self.log.info("Connecting the CSC and mock controller to each other.")
            else:
                self.log.info("Connecting a client to the Operation Manager.")
            connect_tasks.append(self.communicator.connect())
            await asyncio.wait_for(
                asyncio.gather(*connect_tasks), timeout=self.config.connection_timeout
            )
            self.read_loop_task = asyncio.create_task(self.read_loop())
            self.should_be_connected = True
            self.log.debug("connected")
        except Exception as e:
            err_msg = f"Could not connection to client_host={client_host}, "
            f"reply_port={self.config.reply_port}"
            self.log.exception(err_msg)
            self.fault(
                code=enums.CscErrorCode.COULD_NOT_CONNECT, report=f"{err_msg}: {e}"
            )
            return

    def connect_callback(self, communicator):
        self.evt_connected.set_put(
            command=communicator.client_connected, replies=communicator.server_connected
        )
        if self.should_be_connected and not communicator.connected:
            self.should_be_connected = False
            self.fault(
                code=enums.CscErrorCode.CONNECTION_LOST,
                report="connection lost to low-level controller (noticed in connect_callback)",
            )

    async def enable_devices(self):
        self.disable_task.cancel()
        self.enabled_state = enums.EnabledState.ENABLING
        try:
            enable_commands = [
                commands.TopEndChillerPower(on=True),
                # I have asked Tekniker why there is a temperature argument
                # for the top end chiller track ambient command.
                commands.TopEndChillerTrackAmbient(on=True, temperature=0),
                commands.MainPowerSupplyPower(on=True),
                commands.OilSupplySystemPower(on=True),
            ] + [command(drive=-1, on=True) for command in AxisEnableCommands]
            await self.send_commands(*enable_commands)
            self.enabled_state = enums.EnabledState.ENABLED
        except Exception as e:
            self.enabled_state = enums.EnabledState.ENABLE_FAILED
            err_msg = "Failed to enable one or more devices"
            self.log.exception(err_msg)
            self.fault(
                code=enums.CscErrorCode.ACTUATOR_ENABLE_ERROR, report=f"{err_msg}: {e}",
            )

    async def disable_devices(self):
        self.enable_task.cancel()
        self.enabled_state = enums.EnabledState.DISABLING
        try:
            disable_commands = [commands.BothAxesStop()] + [
                command(drive=-1, on=False) for command in AxisEnableCommands
            ]
            await self.send_commands(*disable_commands)
            self.enabled_state = enums.EnabledState.DISABLED
        except Exception as e:
            self.enabled_state = enums.EnabledState.DISABLE_FAILED
            err_msg = "Failed to disable one or more devices"
            self.log.exception(err_msg)
            self.fault(
                code=enums.CscErrorCode.ACTUATOR_DISABLE_ERROR,
                report=f"{err_msg}: {e}",
            )

    async def handle_summary_state(self):
        if self.disabled_or_enabled:
            if not self.connected and self.connect_task.done():
                await self.connect()
            if self.enable_state is not enums.EnabledState.ENABLED:
                self.enable_task.cancel()
                self.enable_task = asyncio.create_task(self.enable_devices())
                await self.enable_task
        else:
            if self.enable_state is not enums.EnabledState.DISABLED:
                self.disable_task.cancel()
                self.disable_task = asyncio.create_task(self.disable_devices())
                await self.disable_task
            await self.close_communication()

    async def implement_simulation_mode(self, simulation_mode):
        if simulation_mode not in (0, 1):
            raise salobj.ExpectedError(
                f"Simulation_mode={simulation_mode} must be 0 or 1"
            )

    async def send_command(self, command, dolock=True):
        """Send a command to the operation manager
        and add it to command_dict.

        Parameters
        ----------
        command : `Command`
            Command to send.
        dolock : `bool` (optional)
            Lock the port while using it?
            Specify False for emergency commands
            or if being called by send_commands.


        Returns
        -------
        command_futures : `command_futures.CommandFutures`
            Futures that monitor the command.
        """
        try:
            if dolock:
                async with self.command_lock:
                    return await self._basic_send_command(command=command)
            else:
                return await self._basic_send_command(command=command)
        except Exception:
            self.log.exception(f"send_command({command}) failed")
            raise

    async def _basic_send_command(self, command):
        """Implementation of send_command. Ignores the command lock.
        """
        if not self.connected:
            raise salobj.ExpectedError("Not connected to the low-level controller.")
        if command.sequence_id in self.command_dict:
            raise RuntimeError(
                f"Bug! Duplicate sequence_id {command.sequence_id} in command_dict"
            )
        futures = command_futures.CommandFutures()
        self.command_dict[command.sequence_id] = futures
        self.log.debug(f"write command: %s", command)
        await self.communicator.write(command)
        await asyncio.wait_for(futures.ack, self.config.ack_timeout)
        if command.command_code in commands.AckOnlyCommandCodes:
            # This command only receives an Ack; mark it done.
            futures.done.set_result(None)
        else:
            await asyncio.wait_for(
                futures.done, timeout=futures.timeout + TIMEOUT_BUFFER
            )
        return futures

    async def send_commands(self, *commands, dolock=True):
        """Run a set of operation manager commands.

        Parameters
        ----------
        commands : `List` [``Command``]
            Commands to send. The sequence_id attribute is set.
        dolock : `bool` (optional)
            Lock the port while using it?
            Specify False for emergency commands.

        Returns
        -------
        futures_list : `List` [`CommandFutures`]
            Command future for each command.
        """
        futures_list = []
        if dolock:
            async with self.command_lock:
                for command in commands:
                    futures_list.append(await self.send_command(command, dolock=False))
        else:
            for command in commands:
                futures_list.append(await self.send_command(command, dolock=False))
        return futures_list

    async def camera_cable_wrap_loop(self):
        self.log.info("camera_cable_wrap_loop begins")
        try:
            self.prev_rot_application = None
            while True:
                rot_application = await self.rotator.tel_Application.next(flush=True)
                estimated_velocity = 0
                if self.prev_rot_application is not None:
                    dt = (
                        rot_application.private_sndStamp
                        - self.prev_rot_application.private_sndStamp
                    )
                    if dt < MAX_ROTATOR_APPLICATION_GAP:
                        estimated_velocity = (
                            rot_application.Position
                            - self.prev_rot_application.Position
                        ) / dt

                command = commands.CameraCableWrapTrack(
                    position=rot_application.Position,
                    velocity=estimated_velocity,
                    tai_time=utils.get_tai_time(),
                )
                await self.send_command(command)
                self.prev_rot_application = rot_application
        except asyncio.CancelledError:
            self.log.info("camera_cable_wrap_loop ends")
        except Exception:
            self.log.exception("camera_cable_wrap failed")

    async def configure(self, config):
        self.config = config

    async def read_loop(self):
        """Read and process replies from the Operation Manager.
        """
        self.log.debug("read loop begins")
        while self.should_be_connected and self.communicator.connected:
            try:
                reply = await self.communicator.read()
                if isinstance(reply, replies.AckReply):
                    # Command acknowledged. Set timeout but leave
                    # futures in command_dict.
                    futures = self.command_dict.get(reply.sequence_id, None)
                    if futures is None:
                        self.log.warning(
                            f"Got Ack for non-existent command {reply.sequence_id}"
                        )
                        continue
                    futures.setack(reply.timeout_ms / 100.0)
                elif isinstance(reply, replies.NoAckReply):
                    # Command failed. Pop the command_dict entry
                    # and report failure.
                    futures = self.command_dict.pop(reply.sequence_id, None)
                    if futures is None:
                        self.log.warning(
                            f"Got NoAck for non-existent command {reply.sequence_id}"
                        )
                        continue
                    futures.setnoack(reply.explanation)
                elif isinstance(reply, replies.DoneReply):
                    futures = self.command_dict.pop(reply.sequence_id, None)
                    if futures is None:
                        self.log.warning(
                            f"Got Done for non-existent command {reply.sequence_id}"
                        )
                        continue
                    futures.setdone()
                elif isinstance(reply, replies.WarningReply):
                    self.evt_warning.set_put(
                        code=reply.code,
                        active=reply.active,
                        text="\n".join(reply.extra_data),
                        force_output=True,
                    )
                elif isinstance(reply, replies.ErrorReply):
                    self.evt_error.set_put(
                        code=reply.code,
                        latched=reply.on,
                        active=reply.active,
                        text="\n".join(reply.extra_data),
                        force_output=True,
                    )
                elif isinstance(reply, replies.OnStateInfoReply):
                    self.log.info(f"Ignoring OnStateInfo reply: {reply}")
                elif isinstance(reply, replies.InPositionReply):
                    # TODO: Handle InPosition
                    self.log.info(f"Read InPosition reply: {reply}")
                else:
                    self.log.warning(f"Ignoring unrecognized reply: {reply}")
            except asyncio.CancelledError:
                return
            except Exception as e:
                if self.should_be_connected:
                    if self.communicator.connected:
                        err_msg = "read_loop failed; possibly a bug"
                        self.log.exception(err_msg)
                        self.fault(
                            code=enums.CscErrorCode.INTERNAL_ERROR,
                            report=f"{err_msg}: {e}",
                        )
                    else:
                        self.fault(
                            code=enums.CscErrorCode.CONNECTION_LOST,
                            report="connection lost to low-level controller (noticed in read_loop)",
                        )
        self.log.debug("read loop ends")

    async def start(self):
        await super().start()
        self.log.debug("Waiting for the Rotator remote to start")
        await self.rotator.start_task
        self.log.debug("Started")

    async def start_mock_ctrl(self):
        """Start the mock controller.

        The simulation mode must be 1.
        """
        try:
            assert self.simulation_mode == 1
            if self.mock_reply_port is not None:
                reply_port = self.mock_reply_port
            else:
                reply_port = self.config.reply_port
            self.mock_controller = mock.Controller(reply_port=reply_port, log=self.log)
            await asyncio.wait_for(self.mock_controller.start_task, timeout=2)
        except Exception as e:
            err_msg = "Could not start mock controller"
            self.log.exception(e)
            self.fault(
                code=enums.CscErrorCode.MOCK_CONTROLLER_ERROR, report=f"{err_msg}: {e}"
            )

    async def do_clearError(self, data):
        self.assert_enabled()
        raise salobj.ExpectedError("Not yet implemented")

    async def do_closeMirrorCovers(self, data):
        self.assert_enabled()
        # Deploy the mirror cover locks/guides.
        await self.send_commands(
            commands.MirrorCoverLocksPower(drive=-1, on=True),
            commands.MirrorCoverLocksMoveAll(drive=-1, deploy=True),
            commands.MirrorCoverLocksPower(drive=-1, on=False),
        )
        # Deploy the mirror covers.
        await self.send_commands(
            commands.MirrorCoversPower(drive=-1, on=True),
            commands.MirrorCoversClose(drive=-1),
            commands.MirrorCoversPower(drive=-1, on=False),
        )

    async def do_openMirrorCovers(self, data):
        self.assert_enabled()
        # Retract the mirror covers.
        await self.send_commands(
            commands.MirrorCoversPower(drive=-1, on=True),
            commands.MirrorCoversOpen(drive=-1),
            commands.MirrorCoversPower(drive=-1, on=False),
        )
        # Retract the mirror cover locks/guides.
        await self.send_commands(
            commands.MirrorCoverLocksPower(drive=-1, on=True),
            commands.MirrorCoverLocksMoveAll(drive=-1, deploy=False),
            commands.MirrorCoverLocksPower(drive=-1, on=False),
        )

    async def do_disableCameraCableWrapTracking(self, data):
        self.assert_enabled()
        self.camera_cable_wrap_task.cancel()
        await self.send_command(commands.CameraCableWrapEnableTracking(on=False))

    async def do_enableCameraCableWrapTracking(self, data):
        self.assert_enabled()
        await self.send_command(commands.CameraCableWrapEnableTracking(on=True))
        self.camera_cable_wrap_task = asyncio.create_task(self.camera_cable_wrap_loop())

    async def do_moveToTarget(self, data):
        self.assert_enabled()
        await self.send_command(
            commands.BothAxesMove(azimuth=data.azimuth, elevation=data.elevation,),
        )

    async def do_trackTarget(self, data):
        self.assert_enabled()
        tai_astropy = salobj.astropy_time_from_tai_unix(data.taiTime)
        await self.send_command(
            commands.BothAxesTrack(
                azimuth=data.azimuth,
                azimuth_velocity=data.azimuthVelocity,
                elevation=data.elevation,
                elevation_velocity=data.elevationVelocity,
                tai_time=tai_astropy,
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
            commands.ElevationAxisEnableTracking(on=True),
            commands.AzimuthAxisEnableTracking(on=True),
        )

    async def do_stop(self, data):
        self.assert_enabled()
        await self.send_commands(
            commands.BothAxesStop(), commands.CameraCableWrapStop(), dolock=False,
        )

    async def do_stopTracking(self, data):
        self.assert_enabled()
        await self.send_commands(
            commands.ElevationAxisEnableTracking(on=False),
            commands.AzimuthAxisEnableTracking(on=False),
        )

    @classmethod
    def add_arguments(cls, parser):
        super(MTMountCsc, cls).add_arguments(parser)
        parser.add_argument(
            "-s", "--simulate", action="store_true", help="Run in simuation mode?"
        )

    @classmethod
    def add_kwargs_from_args(cls, args, kwargs):
        super(MTMountCsc, cls).add_kwargs_from_args(args, kwargs)
        kwargs["simulation_mode"] = 1 if args.simulate else 0
