# This file is part of ts_MTMount.
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

__all__ = ["INITIAL_POSITION", "Controller"]

import argparse
import asyncio
import json
import logging
import signal

from lsst.ts import salobj
from lsst.ts import tcpip
from ..exceptions import CommandSupersededException
from .. import commands
from .. import constants
from .. import enums

# from . import device
from .axis_device import AxisDevice
from .main_power_supply_device import MainPowerSupplyDevice
from .mirror_covers_device import MirrorCoversDevice
from .mirror_cover_locks_device import MirrorCoverLocksDevice
from .oil_supply_system_device import OilSupplySystemDevice
from .top_end_chiller_device import TopEndChillerDevice

# Dict of DeviceId: initial position (deg).
INITIAL_POSITION = {
    enums.DeviceId.ELEVATION_AXIS: 80,
    enums.DeviceId.AZIMUTH_AXIS: 0,
    enums.DeviceId.CAMERA_CABLE_WRAP: 0,
}


def make_reply_dict(id, **parameters):
    """Make a reply dict."""
    return dict(
        id=enums.ReplyCode(id),
        timestamp=salobj.current_tai(),
        parameters=parameters,
    )


async def wait_tasks(*tasks):
    """Wait for one or more tasks to finish; if one fails cancel the others.

    Like asyncio.gather but cancels the remaining tasks if one fails.
    """
    try:
        await asyncio.gather(*tasks)
    except Exception:
        for task in tasks:
            task.cancel()
        raise


class Controller:
    """Simulate the TMA controller (Operation Manager).

    If the commander is `Source.CSC` then acknowledge all commands and mark
    as done. If not, then reject all commands except `Command.ASK_FOR_COMMAND`.
    Also output a few other replies, to exercise the code.

    Parameters
    ----------
    log : `logging.Logger`
        Logger.
    commander : `Source`, optional
        Who initially has command. Defaults to `Source.NONE`,
        which matches the real controller. Two values are special:

        * `Source.CSC`: there is no need to issue `Command.ASK_FOR_COMMAND`
          before issuing other commands. This can simplify unit tests.
        * `Source.HHD`: `Command.ASK_FOR_COMMAND` is rejected for any
          other commander. This reflects the real system, because nobody
          can take command from the handheld device. This offers a convenient
          way to test `Command.ASK_FOR_COMMAND` failures.
    random_ports : `bool`
        Use random ports for commands and telemetry, instead of the
        standard ports? Random ports are intended for unit tests.
    """

    def __init__(self, log, commander=enums.Source.NONE, random_ports=False):
        self.log = log.getChild("MockController")
        self.commander = enums.Source(commander)
        self.closing = False
        self.telemetry_interval = 0.2  # Seconds

        # Maximum position and velocity error,
        # below which an axis is considered in position
        self.max_position_error = 0.01  # degrees
        self.max_velocity_error = 0.01  # degrees/second
        # A dict of device_id: in_position
        self.in_position_dict = {
            enums.DeviceId.AZIMUTH_AXIS: None,
            enums.DeviceId.ELEVATION_AXIS: None,
        }

        # Queue of commands, for unit testing
        self.command_queue = None

        # Dict of command_code: mock method to call
        self.command_dict = {}
        # Dict of DeviceId: mock device
        self.device_dict = {}
        self.add_all_devices()
        self.command_dict[enums.CommandCode.ASK_FOR_COMMAND] = self.do_ask_for_command
        self.command_dict[enums.CommandCode.BOTH_AXES_MOVE] = self.do_both_axes_move
        self.command_dict[enums.CommandCode.BOTH_AXES_STOP] = self.do_both_axes_stop
        self.command_dict[enums.CommandCode.BOTH_AXES_TRACK] = self.do_both_axes_track
        self.command_dict[enums.CommandCode.SAFETY_RESET] = self.do_safety_reset

        self.read_loop_task = asyncio.Future()
        self.telemetry_loop_task = asyncio.Future()
        self.telemetry_monitor_task = asyncio.Future()
        self.start_task = asyncio.create_task(self.start())
        self.done_task = asyncio.Future()

        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, self.signal_handler)

        command_port = 0 if random_ports else constants.CSC_COMMAND_PORT
        telemetry_port = 0 if random_ports else constants.TELEMETRY_PORT
        self.command_server = tcpip.OneClientServer(
            name="MockControllerCommands",
            host=salobj.LOCAL_HOST,
            port=command_port,
            log=self.log,
            connect_callback=self.command_connect_callback,
        )
        self.telemetry_server = tcpip.OneClientServer(
            name="MockControllerTelemetry",
            host=salobj.LOCAL_HOST,
            port=telemetry_port,
            log=self.log,
            connect_callback=self.telemetry_connect_callback,
        )

    @classmethod
    async def amain(cls):
        parser = argparse.ArgumentParser("Simulate the TMA controller")
        parser.add_argument(
            "--random-ports",
            action="store_true",
            default=False,
            help="Use random available ports for commands and telemetry? "
            "Intended for unit tests.",
        )
        parser.add_argument(
            "--loglevel",
            type=int,
            default=logging.INFO,
            help="Log level (DEBUG=10, INFO=20, WARNING=30).",
        )
        namespace = parser.parse_args()
        log = logging.getLogger("TMASimulator")
        log.setLevel(namespace.loglevel)
        print("Mock TMA controller")
        mock_controller = cls(
            random_ports=namespace.random_ports,
            log=log,
        )
        try:
            await mock_controller.start_task
            # Warning: this message is read by the CSC; if you change
            # the message please update the CSC:
            print(
                "Mock TMA controller running: "
                f"command_port={mock_controller.command_server.port}, "
                f"telemetry_port={mock_controller.telemetry_server.port}",
                flush=True,
            )
            await mock_controller.done_task
        except asyncio.CancelledError:
            print("Mock TMA controller done", flush=True)
        except Exception as e:
            print(f"Mock TMA controller failed: {e!r}", flush=True)

    async def put_axis_telemetry(self, device_id, tai):
        """Warning: this minimal and simplistic."""
        topic_id = {
            enums.DeviceId.AZIMUTH_AXIS: enums.TelemetryTopicId.AZIMUTH,
            enums.DeviceId.ELEVATION_AXIS: enums.TelemetryTopicId.ELEVATION,
        }[device_id]
        device = self.device_dict[device_id]
        actuator = device.actuator
        target = actuator.target.at(tai)
        actual = actuator.path.at(tai)

        data_dict = {
            "topicID": topic_id,
            "angleActual": actual.position,
            "angleSet": target.position,
            "velocityActual": actual.velocity,
            "velocitySet": target.velocity,
            "accelerationActual": actual.acceleration,
            # Torque is arbitrary; I have no idea
            # what realistic values are.
            "torqueActual": actual.acceleration / 10,
            "timestamp": tai,
        }
        await self.write_telemetry(data_dict)

        # Write the InPosition TCP/IP message, if command/event
        # server connected.
        in_position = (
            device.has_target
            and abs(target.position - actual.position) < self.max_position_error
            and abs(target.velocity - actual.velocity) < self.max_velocity_error
        )
        if in_position != self.in_position_dict[device_id]:
            self.in_position_dict[device_id] = in_position
            axis = {enums.DeviceId.AZIMUTH_AXIS: 0, enums.DeviceId.ELEVATION_AXIS: 1}[
                device_id
            ]
            if self.command_server.connected:
                await self.write_reply(
                    make_reply_dict(
                        id=enums.ReplyCode.IN_POSITION,
                        axis=axis,
                        inPosition=in_position,
                    )
                )

    async def put_camera_cable_wrap_telemetry(self, tai):
        """Warning: this minimal and simplistic."""
        device = self.device_dict[enums.DeviceId.CAMERA_CABLE_WRAP]
        actuator = device.actuator
        actual = actuator.path.at(tai)

        data_dict = dict(
            topicID=enums.TelemetryTopicId.CAMERA_CABLE_WRAP,
            angle=actual.position,
            speed=actual.velocity,
            acceleration=actual.acceleration,
            timestamp=tai,
        )
        await self.write_telemetry(data_dict)

    async def write_reply(self, reply_dict):
        """Write a reply to the command/reply stream.

        Parameters
        ----------
        reply_dict : `dict`
            Reply as a dict.
            It will be formatted as json before being written.
        """
        self.log.debug("write_reply(%s)", reply_dict)
        reply_str = json.dumps(reply_dict)
        self.command_server.writer.write(reply_str.encode())
        self.command_server.writer.write(constants.LINE_TERMINATOR)
        await self.command_server.writer.drain()

    async def write_telemetry(self, data_dict):
        data_str = json.dumps(data_dict)
        self.telemetry_server.writer.write(data_str.encode())
        self.telemetry_server.writer.write(constants.LINE_TERMINATOR)
        await self.telemetry_server.writer.drain()

    def telemetry_connect_callback(self, server):
        """Called when a client connects to or disconnects from
        the telemetry port.
        """
        self.telemetry_loop_task.cancel()
        self.telemetry_monitor_task.cancel()
        if server.connected:
            self.log.info("Telemetry server connected; start telemetry loop")
            self.telemetry_monitor_task = asyncio.create_task(self.telemetry_monitor())
            self.telemetry_loop_task = asyncio.create_task(self.telemetry_loop())
        else:
            self.log.info("Telemetry server disconnected; stop telemetry loop")

    async def telemetry_loop(self):
        """Warning: this minimal and simplistic."""
        try:
            while self.telemetry_server.connected:
                tai = salobj.current_tai()
                await self.put_axis_telemetry(
                    device_id=enums.DeviceId.AZIMUTH_AXIS, tai=tai
                )
                await self.put_axis_telemetry(
                    device_id=enums.DeviceId.ELEVATION_AXIS, tai=tai
                )
                await self.put_camera_cable_wrap_telemetry(tai=tai)
                await asyncio.sleep(self.telemetry_interval)
        except ConnectionResetError:
            self.log.warning("Disconnected")
        except asyncio.CancelledError:
            self.log.info("Telemetry loop cancelled")
            pass
        except Exception:
            self.log.exception("Telemetry loop failed")
            await self.telemetry_server.close_client()
            raise

    async def telemetry_monitor(self):
        """Monitor the telemetry reader for disconnection."""
        try:
            while self.telemetry_server.connected:
                read_bytes = await self.telemetry_server.reader.read(1000)
                if len(read_bytes) > 0:
                    self.log.warning(
                        f"Ignoring unexpected data from the telemetry port: {read_bytes}"
                    )
        except asyncio.CancelledError:
            pass
        except (ConnectionResetError, asyncio.IncompleteReadError):
            self.log.warning("Telemetry connection lost")
            await self.telemetry_server.close_client()
        except Exception:
            self.log.exception("Telemetry monitoring read failed")
            await self.telemetry_server.close_client()
        finally:
            # Make sure telemetry stops, even if telemetry_connect_callback
            # is not called.
            self.telemetry_loop_task.cancel()
        self.log.debug("Telemetry monitor read ends")

    def add_all_devices(self):
        """Add all mock devices.

        The devices are added to self.device_dict.

        Parameters
        ----------
        device_id : `DeviceId`
            Device identifier.
        """
        for device_id in (
            enums.DeviceId.ELEVATION_AXIS,
            enums.DeviceId.AZIMUTH_AXIS,
            enums.DeviceId.CAMERA_CABLE_WRAP,
        ):
            self.add_device(
                AxisDevice,
                device_id=device_id,
                start_position=INITIAL_POSITION[device_id],
            )
        self.add_device(MainPowerSupplyDevice)
        self.add_device(MirrorCoverLocksDevice)
        self.add_device(MirrorCoversDevice)
        self.add_device(OilSupplySystemDevice)
        self.add_device(TopEndChillerDevice)

    def add_device(self, device_class, **kwargs):
        """Add a mock device.

        The device is added to self.device_dict.

        Parameters
        ----------
        device_class : `mock.Device`
            Mock device class.
        **kwargs : `dict`
            Additional arguments for ``device_class``.
        """
        device = device_class(controller=self, **kwargs)
        self.device_dict[device.device_id] = device

    def set_command_queue(self, maxsize=0):
        """Create or replace the command queue.

        This sets attribute ``self.command_queue`` to an `asyncio.Queue`
        and uses it to record commands as they are received.

        Parameters
        ----------
        maxsize : `int`, optional
            Maximum number of items on the queue.
            If <= 0 then no limit.
            If the queue gets full then the newest items are dropped.
        """
        self.command_queue = asyncio.Queue(maxsize=maxsize)

    def delete_command_queue(self):
        """Delete the command queue, if present.

        This sets ``self.command_queue=None``.
        """
        self.command_queue = None

    async def start(self):
        await self.command_server.start_task
        await self.telemetry_server.start_task

    async def close(self, exception=None):
        """Close the controller.

        Parameters
        ----------
        exception : `Exception` or `None`
            Exception to which to set done_task, or None.
        """
        self.log.info("Closing")
        if self.closing:
            if not self.done_task.done():
                await self.done_task
            return

        try:
            self.closing = True
            self.read_loop_task.cancel()
            self.telemetry_loop_task.cancel()
            self.telemetry_monitor_task.cancel()
            self.start_task.cancel()
            for device in self.device_dict.values():
                await device.close()
            await self.command_server.close()
            await self.telemetry_server.close()
        except Exception:
            self.log.exception("close failed")
        finally:
            if not self.done_task.done():
                if exception:
                    self.done_task.set_exception(exception)
                else:
                    self.done_task.set_result(None)

    async def handle_command(self, command):
        if (
            self.commander != enums.Source.CSC
            and command.command_code != enums.CommandCode.ASK_FOR_COMMAND
        ):
            await self.write_cmd_rejected(
                command=command,
                explanation=f"The commander is {self.commander!r}, not the CSC.",
            )
            return

        command_func = self.command_dict.get(command.command_code)
        if command_func is None:
            await self.write_cmd_rejected(
                command=command, explanation="This command is not yet supported"
            )
            return

        try:
            timeout_task = command_func(command)
        except Exception as e:
            await self.write_cmd_rejected(command=command, explanation=repr(e))
            return
        if command.command_code in commands.AckOnlyCommandCodes:
            # Command is done when acknowledged;
            # timeout=-1 is a special value to indicate this.
            await self.write_cmd_acknowledged(command, timeout=-1)
        elif timeout_task is None:
            # Command takes no time. Note: timeout for commands that are
            # not done when acknowledged must be > 0, so pick a small value.
            await self.write_cmd_acknowledged(command, timeout=0.1)
            await self.write_cmd_succeeded(command)
        else:
            timeout, task = timeout_task
            await self.write_cmd_acknowledged(command, timeout=timeout)
            # Do not bother to save the task because `Controller.close` calls
            # `BaseDevice.close` on each mock device, which cancels the task
            # that `Controller.monitor_command` is awaiting.
            asyncio.create_task(self.monitor_command(command=command, task=task))

    async def read_loop(self):
        self.log.debug("Read loop begins")
        try:
            while self.command_server.connected:
                read_bytes = await self.command_server.reader.readuntil(
                    constants.LINE_TERMINATOR
                )
                try:
                    command = commands.parse_command(read_bytes.decode())
                except Exception as e:
                    self.log.error(f"Ignoring unparsable command {read_bytes}: {e!r}")
                    continue
                if self.command_queue and not self.command_queue.full():
                    self.command_queue.put_nowait(command)
                asyncio.create_task(self.handle_command(command))
        except asyncio.CancelledError:
            pass
        except (ConnectionResetError, asyncio.IncompleteReadError):
            self.log.warning("Connection lost")
            await self.command_server.close_client()
        except Exception:
            self.log.exception("Read loop failed")
            await self.command_server.close_client()
        self.log.debug("Read loop ends")

    async def reply_to_command(self, command):
        if not self.command_server.connected:
            raise RuntimeError(f"reply_to_command({command}) failed: not connected")
        try:
            await self.write_cmd_acknowledged(command, timeout=1)
            if command.command_code not in commands.AckOnlyCommandCodes:
                await asyncio.sleep(0.1)
                if not self.command_server.connected:
                    raise RuntimeError(
                        f"reply_to_command({command}) failed: disconnected before writing Done"
                    )
                await self.write_cmd_succeeded(command)
        except Exception:
            self.log.exception(f"reply_to_command({command}) failed")
            raise

    def command_connect_callback(self, server):
        state_str = "connected to" if server.connected else "disconnected from"
        self.log.info(f"Mock controller {state_str} the CSC")
        self.read_loop_task.cancel()
        if not server.connected:
            return

        if self.closing:
            self.log.warning("Cannot connect: closing or closed")
            return

        self.read_loop_task = asyncio.create_task(self.read_loop())

    def do_ask_for_command(self, command):
        """Handle ASK_FOR_COMMAND.

        For this mock controller to accept other commands,
        the commander must be `enums.Source.CSC`.

        If the HHD has command then no other commander can change it.
        This reflects reality and offers a nice way to test what happens
        if ASK_FOR_COMMAND fails.
        """
        if (
            self.commander == enums.Source.HHD
            and command.commander != enums.Source.HHD
            and command.source != enums.Source.HHD
        ):
            raise RuntimeError(
                f"HHD has command; cannot give command to {command.commander!r}; "
                f"from source={command.source}"
            )
        self.commander = command.commander

    def do_both_axes_move(self, command):
        azimuth_command = commands.AzimuthAxisMove(
            sequence_id=command.sequence_id,
            position=command.azimuth,
        )
        elevation_command = commands.ElevationAxisMove(
            sequence_id=command.sequence_id,
            position=command.elevation,
        )
        azimuth_time, azimuth_task = self.device_dict[
            enums.DeviceId.AZIMUTH_AXIS
        ].do_move(azimuth_command)
        elevation_time, elevation_task = self.device_dict[
            enums.DeviceId.ELEVATION_AXIS
        ].do_move(elevation_command)
        timeout = max(azimuth_time, elevation_time)
        task = asyncio.create_task(wait_tasks(azimuth_task, elevation_task))
        return timeout, task

    def do_both_axes_stop(self, command):
        azimuth_command = commands.AzimuthAxisStop(sequence_id=command.sequence_id)
        elevation_command = commands.ElevationAxisStop(sequence_id=command.sequence_id)
        self.device_dict[enums.DeviceId.AZIMUTH_AXIS].do_stop(azimuth_command)
        self.device_dict[enums.DeviceId.ELEVATION_AXIS].do_stop(elevation_command)

    def do_both_axes_track(self, command):
        azimuth_command = commands.AzimuthAxisTrack(
            sequence_id=command.sequence_id,
            position=command.azimuth,
            velocity=command.azimuth_velocity,
            tai=command.tai,
        )
        elevation_command = commands.ElevationAxisTrack(
            sequence_id=command.sequence_id,
            position=command.elevation,
            velocity=command.elevation_velocity,
            tai=command.tai,
        )
        self.device_dict[enums.DeviceId.AZIMUTH_AXIS].do_track(azimuth_command)
        self.device_dict[enums.DeviceId.ELEVATION_AXIS].do_track(elevation_command)

    def do_safety_reset(self, command):
        """This is presently a no-op."""
        pass

    async def monitor_command(self, command, task):
        try:
            await task
            if self.command_server.connected:
                await self.write_cmd_succeeded(command)
        except asyncio.CancelledError:
            if self.command_server.connected:
                await self.write_cmd_superseded(command, superseded_by=None)
        except CommandSupersededException as e:
            if self.command_server.connected:
                await self.write_cmd_superseded(command, superseded_by=e.command)
        except Exception as e:
            if self.command_server.connected:
                await self.write_cmd_failed(command, explanation=str(e))

    def signal_handler(self):
        asyncio.create_task(self.close())

    async def write_cmd_acknowledged(self, command, timeout):
        """Report a command as acknowledged.

        Parameters
        ----------
        command : `Command`
            Command to report as acknowledged.
        timeout : `float`
            Timeout for command (second)
        """
        await self.write_reply(
            make_reply_dict(
                id=enums.ReplyCode.CMD_ACKNOWLEDGED,
                commander=command.source,
                sequenceId=command.sequence_id,
                timeout=timeout,
            )
        )

    async def write_cmd_failed(self, command, explanation):
        """Report a command as failed (after acknowledged).

        Parameters
        ----------
        command : `Command`
            Command to report as failed.
        explanation : `str`
            Reason for the failure.
        """
        await self.write_reply(
            make_reply_dict(
                id=enums.ReplyCode.CMD_FAILED,
                commander=command.source,
                sequenceId=command.sequence_id,
                explanation=explanation,
            )
        )

    async def write_cmd_rejected(self, command, explanation):
        """Report a command as rejected (failed before acknowledged).

        Parameters
        ----------
        command : `Command`
            Command to report as failed.
        explanation : `str`
            Reason for the failure.
        """
        await self.write_reply(
            make_reply_dict(
                id=enums.ReplyCode.CMD_REJECTED,
                commander=command.source,
                sequenceId=command.sequence_id,
                explanation=explanation,
            )
        )

    async def write_cmd_succeeded(self, command):
        """Report a command as done.

        Parameters
        ----------
        command : `Command`
            Command to report as done.
        """
        await self.write_reply(
            make_reply_dict(
                id=enums.ReplyCode.CMD_SUCCEEDED,
                commander=command.source,
                sequenceId=command.sequence_id,
            )
        )

    async def write_cmd_superseded(self, command, superseded_by):
        """Report a command as done.

        Parameters
        ----------
        command : `Command`
            Command to report as done.
        superseded_by : `Command` or `None`
            Superseding command, if known.
        """
        if superseded_by is None:
            superseding_kwargs = dict(
                supersedingSequenceId=0,
                supersedingCommander=0,
                supersedingCommandCode=0,
            )
        else:
            superseding_kwargs = dict(
                supersedingSequenceId=superseded_by.sequence_id,
                supersedingCommander=superseded_by.source,
                supersedingCommandCode=superseded_by.command_code,
            )
        await self.write_reply(
            make_reply_dict(
                id=enums.ReplyCode.CMD_SUPERSEDED,
                commander=command.source,
                sequenceId=command.sequence_id,
                **superseding_kwargs,
            )
        )
