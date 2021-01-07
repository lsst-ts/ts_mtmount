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

__all__ = ["Controller"]

import argparse
import asyncio
import json
import logging
import signal

from lsst.ts import salobj
from lsst.ts import hexrotcomm
from .. import constants
from .. import commands
from .. import enums
from .. import replies
from .. import communicator

# from . import device
from .axis_device import AxisDevice
from .main_power_supply_device import MainPowerSupplyDevice
from .mirror_covers_device import MirrorCoversDevice
from .mirror_cover_locks_device import MirrorCoverLocksDevice
from .oil_supply_system_device import OilSupplySystemDevice
from .top_end_chiller_device import TopEndChillerDevice


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

    If the commander is `Source.HHD` then acknowledge all commands and mark
    as done. If not, then reject all commands except `Command.ASK_FOR_COMMAND`.
    Also output a few other replies, to exercise the code.

    Parameters
    ----------
    command_port : `int`
        Port for reading commands (that the CSC writes).
        The reply port is one greater than the command port.
    telemetry_port : `int`
        Port for the telemetry server.
    log : `logging.Logger`
        Logger.
    reconnect : `bool`, optional
        Try to reconnect if the connection is lost?
        Defaults to False for unit tests.
    commander : `Source`, optional
        Who initially has command. Defaults to `Source.NONE`,
        which matches the real controller. Two values are special:

        * `Source.HHD`: there is no need to issue `Command.ASK_FOR_COMMAND`
          before issuing other commands. This can simplify unit tests.
    """

    def __init__(
        self,
        command_port,
        telemetry_port,
        log,
        reconnect=False,
        commander=enums.Source.NONE,
    ):
        self.command_port = command_port
        self.log = log.getChild("MockController")
        self.reconnect = reconnect
        self.commander = enums.Source(commander)
        self.closing = False
        self.telemetry_interval = 0.2  # Seconds
        self.telemetry_server = hexrotcomm.OneClientServer(
            name="MockControllerTelemetry",
            host=salobj.LOCAL_HOST,
            port=telemetry_port,
            log=self.log,
            connect_callback=self.telemetry_connect_callback,
        )

        # Maximum position and velocity error,
        # below which an axis is considered in position
        self.max_position_error = 0.01  # degrees
        self.max_velocity_error = 0.01  # degrees/second
        # A dict of device_id: in_position
        self.in_position_dict = {
            enums.DeviceId.AZIMUTH_AXIS: None,
            enums.DeviceId.ELEVATION_AXIS: None,
        }

        self.communicator = None

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
        self.telemetry_task = asyncio.Future()
        self.start_task = asyncio.create_task(self.start())
        self.connect_task = salobj.make_done_future()
        self.done_task = asyncio.Future()
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, self.signal_handler)

    @property
    def connected(self):
        return self.communicator is not None and self.communicator.connected

    @classmethod
    async def amain(cls):
        parser = argparse.ArgumentParser("Simulate the TMA controller")
        parser.add_argument(
            "--command-port",
            type=int,
            default=constants.CSC_COMMAND_PORT,
            help="TCP/IP port for commands.",
        )
        parser.add_argument(
            "--telemetry-port",
            type=int,
            default=constants.TELEMETRY_PORT,
            help="TCP/IP port for telemetry.",
        )
        parser.add_argument(
            "--loglevel",
            type=int,
            default=logging.INFO,
            help="Log level (DEBUG=10, INFO=20, WARNING=30).",
        )
        parser.add_argument(
            "--noreconnect",
            action="store_true",
            help="Shut down when the the CSC disconnects?",
        )
        namespace = parser.parse_args()
        log = logging.getLogger("TMASimulator")
        log.setLevel(namespace.loglevel)
        print(
            "Mock TMA controller: "
            f"command_port={namespace.command_port}; "
            f"telemetry_port={namespace.telemetry_port}; "
            f"reconnect={not namespace.noreconnect}"
        )
        mock_controller = cls(
            command_port=namespace.command_port,
            telemetry_port=namespace.telemetry_port,
            log=log,
            reconnect=not namespace.noreconnect,
        )
        try:
            print("Mock TMA controller starting")
            await mock_controller.start_task
            print("Mock TMA controller running")
            await mock_controller.done_task
        except asyncio.CancelledError:
            print("Mock TMA controller done")
        except Exception as e:
            print(f"Mock TMA controller failed: {e}")

    def telemetry_connect_callback(self, server):
        """Called when a client connects to or disconnects from
        the telemetry port.
        """
        self.telemetry_task.cancel()
        if server.connected:
            self.log.info("Telemetry server connected; start telemetry loop")
            self.telemetry_task = asyncio.create_task(self.telemetry_loop())
        else:
            self.log.info("Telemetry server disconnected; stop telemetry loop")

    async def put_axis_telemetry(self, device_id, tai):
        """Warning: this minimal and simplistic."""
        prefix, topic_id = {
            enums.DeviceId.AZIMUTH_AXIS: ("az", enums.TelemetryTopicId.AZIMUTH),
            enums.DeviceId.ELEVATION_AXIS: ("el", enums.TelemetryTopicId.ELEVATION),
        }[device_id]
        device = self.device_dict[device_id]
        actuator = device.actuator
        target = actuator.target.at(tai)
        actual = actuator.path.at(tai)

        data_dict = {
            "topicID": topic_id,
            f"{prefix}AngleActual": actual.position,
            f"{prefix}AngleSet": target.position,
            f"{prefix}VelocityActual": actual.velocity,
            f"{prefix}VelocitySet": target.velocity,
            f"{prefix}AccelerationActual": actual.acceleration,
            # Torque is arbitrary; I have no idea
            # what realistic values are.
            f"{prefix}TorqueActual": actual.acceleration / 10,
            f"{prefix}TimeStamp": tai,
        }
        await self.write_telemetry(data_dict)

        # Write the InPosition TCP/IP message, if command/event
        # communicator connected.
        in_position = (
            device.has_target
            and abs(target.position - actual.position) < self.max_position_error
            and abs(target.velocity - actual.velocity) < self.max_velocity_error
        )
        if in_position != self.in_position_dict[device_id]:
            self.in_position_dict[device_id] = in_position
            what = {enums.DeviceId.AZIMUTH_AXIS: 0, enums.DeviceId.ELEVATION_AXIS: 1}[
                device_id
            ]
            if self.connected:
                reply = replies.InPositionReply(what=what, in_position=in_position)
                await self.communicator.write(reply)

    async def put_camera_cable_wrap_telemetry(self, tai):
        """Warning: this minimal and simplistic."""
        device = self.device_dict[enums.DeviceId.CAMERA_CABLE_WRAP]
        actuator = device.actuator
        actual = actuator.path.at(tai)
        kind = actuator.kind(tai)

        if device.power_on:
            drive_status = {
                actuator.Kind.Stopped: "Standstill",
                actuator.Kind.Tracking: "Discrete Motion",
                actuator.Kind.Slewing: "Discrete Motion",
                actuator.Kind.Stopping: "Stopping",
            }.get(kind, "Unknown")
        else:
            drive_status = "Off"

        data_dict = dict(
            topicID=enums.TelemetryTopicId.CAMERA_CABLE_WRAP,
            cCWStatus="Enabled" if device.power_on else "Off",
            cCWStatusDrive1=drive_status,
            cCWStatusDrive2="Off",  # Only one drive is on; assume drive 1
            cCWAngle1=actual.position,
            cCWAngle2=actual.position,
            cCWSpeed1=actual.velocity,
            cCWSpeed2=actual.velocity,
            cCWCurrent1=0,
            cCWCurrent2=0,
            timestamp=tai,
        )
        await self.write_telemetry(data_dict)

    async def write_telemetry(self, data_dict):
        data_str = json.dumps(data_dict)
        self.telemetry_server.writer.write(data_str.encode() + b"\r\n")
        await self.telemetry_server.writer.drain()

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
            raise

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
            self.add_device(AxisDevice, device_id=device_id)
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
        self.connect_task = asyncio.create_task(self.connect())

    async def connect(self):
        self.log.info("Connect to the CSC")
        self.read_loop_task.cancel()
        if self.closing:
            self.log.warning("Cannot connect: closing or closed")
            return
        try:
            self.communicator = communicator.Communicator(
                name="MockController",
                client_host=salobj.LOCAL_HOST,
                # Tekniker uses repy port = command port + 1
                client_port=self.command_port + 1,
                server_host=salobj.LOCAL_HOST,
                server_port=self.command_port,
                log=self.log,
                read_replies=False,
                connect=False,
                connect_callback=self.connect_callback,
            )
            self.log.debug("Waiting for the communicator to start")
            await self.communicator.start_task
        except Exception as e:
            asyncio.create_task(self.close(exception=e))
            return

        try:
            self.log.debug("Connecting to the CSC")
            await self.communicator.connect()
            self.log.debug("Connected")
            self.read_loop_task = asyncio.create_task(self.read_loop())
        except asyncio.CancelledError:
            self.log.info("Connection cancelled")
        except Exception:
            self.log.exception("Could not connect")

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
            self.telemetry_task.cancel()
            self.connect_task.cancel()
            self.start_task.cancel()
            for device in self.device_dict.values():
                await device.close()
            if self.communicator is not None:
                await self.communicator.close()
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
            self.commander != enums.Source.HHD
            and command.command_code != enums.CommandCode.ASK_FOR_COMMAND
        ):
            await self.write_noack(
                command=command,
                explanation=f"The commander is {self.commander!r}, not the CSC/HHD.",
            )
            return

        command_func = self.command_dict.get(command.command_code)
        if command_func is None:
            await self.write_noack(
                command=command, explanation="This command is not yet supported"
            )
            return

        try:
            timeout_task = command_func(command)
        except Exception as e:
            await self.write_noack(command=command, explanation=repr(e))
            return
        if timeout_task is None:
            await self.write_ack(command, timeout=None)
            if command.command_code not in commands.AckOnlyCommandCodes:
                await self.write_done(command)
        else:
            timeout, task = timeout_task
            await self.write_ack(command, timeout=timeout)
            # Do not bother to save the task because `Controller.close` calls
            # `BaseDevice.close` on each mock device, which cancels the task
            # that `Controller.monitor_command` is awaiting.
            asyncio.create_task(self.monitor_command(command=command, task=task))

    async def read_loop(self):
        self.log.debug("Read loop begins")
        try:
            while self.connected:
                command = await self.communicator.read()
                if self.command_queue and not self.command_queue.full():
                    self.command_queue.put_nowait(command)
                asyncio.create_task(self.handle_command(command))
        except asyncio.CancelledError:
            pass
        except (ConnectionResetError, asyncio.IncompleteReadError):
            self.log.warning("Connection lost")
            if self.closing:
                return
            if self.reconnect:
                self.connect_task.cancel()
                await self.communicator.close()
                if not self.closing:
                    self.connect_task = asyncio.create_task(self.connect())
            else:
                asyncio.create_task(self.close())
        self.log.debug("Read loop ends")

    async def reply_to_command(self, command):
        try:
            await self.communicator.write(
                replies.AckReply(sequence_id=command.sequence_id, timeout_ms=1000)
            )
            if command.command_code not in commands.AckOnlyCommandCodes:
                await asyncio.sleep(0.1)
                await self.communicator.write(
                    replies.DoneReply(sequence_id=command.sequence_id)
                )
        except Exception:
            self.log.exception(f"reply_to_command({command}) failed")
            raise

    def signal_handler(self):
        asyncio.create_task(self.close())

    def connect_callback(self, server):
        state_str = "connected to" if server.connected else "disconnected from"
        self.log.info(f"Mock controller {state_str} the CSC")

    def do_ask_for_command(self, command):
        """Handle ASK_FOR_COMMAND.

        For this mock controller to accept other commands,
        the commander must be `enums.Source.HHD`.

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
            sequence_id=command.sequence_id, position=command.azimuth,
        )
        elevation_command = commands.ElevationAxisMove(
            sequence_id=command.sequence_id, position=command.elevation,
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
            if self.connected:
                await self.write_done(command)
        except asyncio.CancelledError:
            if self.connected:
                await self.write_noack(command, explanation="Superseded")
        except Exception as e:
            if self.connected:
                await self.write_noack(command, explanation=str(e))

    async def write_ack(self, command, timeout):
        """Report a command as acknowledged.

        Parameters
        ----------
        command : `Command`
            Command to report as acknowledged.
        timeout : `float` or `None`
            Timeout for command (second)
        """
        if timeout is None:
            timeout = 0
        reply = replies.AckReply(
            sequence_id=command.sequence_id,
            source=command.source,
            timeout_ms=int(timeout * 1000),
        )
        await self.communicator.write(reply)

    async def write_done(self, command):
        """Report a command as done.

        Parameters
        ----------
        command : `Command`
            Command to report as done.
        """
        reply = replies.DoneReply(
            sequence_id=command.sequence_id, source=command.source,
        )
        await self.communicator.write(reply)

    async def write_noack(self, command, explanation):
        """Report a command as failed.

        Parameters
        ----------
        command : `Command`
            Command to report as failed.
        explanation : `str`
            Reason for the failure.
        """
        reply = replies.NoAckReply(
            sequence_id=command.sequence_id,
            source=command.source,
            explanation=explanation,
        )
        await self.communicator.write(reply)
