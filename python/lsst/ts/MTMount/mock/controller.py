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

import asyncio

from lsst.ts import salobj
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
    """Simulate the most basic responses from the low-level controller
    (Operation Manager).

    Acknowledge all commands and mark as done.
    Also output a few other replies, to exercise the code.

    Parameters
    ----------
    command_port : `int`
        Port for reading commands (that the CSC writes).
        The reply port is one greater than the command port.
    log : `logging.Logger`
        Logger.
    reconnect : `bool` (optional)
        Try to reconnect if the connection is lost?
        Defaults to False for unit tests.
    """

    def __init__(self, command_port, log, reconnect=False):
        self.command_port = command_port
        self.log = log.getChild("Controller")
        self.reconnect = reconnect
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

        self.communicator = None
        self.sal_controller = None

        # Queue of commands, for unit testing
        self.command_queue = None

        # Dict of command_code: mock method to call
        self.command_dict = {}
        # Dict of DeviceId: mock device
        self.device_dict = {}
        self.add_all_devices()
        self.command_dict[enums.CommandCode.BOTH_AXES_MOVE] = self.do_both_axes_move
        self.command_dict[enums.CommandCode.BOTH_AXES_STOP] = self.do_both_axes_stop
        self.command_dict[enums.CommandCode.BOTH_AXES_TRACK] = self.do_both_axes_track

        self.read_loop_task = asyncio.Future()
        self.telemetry_task = asyncio.Future()
        self.start_task = asyncio.create_task(self.connect())

    @property
    def connected(self):
        return self.communicator is not None and self.communicator.connected

    async def put_axis_telemetry(self, device_id, tai):
        """Warning: this minimal and simplistic.
        """
        prefix = {
            enums.DeviceId.AZIMUTH_AXIS: "Azimuth",
            enums.DeviceId.ELEVATION_AXIS: "Elevation",
        }[device_id]
        device = self.device_dict[device_id]
        actuator = device.actuator
        target = actuator.target.at(tai)
        actual = actuator.path.at(tai)

        # Write the SAL telemetry topic
        topic = getattr(self.sal_controller, f"tel_{prefix}")
        state_strs = [
            "On" if device.power_on else "Off",
            "DriveEnabled" if device.enabled else "DriveDisabled",
            "TrackingEnabled" if device.tracking_enabled else "TrackingDisabled",
        ]
        kwargs = {
            f"{prefix}_Status": "/".join(state_strs),
            f"{prefix}_Angle_Set": target.position,
            f"{prefix}_Velocity_Set": target.velocity,
            f"{prefix}_Angle_Actual": actual.position,
            f"{prefix}_Velocity_Actual": actual.velocity,
            f"{prefix}_Aceleration_Actual": actual.acceleration,
            "timestamp": tai,
        }
        topic.set_put(**kwargs)

        # Write the InPosition TCP/IP message
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
            reply = replies.InPositionReply(what=what, in_position=in_position)
            await self.communicator.write(reply)

    async def put_camera_cable_wrap_telemetry(self, tai):
        """Warning: this minimal and simplistic.
        """
        device = self.device_dict[enums.DeviceId.CAMERA_CABLE_WRAP]
        actuator = device.actuator
        actual = actuator.path.at(tai)

        state_strs = [
            "On" if device.power_on else "Off",
            "DriveEnabled" if device.enabled else "DriveDisabled",
            "TrackingEnabled" if device.tracking_enabled else "TrackingDisabled",
        ]
        self.sal_controller.tel_Camera_Cable_Wrap.set_put(
            CCW_Status="/".join(state_strs),
            CCW_Angle_1=actual.position,
            CCW_Angle_2=actual.position,
            CCW_Speed_1=actual.velocity,
            CCW_Speed_2=actual.velocity,
            timestamp=tai,
        )

    async def telemetry_loop(self):
        """Warning: this minimal and simplistic.
        """
        try:
            while True:
                tai = salobj.current_tai()
                await self.put_axis_telemetry(
                    device_id=enums.DeviceId.AZIMUTH_AXIS, tai=tai
                )
                await self.put_axis_telemetry(
                    device_id=enums.DeviceId.ELEVATION_AXIS, tai=tai
                )
                await self.put_camera_cable_wrap_telemetry(tai=tai)
                await asyncio.sleep(self.telemetry_interval)
        except asyncio.CancelledError:
            raise
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
        maxsize : `int` (optional)
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

    async def connect(self):
        self.read_loop_task.cancel()
        self.telemetry_task.cancel()
        self.communicator = communicator.Communicator(
            name="Controller",
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
        self.sal_controller = salobj.Controller(name="MTMount")
        await self.sal_controller.start_task

        self.log.debug("Connecting to the CSC")
        await self.communicator.connect()
        self.log.debug("Connected")
        self.read_loop_task = asyncio.create_task(self.read_loop())
        self.telemetry_task = asyncio.create_task(self.telemetry_loop())

    async def close(self):
        self.read_loop_task.cancel()
        self.telemetry_task.cancel()
        if self.sal_controller is not None:
            await self.sal_controller.close()
            self.sal_controller = None
        await self.communicator.close()
        for device in self.device_dict.values():
            await device.close()
        self.communicator = None

    async def handle_command(self, command):
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
            return
        except asyncio.IncompleteReadError:
            self.log.warning("Connection lost")
            await self.close()
            if self.reconnect:
                asyncio.create_task(self.connect())
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

    def connect_callback(self, server):
        state_str = "connected to" if server.connected else "disconnected from"
        self.log.info(f"Mock controller {state_str} the CSC")

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
