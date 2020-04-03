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


class Controller:
    """Simulate the most basic responses from the Operation Manager.

    Acknowledge all commands and mark as done.
    Also output a few other replies, to exercise the code.

    Parameters
    ----------
    reply_port : `int`
        Port for writing replies (that the CSC reads).
        The command port is one greater than the reply port.
    log : `logging.Logger`
        Logger.
    """

    def __init__(self, reply_port, log):
        self.log = log.getChild("Controller")
        self.communicator = communicator.Communicator(
            name="Controller",
            client_host=salobj.LOCAL_HOST,
            client_port=reply_port,
            server_host=salobj.LOCAL_HOST,
            server_port=reply_port + 1,
            log=self.log,
            read_replies=False,
            connect_client=False,
            connect_callback=self.connect_callback,
        )
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
        self.start_task = asyncio.create_task(self.start())

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
        """Create the command queue.

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
        self.command_code = None

    async def start(self):
        self.log.debug("start: connecting")
        await self.communicator.connect()
        self.log.debug("start: connected")
        self.read_loop_task = asyncio.create_task(self.read_loop())

    async def disconnect(self):
        await self.communicator.disconnect()

    async def close(self):
        self.read_loop_task.cancel()
        await self.communicator.close()

    async def handle_command(self, command):
        command_func = self.command_dict.get(command.command_code)
        if command_func is None:
            await self.write_noack(
                command=command, explanation="This command is not yet supported"
            )

        try:
            timeout = command_func(command)
        except Exception as e:
            await self.write_noack(command=command, explanation=repr(e))
            return
        await self.write_ack(command, timeout=timeout)
        if timeout is None and command.command_code not in commands.AckOnlyCommandCodes:
            await self.write_done(command)

    async def read_loop(self):
        self.log.debug("read_loop begins")
        try:
            while self.communicator.connected:
                command = await self.communicator.read()
                if self.command_queue and not self.command_queue.full():
                    self.command_queue.put_nowait(command)
                asyncio.create_task(self.handle_command(command))
        except asyncio.CancelledError:
            return
        except asyncio.streams.IncompleteReadError:
            self.log.error("Connection lost")
            self.close()
        self.log.debug("read_loop ends")

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
        azimuth_time = self.device_dict[enums.DeviceId.AZIMUTH_AXIS].do_move(
            azimuth_command
        )
        elevation_time = self.device_dict[enums.DeviceId.ELEVATION_AXIS].do_move(
            elevation_command
        )
        return max(azimuth_time, elevation_time)

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
            tai_time=command.tai_time,
        )
        elevation_command = commands.ElevationAxisTrack(
            sequence_id=command.sequence_id,
            position=command.elevation,
            velocity=command.elevation_velocity,
            tai_time=command.tai_time,
        )
        self.device_dict[enums.DeviceId.AZIMUTH_AXIS].do_track(azimuth_command)
        self.device_dict[enums.DeviceId.ELEVATION_AXIS].do_track(elevation_command)

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
