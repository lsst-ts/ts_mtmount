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
import json
import time

import numpy as np

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
    """Simulate the most basic responses from the Operation Manager.

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
    telemetry_dict : `dict`
        Dict of telemetry topic name: data dict
        where data dict is a dict of key: value
        for the public fields of a telemetry topic.
    """

    def __init__(self, command_port, log, reconnect=False, telemetry_dict=None):
        self.command_port = command_port
        self.log = log.getChild("Controller")
        self.reconnect = reconnect

        if telemetry_dict is None:
            import yaml

            with open("telemetry_dict.yaml", "r") as f:
                data = f.read()
            telemetry_dict = yaml.safe_load(data)

        self.telemetry_dict = telemetry_dict
        self.telemetry_interval = 0.05  # Seconds between telemetry output
        self.telemetry_margins = []

        self.communicator = None

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

    async def telemetry_loop(self):
        # For now set fake data in all fields so the telemetry topics have
        # semi-realistic lengths; if we decide that the MCS will publish
        # telemetry then obviously the data will have to be properly set
        # to match the state of the mock devices.
        print("telemetry_loop: setting arbitrary telemetry values")
        for tel_name, data_dict in self.telemetry_dict.items():
            for name, data in data_dict.items():
                if type(data) is str:
                    data = "idle"
                elif type(data) is float:
                    data = 123.456789
                elif type(data) is int:
                    data = 123456789
                elif type(data) is bool:
                    data = True
                else:
                    raise RuntimeError(
                        f"Unsupported field type for {tel_name}.{name}: "
                        f"{type(data).__name__}"
                    )
                data_dict[name] = data

        try:
            t0margin = time.monotonic()
            while True:
                t0 = time.monotonic()
                for name, data_dict in self.telemetry_dict.items():
                    data = json.dumps(data_dict)
                    reply = replies.TelemetryReply(name=name, data=data)
                    await self.communicator.write(reply)
                t1 = time.monotonic()
                margin = self.telemetry_interval - (t1 - t0)
                self.telemetry_margins.append(margin)
                if margin > 0:
                    await asyncio.sleep(margin)
                if t1 - t0margin > 10:
                    dt = t1 - t0margin
                    t0margin = t1
                    margin_arr = np.array(self.telemetry_margins)
                    print(
                        f"margin min={margin_arr.min():0.3f}; "
                        f"max={margin_arr.max():0.3f}; "
                        f"mean={margin_arr.mean():0.3f}; "
                        f"std={margin_arr.std():0.3f}; "
                        f"rate={len(margin_arr)/dt:0.1f}"
                    )
                    self.telemetry_margins = []
        except Exception:
            self.log.exception("telemetry_loop failed")

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
            client_port=self.command_port + 1,
            server_host=salobj.LOCAL_HOST,
            server_port=self.command_port,
            log=self.log,
            read_replies=False,
            connect=False,
            connect_callback=self.connect_callback,
        )

        self.log.debug("start: connecting")
        await self.communicator.connect()
        self.log.debug("start: connected")
        self.read_loop_task = asyncio.create_task(self.read_loop())
        if self.telemetry_dict is not None:
            self.telemetry_task = asyncio.create_task(self.telemetry_loop())

    async def close(self):
        self.read_loop_task.cancel()
        self.telemetry_task.cancel()
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
            self.log.warning("Connection lost")
            await self.close()
            if self.reconnect:
                asyncio.create_task(self.connect())
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

    async def monitor_command(self, command, task):
        try:
            await task
            if self.communicator.connected:
                await self.write_done(command)
        except asyncio.CancelledError:
            if self.communicator.connected:
                await self.write_noack(command, explanation="Superseded")
        except Exception as e:
            if self.communicator.connected:
                await self.controller.write_noack(command, explanation=str(e))

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
