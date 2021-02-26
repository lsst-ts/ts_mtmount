# This file is part of ts_MTMount.
#
# Developed for Vera Rubin Observatory.
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

__all__ = ["PointToPointDevice"]

import asyncio

from lsst.ts import simactuators
from ..exceptions import CommandSupersededException
from .base_device import BaseDevice


class PointToPointDevice(BaseDevice):
    """Base class for devices have a single point to point actuator.

    This also works for multi-drive devices where we always
    move all drives at the same time, for example mirror covers.

    Subclass must add do_ methods for motion commands,
    since every device is different.

    Parameters
    ----------
    controller : `MockController`
        Mock controller.
    device_id : `DeviceId`
        Device ID
    min_position : `float`
        Minimum position.
    max_position : `float`
        Maximum position.
    start_position : `float`
        Initial position.
    speed : `float`
        Speed of motion (acceleration is assumed infinite).
    multi_drive : `bool`
        Does this system have multiple drives (being treated as one)?
        If True then `move` calls `assert_drive_all`.
    """

    def __init__(
        self,
        controller,
        device_id,
        min_position,
        max_position,
        start_position,
        speed,
        multi_drive,
    ):
        self.actuator = simactuators.PointToPointActuator(
            min_position=min_position,
            max_position=max_position,
            start_position=start_position,
            speed=speed,
        )
        self.multi_drive = multi_drive
        self._move_result_task = asyncio.Future()
        self._move_result_task.set_result(None)
        self._monitor_move_task = asyncio.Future()
        self._monitor_move_task.set_result(None)
        super().__init__(controller=controller, device_id=device_id)

    def assert_drive_all(self, command):
        """Assert that the drive argument is -1 (all drives).
        """
        if command.drive != -1:
            raise RuntimeError(
                f"drive={command.drive}; must be -1 (all actuators) for this mock"
            )

    async def close(self):
        await super().close()
        self._move_result_task.cancel()
        self._monitor_move_task.cancel()

    def monitor_move_command(self, command):
        """Return a task that is set done when the move is done.

        Parameters
        ----------
        command : `Command`
            Move command.

        Returns
        -------
        task : `asyncio.Task`
            A task that is set done when the move is done.
        """
        self._move_result_task.cancel()
        self._monitor_move_task.cancel()
        self._move_result_task = asyncio.Future()
        self._monitor_move_task = asyncio.create_task(self._monitor_move(command))
        return self._move_result_task

    async def _monitor_move(self, command):
        """Do most of the work for monitor_move_command.
        """
        # Provide some slop for non-monotonic clocks, which are
        # sometimes seen when running Docker on macOS.
        await asyncio.sleep(self.actuator.remaining_time() + 0.2)
        if not self._move_result_task.done():
            self._move_result_task.set_result(None)

    def supersede_move_command(self, command):
        """Report the current move command (if any) as superseded.
        """
        if not self._move_result_task.done():
            self._move_result_task.set_exception(
                CommandSupersededException(command=command)
            )
        self._monitor_move_task.cancel()

    def do_move(self, command):
        raise NotImplementedError("Not implemented")

    def do_move_velocity(self, command):
        raise NotImplementedError("Not implemented")

    def do_close(self, command):
        return self.move(position=0, command=command)

    def do_open(self, command):
        return self.move(position=100, command=command)

    def do_stop(self, command):
        """Stop the actuator.
        """
        self.supersede_move_command(command)
        self.actuator.stop()

    def move(self, position, command):
        """Move to the specified position.

        Parameters
        ----------
        position : `float`
            Desired position, in degrees.
        command : `Command`
            Command to monitor and report done.

        Returns
        -------
        duration : `float`
           Minimum duration of move (sec)
        """
        if not self.power_on:
            raise RuntimeError("Device not powered on.")
        if self.multi_drive:
            self.assert_drive_all(command)
        self.supersede_move_command(command)
        timeout = self.actuator.set_position(position)
        task = self.monitor_move_command(command)
        return timeout, task
