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

__all__ = ["MirrorCoversDevice"]

import asyncio

from lsst.ts import simactuators
from .. import enums
from . import base_device


class MirrorCoversDevice(base_device.BaseDevice):
    """Mirror covers.

    Supports all commands except MOVE and MOVE_VELOCITY.

    Unlike the real system, the drive argument must always be -1
    (meaning all drives). That suffices for the CSC.

    Parameters
    ----------
    controller : `MockController`
        Mock controller.
    device_id : `DeviceId`
        Device ID
    """

    def __init__(self, controller, device_id):
        device_id = enums.DeviceId(device_id)
        if device_id is not enums.DeviceId.MIRROR_COVERS:
            raise ValueError(f"device_id={device_id!r} must be MIRROR_COVERS")
        self.actuator = simactuators.PointToPointActuator(
            min_position=0, max_position=100, start_position=0, speed=50,
        )
        self._current_move_command = None
        self._monitor_move_task = asyncio.Future()
        super().__init__(controller=controller, device_id=device_id)

    def assert_drive_all(self, command):
        """Assert that the drive argument is -1 (all drives).
        """
        if command.drive != -1:
            raise RuntimeError(
                f"drive={command.drive}; must be -1 (all actuators) for this mock"
            )

    def assert_tracking_enabled(self, enabled):
        """Raise `RuntimeError` if tracking is or is not enabled.
        """
        if enabled:
            if not self.tracking_enabled:
                raise RuntimeError("Tracking not enabled")
        else:
            if self.tracking_enabled:
                raise RuntimeError("Tracking not disabled")

    async def close(self):
        await super().close()
        self._monitor_move_task.cancel()

    def monitor_move_command(self, command):
        """Monitor motion and report a move command as done.
        """
        self._monitor_move_task.cancel()
        self._monitor_move_task = asyncio.create_task(self._monitor_move(command))

    async def _monitor_move(self, command):
        """Do most of the work for monitor_move_command.
        """
        self._current_move_command = command
        try:
            await asyncio.sleep(self.actuator.remaining_time)
            await self.controller.write_done(self._current_move_command)
        except asyncio.CancelledError:
            await self.controller.write_noack(
                self._current_move_command, explanation="Superseded"
            )
        except Exception as e:
            await self.controller.write_noack(
                self._current_move_command, explanation=str(e)
            )
        finally:
            self._current_move_command = None

    def supersede_move_command(self):
        """Report the current move command (if any) as superseded.
        """
        self._monitor_move_task.cancel()

    def do_move(self, command):
        raise NotImplementedError("Not implemented")

    def do_move_velocity(self, command):
        raise NotImplementedError("Not implemented")

    def do_close(self, command):
        return self._move(position=0, command=command)

    def do_open(self, command):
        return self._move(position=100, command=command)

    def do_stop(self, command):
        """Stop the actuator.
        """
        self.supersede_move_command()
        self.actuator.stop()

    def _move(self, position, command):
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
        self.assert_drive_all(command)
        self.supersede_move_command()
        self.actuator.set_position(position)
        self.monitor_move_command(command)
        return self.actuator.remaining_time
