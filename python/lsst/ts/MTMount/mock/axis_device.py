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

__all__ = ["AxisDevice"]

import asyncio

from lsst.ts import salobj
from lsst.ts import simactuators
from .. import enums
from .. import limits
from . import base_device


class AxisDevice(base_device.BaseDevice):
    """Mock axis controller device.

    Suports all commands except MOVE_VELOCITY.

    Parameters
    ----------
    controller : `MockController`
        Mock controller.
    device_id : `DeviceId`
        Device ID
    """

    def __init__(self, controller, device_id):
        device_id = enums.DeviceId(device_id)
        device_limits = limits.LimitsDict[device_id]
        device_limits.scale(factor=1.01)
        self.enabled = False
        self.tracking_enabled = False
        self.actuator = simactuators.TrackingActuator(
            min_position=device_limits.min_position,
            max_position=device_limits.max_position,
            max_velocity=device_limits.max_velocity,
            max_acceleration=device_limits.max_acceleration,
            dtmax_track=0.2,
        )
        self._current_move_command = None
        self._monitor_move_task = asyncio.Future()
        super().__init__(controller=controller, device_id=device_id)

    @property
    def end_tai_unix(self):
        """Get the end time of the current path, as TAI unix seconds.
        """
        return self.actuator.path[-1].tai

    def assert_enabled(self):
        """Raise `RuntimeError` if the drive is not enabled.
        """
        if not self.enabled:
            raise RuntimeError("Not enabled")

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
            # Give a little margin to make sure the actuator is truly done
            duration = 0.1 + self.end_tai_unix - salobj.current_tai()
            await asyncio.sleep(duration)
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

    def do_drive_enable(self, command):
        """Enable or disable the drive.
        """
        self.supersede_move_command()
        self.enabled = command.on

    def do_drive_reset(self, command):
        """Reset the drive.
        """
        self.supersede_move_command()
        self.actuator.abort()
        self.enabled = False

    def do_enable_tracking(self, command):
        """Enable or disable tracking mode.
        """
        self.supersede_move_command()
        self.actuator.abort()
        self.tracking_enabled = command.on

    def do_home(self, command):
        """Home the actuator.

        For sake of doing something vaguely plausible,
        move to the mid-point of the position limits.
        """
        position = (self.limits.min_position + self.limits.max_position) / 2
        return self._move_point_to_point(position=position, command=command)

    def do_move(self, command):
        """Set target position.

        Velocity and acceleration limits are not applied,
        because it is tricky to make sure the values are
        correctly restored when the move finishes,
        and the CSC doesn't support these parameters anyway.
        """

        return self._move_point_to_point(position=command.position, command=command)

    def do_move_velocity(self, command):
        raise NotImplementedError("Not implemented")

    def do_stop(self, command):
        """Stop the actuator.
        """
        self.supersede_move_command()
        self.tracking_enabled = False
        self.actuator.stop()

    def do_track(self, command):
        """Specify a tracking target tai_time, position, velocity.
        """
        self.assert_enabled()
        self.assert_tracking_enabled(True)
        self.supersede_move_command()
        # Despite the name, tai_from_utc works with any astropy.time.Time
        tai_unix = salobj.tai_from_utc(command.tai_time, format=None)
        self.actuator.set_target(
            tai=tai_unix, position=command.position, velocity=command.velocity
        )

    def _move_point_to_point(self, position, command):
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
        self.assert_enabled()
        self.assert_tracking_enabled(False)
        self.supersede_move_command()
        # Despite the name, tai_from_utc works with any astropy.time.Time
        tai_unix = salobj.current_tai()
        self.actuator.set_target(tai=tai_unix, position=position, velocity=0)
        self.monitor_move_command(command)
        return self.end_tai_unix - tai_unix
