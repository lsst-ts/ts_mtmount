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

__all__ = ["MAX_TRACKING_DELAY", "AxisDevice"]

import asyncio

from lsst.ts import salobj
from lsst.ts import simactuators
from .. import enums
from .. import limits
from ..exceptions import CommandSupersededException
from .base_device import BaseDevice


# Maximum time (seconds) after the end time of one track command
# before another must arrive. In other words, the maximum amount
# of time the axis is willing to extrapolate a track command.
MAX_TRACKING_DELAY = 1


class AxisDevice(BaseDevice):
    """Mock axis controller device.

    Suports all commands except MOVE_VELOCITY.

    Parameters
    ----------
    controller : `MockController`
        Mock controller.
    device_id : `DeviceId`
        Device ID. Must be one of:

        * enums.DeviceId.AZIMUTH_AXIS
        * enums.DeviceId.ELEVATION_AXIS
        * enums.DeviceId.CAMERA_CABLE_WRAP
    start_position : `float`, optional
        Initial position (deg)

    Notes
    -----
    There is no mock azimuth cable wrap because in the real system
    the azimuth axis takes care of that cable wrap, so `MTMountCsc`
    has no need to send commands to it.

    Turning on the device also enables it.
    """

    def __init__(self, controller, device_id, start_position=0):
        device_id = enums.DeviceId(device_id)
        device_limits = limits.LimitsDict[device_id].scaled()
        self.enabled = False
        self.tracking_enabled = False
        self.tracking_paused = False
        # Has a target position been specified?
        # Set True when tracking or moving point to point and False otherwise.
        self.has_target = False
        self.actuator = simactuators.TrackingActuator(
            min_position=device_limits.min_position,
            max_position=device_limits.max_position,
            max_velocity=device_limits.max_velocity,
            max_acceleration=device_limits.max_acceleration,
            dtmax_track=0.2,
            start_position=start_position,
        )
        self._monitor_move_task = asyncio.Future()
        self._monitor_move_task.set_result(None)
        self._move_result_task = asyncio.Future()
        self._move_result_task.set_result(None)
        self._tracking_timeout_task = asyncio.Future()
        self._tracking_timeout_task.set_result(None)
        super().__init__(controller=controller, device_id=device_id)

    @property
    def end_tai(self):
        """Get the end time of the current path, as TAI unix seconds.
        """
        return self.actuator.path[-1].tai

    def assert_enabled(self):
        """Raise `RuntimeError` if device is not fully enabled.

        To be fully enabled, the device must be on, no alarm present,
        and the drive enabled.
        """
        self.assert_on()
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
        self._move_result_task.cancel()
        self._tracking_timeout_task.cancel()

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
        self._monitor_move_task.cancel()
        self._move_result_task.cancel()
        self._move_result_task = asyncio.Future()
        self._monitor_move_task = asyncio.create_task(self._monitor_move(command))
        return self._move_result_task

    async def _monitor_move(self, command):
        """Do most of the work for monitor_move_command.
        """
        # Provide some slop for non-monotonic clocks, which are
        # sometimes seen when running Docker on macOS.
        duration = 0.2 + self.end_tai - salobj.current_tai()
        await asyncio.sleep(duration)
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

    def do_drive_enable(self, command):
        """Enable or disable the drive.

        Abort motion and disable tracking.
        """
        self._stop_motion(command=command, gently=False)
        self.enabled = command.on

    def do_drive_reset(self, command):
        """Reset the drive.

        The power must be on.

        Abort motion, disable tracking, disable the drive.
        """
        self.assert_on()
        self._stop_motion(command=command, gently=False)
        self.enabled = False

    def do_enable_tracking(self, command):
        """Enable or pause tracking mode.

        The drive must be enabled.
        """
        self.assert_enabled()
        self.supersede_move_command(command)
        self.actuator.stop()
        self.has_target = False
        # Camera cable wrap enable tracking has an "on" parameter:
        # on=1 means enable tracking, on=0 means pause tracking.
        # Azimuth and elevation enable tracking commands have no parameters;
        # they always enable tracking.
        on_value = getattr(command, "on", True)
        if on_value:
            # Enable tracking.
            self.tracking_enabled = True
            self.tracking_paused = False
        else:
            # Pause tracking (this must be the camera cable wrap).
            if not self.tracking_enabled:
                raise RuntimeError(
                    "Tracking cannot be paused because tracking is not enabled"
                )
            self._tracking_timeout_task.cancel()
            self.tracking_paused = True

    def do_home(self, command):
        """Home the actuator.

        The drive must be enabled and tracking must be disabled.

        For sake of doing something vaguely plausible,
        move to the mid-point of the position limits.
        """
        self.assert_enabled()
        position = (self.actuator.min_position + self.actuator.max_position) / 2
        return self.move_point_to_point(position=position, command=command)

    def do_move(self, command):
        """Set target position.

        The drive must be enabled and tracking must be disabled.

        Velocity and acceleration limits are not applied,
        because it is tricky to make sure the values are
        correctly restored when the move finishes,
        and the CSC doesn't support these parameters anyway.
        """
        self.assert_enabled()
        self.has_target = True
        return self.move_point_to_point(position=command.position, command=command)

    def do_move_velocity(self, command):
        raise NotImplementedError("Not implemented")

    def do_power(self, command):
        if command.on == self.power_on and self.enabled == self.power_on:
            # Nothing to do; don't stop existing motion
            return
        self._stop_motion(command=command, gently=False)
        super().do_power(command)
        self.enabled = command.on

    def do_stop(self, command):
        """Stop the actuator.
        """
        self.assert_enabled()
        self._stop_motion(command=command, gently=True)

    def do_track(self, command):
        """Specify a tracking target position, velocity, and time.

        The drive must be enabled and tracking must be enabled.
        """
        self.assert_enabled()
        self.assert_tracking_enabled(True)
        self.supersede_move_command(command)
        if self.tracking_paused:
            self._tracking_timeout_task.cancel()
            return
        self.start_tracking_timer(command)
        self.actuator.set_target(
            tai=command.tai, position=command.position, velocity=command.velocity
        )
        self.has_target = True

    def move_point_to_point(self, position, command):
        """Move to the specified position.

        The drive must be enabled and tracking must be disabled.

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
        self.supersede_move_command(command)
        tai = salobj.current_tai()
        self.actuator.set_target(tai=tai, position=position, velocity=0)
        self.has_target = True
        timeout = self.end_tai - tai
        task = self.monitor_move_command(command)
        return timeout, task

    def start_tracking_timer(self, command):
        """Start or restart the tracking timer.

        Call this whenever a tracking command is received
        (if tracking is not paused).
        """
        self._tracking_timeout_task.cancel()
        duration = MAX_TRACKING_DELAY + command.tai - salobj.current_tai()
        if duration <= 0:
            raise RuntimeError(f"track command too late by {-duration:0.2} seconds")
        self._tracking_timeout_task = asyncio.create_task(
            self._tracking_timer(duration)
        )

    async def _tracking_timer(self, duration):
        """Wait for the specified duration (seconds) and kill tracking.

        start_tracking_timer re-starts this for every track command
        (while tracking is not paused).
        """
        await asyncio.sleep(duration)
        self.log.error("Tracking timed out")
        self.alarm_on = True
        self.power_on = False
        self.enabled = False
        self._stop_motion(command=None, gently=False)

    def _stop_motion(self, command, gently):
        """Stop motion, if any, and clear tracking_x and has_target flags.

        Parameters
        ----------
        command : `Command` or `None`
            New command, if any.
            Used to mark the existing motion command (if any) as superseded.
        gently : `bool`
            If True then stop motion gently, else abruptly.
        """
        self.has_target = False
        self.tracking_enabled = False
        self.tracking_paused = False
        self._tracking_timeout_task.cancel()
        if command is not None:
            self.supersede_move_command(command)
        if gently:
            self.actuator.stop()
        else:
            self.actuator.abort()
