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

__all__ = ["Limits", "LimitsDict"]

from . import enums


class Limits:
    """Motion limits for an axis controller.

    Parameters
    ----------
    min_position : `float`
        Minimum position (deg)
    max_position : `float`
        Maximum position (deg)
    max_velocity : `float`
        Maximum absolute value of velocity (deg/sec)
    max_acceleration : `float`
        Maximum absolute value of acceleration (deg/sec/sec)
    """

    def __init__(self, min_position, max_position, max_velocity, max_acceleration):
        self.min_position = min_position
        self.max_position = max_position
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration

    def scaled(self, factor=1.01):
        """Return a copy scaled by the specified factor.

        Intended for use in mock axis controllers, because the actuator
        soft limits should be slighly larger than the limits
        allowed by the CSC.
        """
        return Limits(
            min_position=self.min_position * factor,
            max_position=self.max_position * factor,
            max_velocity=self.max_velocity * factor,
            max_acceleration=self.max_acceleration * factor,
        )


LimitsDict = {
    # From LTS-103.
    enums.DeviceId.ELEVATION_AXIS: Limits(
        min_position=20, max_position=86.5, max_velocity=3.5, max_acceleration=3.5
    ),
    # From LTS-103.
    enums.DeviceId.AZIMUTH_AXIS: Limits(
        min_position=-270, max_position=270, max_velocity=7.0, max_acceleration=7.0
    ),
    # From LTS-218.
    enums.DeviceId.CAMERA_CABLE_WRAP: Limits(
        min_position=-90, max_position=90, max_velocity=3.5, max_acceleration=1
    ),
}
