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

__all__ = ["MirrorCoverLocksDevice"]

from .. import enums
from .point_to_point_device import PointToPointDevice


class MirrorCoverLocksDevice(PointToPointDevice):
    """Mirror cover locks.

    Supports all commands except MOVE and MOVE_VELOCITY.

    Unlike the real system, the drive argument must always be -1
    (meaning all drives). That suffices for the CSC.

    Parameters
    ----------
    controller : `MockController`
        Mock controller.
    """

    def __init__(self, controller):
        super().__init__(
            controller=controller,
            device_id=enums.DeviceId.MIRROR_COVER_LOCKS,
            min_position=0,
            max_position=100,
            start_position=0,
            speed=100,
            multi_drive=True,
        )

    def do_move(self, command):
        raise NotImplementedError("Not implemented")

    def do_move_velocity(self, command):
        raise NotImplementedError("Not implemented")

    def do_move_all(self, command):
        position = 100 if command.deploy else 0
        return self.move(position=position, command=command)

    def do_stop(self, command):
        """Stop the actuator."""
        self.supersede_move_command(command)
        self.actuator.stop()
