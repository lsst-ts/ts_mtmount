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

__all__ = ["MirrorCoversDevice"]

from lsst.ts.idl.enums.MTMount import System

from .deployable_device import DeployableDevice


class MirrorCoversDevice(DeployableDevice):
    """Mirror covers.

    Parameters
    ----------
    controller : `MockController`
        Mock controller.
    """

    def __init__(self, controller):
        super().__init__(
            controller=controller,
            system_id=System.MIRROR_COVERS,
            deployed_position=0,
            retracted_position=100,
            start_deployed=True,
        )
