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

__all__ = ["MainAxesPowerSupplyDevice"]


from lsst.ts import salobj
from lsst.ts.idl.enums.MTMount import System
from .base_device import BaseDevice


class MainAxesPowerSupplyDevice(BaseDevice):
    """Main power supply.

    Parameters
    ----------
    controller : `MockController`
        Mock controller.

    Notes
    -----
    The real main power supply takes about 2 minutes to turn on.
    This mock is fast, to avoid needlessly slowing down unit tests.
    """

    def __init__(self, controller):
        super().__init__(controller=controller, system_id=System.MAIN_AXES_POWER_SUPPLY)

    def do_power(self, command):
        super().do_power(command)
        # The real system takes about 2 minutes to turn on.
        timeout = 120 if command.on else 0
        return timeout, salobj.make_done_future()
