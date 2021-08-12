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

__all__ = ["OilSupplySystemDevice"]

from lsst.ts import salobj
from lsst.ts.idl.enums.MTMount import System
from .base_device import BaseDevice


class OilSupplySystemDevice(BaseDevice):
    """Oil supply system supply.

    Parameters
    ----------
    controller : `MockController`
        Mock controller.

    Notes
    -----
    The real oil supply system can take 1/4 hour to turn on,
    as the oil is brought to the correct temperature.
    This mock uses no actual delay, to simplify the code,
    but it reports semi-normal timeouts.

    There must be rules about which oil subsystem must be on or off
    in other to turn the others on or off. But I don't know the rules,
    so I have not tried to enfoce anything.

    This class ignores the _power_on attribute;
    instead it keeps track with cooling_on, main_pump_on, oil_on.
    """

    def __init__(self, controller):
        self.cooling_on = False
        self.main_pump_on = False
        self.oil_on = False
        super().__init__(controller=controller, system_id=System.OIL_SUPPLY_SYSTEM)

    @property
    def power_on(self):
        """Return True if all three subsystems are powered on."""
        return self.cooling_on and self.main_pump_on and self.oil_on

    @power_on.setter
    def power_on(self, on):
        self.cooling_on = on
        self.main_pump_on = on
        self.oil_on = on

    def do_power(self, command):
        super().do_power(command)
        # The real system can take roughly 15 minutes
        # to bring the oil to an acceptable temperature.
        timeout = 15 * 60 if command.on else 1
        return timeout, salobj.make_done_future()

    def do_power_cooling(self, command):
        self.cooling_on = command.on
        # The real system can take roughly 15 minutes
        # to bring the oil to an acceptable temperature.
        timeout = 15 * 60 if command.on else 1
        return timeout, salobj.make_done_future()

    def do_power_main_pump(self, command):
        self.main_pump_on = command.on

    def do_power_oil(self, command):
        self.oil_on = command.on
