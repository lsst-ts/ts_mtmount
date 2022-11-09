# This file is part of ts_mtmount.
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

from lsst.ts import utils
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

    Here are the rules for when the four power commands are allowed:

    * OilSupplySystemPower (all subsystems):

        * Only accepted in auto mode.

    * OilSupplySystemPowerCooling:

        * Only accepted in manual mode.
        * Can only turn off if the circulation pump and main pump are both off.

    * OilSupplySystemPowerCirculationPump:

        * Only accepted in manual mode.
        * Can only turn on if cooling is already on.
        * Can only turn off if the main pump is off.

    * OilSupplySystemPowerMainPump:

        * Only accepted in manual mode.
        * Can only turn on if cooling and the circulation pump are both on.

    This class ignores the _power_on attribute;
    instead it keeps track with cooling_on, main_pump_on, circulation_pump_on.
    """

    def __init__(self, controller):
        self.cooling_on = False
        self.main_pump_on = False
        self.circulation_pump_on = False
        self.auto_mode = False
        super().__init__(controller=controller, system_id=System.OIL_SUPPLY_SYSTEM)

    @property
    def power_on(self):
        """Return True if all three subsystems are powered on."""
        return self.cooling_on and self.main_pump_on and self.circulation_pump_on

    @power_on.setter
    def power_on(self, on):
        self.cooling_on = on
        self.main_pump_on = on
        self.circulation_pump_on = on

    def do_power(self, command):
        if not self.auto_mode:
            raise RuntimeError("Must be in auto mode")
        super().do_power(command)
        # The real system can take roughly 15 minutes
        # to bring the oil to an acceptable temperature.
        timeout = 15 * 60 if command.on else 1
        return timeout, utils.make_done_future()

    def do_power_cooling(self, command):
        if self.auto_mode:
            raise RuntimeError("Must be in manual mode")
        if not command.on and (self.circulation_pump_on or self.main_pump_on):
            raise RuntimeError(
                "Cannot turn off cooling unless "
                f"the oil circulation pump={self.circulation_pump_on} "
                f"and main pump={self.main_pump_on} are both off."
            )

        self.cooling_on = command.on
        # The real system can take roughly 15 minutes
        # to bring the oil to an acceptable temperature.
        timeout = 15 * 60 if command.on else 1
        return timeout, utils.make_done_future()

    def do_power_circulation_pump(self, command):
        if self.auto_mode:
            raise RuntimeError("Must be in manual mode")
        if command.on and not self.cooling_on:
            raise RuntimeError(
                f"Cannot turn on the circulation pump if cooling={self.cooling_on} is off."
            )
        elif not command.on and self.main_pump_on:
            raise RuntimeError(
                f"Cannot turn off the circulation pump if the main pump={self.main_pump_on} is on."
            )
        self.circulation_pump_on = command.on

    def do_power_main_pump(self, command):
        if self.auto_mode:
            raise RuntimeError("Must be in manual mode")
        if command.on and not (self.cooling_on and self.circulation_pump_on):
            raise RuntimeError(
                f"Cannot turn on the main pump if cooling={self.cooling_on} is off "
                f"or the oil circulation pump={self.circulation_pump_on} is off."
            )
        self.main_pump_on = command.on

    def do_set_mode(self, command):
        self.auto_mode = command.auto
