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

__all__ = ["AuxiliaryCabinetsThermalDevice"]

from lsst.ts.xml.enums.MTMount import System

from .base_thermal_device import BaseThermalDevice


class AuxiliaryCabinetsThermalDevice(BaseThermalDevice):
    """Mock the auxiliary cabinets thermal controller (which is always on).

    Limitations:

    * Ignores the "item" command parameter; it always controls all items.
    * Instantly adjusts the actual temperature to almost match the setpoint.

    Parameters
    ----------
    controller : `MockController`
        Mock controller.
    """

    def __init__(self, controller):
        self.fans_on = False
        super().__init__(
            controller=controller,
            system_id=System.AUXILIARY_CABINETS_THERMAL,
        )

    @property
    def power_on(self):
        # This device is always on unless in alarm state.
        return not self.alarm_on

    @power_on.setter
    def power_on(self, on):
        raise RuntimeError("This device is always on, unless in fault.")

    def do_fan_power(self, command):
        if self.alarm_on:
            raise RuntimeError("In fault state")
        self.fans_on = command.on

    def do_setpoint(self, command):
        self.set_setpoint(command.setpoint)
