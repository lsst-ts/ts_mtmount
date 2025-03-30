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

__all__ = ["MainCabinetThermalDevice"]

from lsst.ts.xml.enums.MTMount import System

from ..enums import ThermalMode
from .base_thermal_device import BaseThermalDevice


class MainCabinetThermalDevice(BaseThermalDevice):
    """Mock main cabinet thermal controller.

    Limitations:

    * Instantly adjusts the actual temperature to almost match the setpoint.

    Parameters
    ----------
    controller : `MockController`
        Mock controller.
    """

    def __init__(self, controller):
        super().__init__(controller=controller, system_id=System.MAIN_CABINET_THERMAL)

    @property
    def power_on(self):
        # Always on, unless in fault.
        return not self.alarm_on

    @power_on.setter
    def power_on(self, on):
        raise RuntimeError("This device is always on, unless in fault.")

    def do_set_temperature(self, command):
        raise NotImplementedError("Use TRACK_TEMPERATURE instead")

    def do_track_ambient(self, command):
        self.assert_on()
        self.thermal_mode = (
            ThermalMode.TRACK_AMBIENT
            if command.track_ambient
            else ThermalMode.TRACK_SETPOINT
        )
        if command.track_ambient:
            self.set_ambient(command.setpoint)
        else:
            self.set_setpoint(command.setpoint)
