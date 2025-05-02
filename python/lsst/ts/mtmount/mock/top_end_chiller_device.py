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

__all__ = ["TopEndChillerDevice"]

from lsst.ts.xml.enums.MTMount import System

from ..enums import ThermalMode
from .base_thermal_device import BaseThermalDevice


class TopEndChillerDevice(BaseThermalDevice):
    """Top end chiller.

    This is a guess, since the command set and initial state are unknown.

    Parameters
    ----------
    controller : `MockController`
        Mock controller.
    """

    def __init__(self, controller):
        super().__init__(controller=controller, system_id=System.TOP_END_CHILLER)

    def do_track_ambient(self, command):
        self.assert_on()
        self.thermal_mode = (
            ThermalMode.TRACK_AMBIENT
            if command.track_ambient
            else ThermalMode.TRACK_SETPOINT
        )
        if command.track_ambient:
            self.set_ambient(setpoint=self.controller.ambient_temperature)
        else:
            self.set_setpoint(setpoint=command.setpoint)
