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

__all__ = ["BaseThermalDevice"]

from ..enums import ThermalMode
from .base_device import BaseDevice

# The amount by which to multiply temperature_slop to get the
# slop actually used. This allows unit tests to assume temperature
# will always be within BaseThermalDevice.temperature_slop of setpoint.
TEMPERATURE_SLOP_FACTOR = 0.9


class BaseThermalDevice(BaseDevice):
    """Mock the modbus cabinets thermal controller (which is always on).

    Limitations:

    * Ignores the "item" command parameter; it always controls all items.
    * Instantly adjusts the actual temperature to almost match the setpoint.

    Parameters
    ----------
    controller : `MockController`
        Mock controller.
    """

    # Error in temperature following (C).
    temperature_slop = 0.1

    def __init__(self, controller, system_id):
        self.ambient_setpoint = 0
        self.setpoint = 0
        self.temperature = TEMPERATURE_SLOP_FACTOR * self.temperature_slop
        # Notes:
        #
        # * Most thermal controllers only support TRACK_SETPOINT mode,
        #   but making them all have a mode simplifies the code.
        # * Starting them all in TRACK_SETPOINT mode simplifies unit tests.
        self.thermal_mode = ThermalMode.TRACK_SETPOINT
        super().__init__(controller=controller, system_id=system_id)

    @property
    def track_ambient(self):
        return self.thermal_mode == ThermalMode.TRACK_AMBIENT

    @property
    def track_setpoint(self):
        return self.thermal_mode == ThermalMode.TRACK_SETPOINT

    def set_ambient(self, setpoint):
        self.ambient = setpoint
        if self.power_on and self.track_ambient:
            self.temperature = (
                self.controller.ambient_temperature
                + TEMPERATURE_SLOP_FACTOR * self.temperature_slop
            )

    def set_setpoint(self, setpoint):
        if self.alarm_on:
            raise RuntimeError("In fault state")
        self.setpoint = setpoint
        if self.power_on and self.track_setpoint:
            self.temperature = (
                self.setpoint + TEMPERATURE_SLOP_FACTOR * self.temperature_slop
            )
