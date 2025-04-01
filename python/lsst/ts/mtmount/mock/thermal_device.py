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

__all__ = ["ThermalDevice"]

from lsst.ts.xml.enums.MTMount import System

from ..enums import ThermalMode
from .base_thermal_device import BaseThermalDevice


class ThermalDevice(BaseThermalDevice):
    """Mock several thermal controllers: azimuth/elevation drives,
    and cabinet 0101.

    Limitations:

    * Ignores the "item" command parameter; it always controls all items.
    * Instantly adjusts the actual temperature to almost match the setpoint.
    * Only supports two modes: TRACK_AMBIENT and TRACK_SETPOINT

    Parameters
    ----------
    controller : `MockController`
        Mock controller.
    """

    supported_system_ids = frozenset(
        (
            System.AZIMUTH_DRIVES_THERMAL,
            System.ELEVATION_DRIVES_THERMAL,
            System.CABINET_0101_THERMAL,
        )
    )

    def __init__(self, controller, system_id):
        if system_id not in self.supported_system_ids:
            raise ValueError(f"{system_id=} not in {self.supported_system_ids}")
        super().__init__(controller=controller, system_id=system_id)

    def do_control_mode(self, command):
        """Set the thermal control mode."""
        if self.alarm_on:
            raise RuntimeError("In fault state")
        if not self.power_on:
            raise RuntimeError("Device not powered on.")
        thermal_mode = ThermalMode(command.mode)
        if thermal_mode not in {ThermalMode.TRACK_SETPOINT, ThermalMode.TRACK_AMBIENT}:
            raise RuntimeError(
                f"mode={thermal_mode!r}; the only modes supported by this mock "
                "are TRACK_SETPOINT and TRACK_AMBIENT"
            )
        self.thermal_mode = thermal_mode
        if self.track_setpoint:
            self.set_setpoint(command.setpoint)
        else:
            self.set_ambient(self.controller.ambient_temperature)
