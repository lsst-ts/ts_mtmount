# This file is part of ts_MTMount.
#
# Developed for the LSST Data Management System.
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

import asyncio

from .. import enums
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
    This mock does not implement that delay.

    There must be rules about which oil subsystem must be on or off
    in other to turn the others on or off. But I don't know the rules,
    so I have not tried to enfoce anything.
    """

    def __init__(self, controller):
        self.cooling_on = False
        self.main_pump_on = False
        self.oil_on = False
        super().__init__(
            controller=controller, device_id=enums.DeviceId.OIL_SUPPLY_SYSTEM
        )

    def do_power(self, command):
        super().do_power(command)
        self.cooling_on = command.on
        self.main_pump_on = command.on
        self.oil_on = command.on
        # The real system can take roughly 15 minutes
        # to bring the oil to an acceptable temperature.
        timeout = 15 * 60 if command.on else 1
        asyncio.create_task(self.controller.write_done(command))
        return timeout

    def do_power_cooling(self, command):
        self.cooling_on = command.on
        # The real system can take roughly 15 minutes
        # to bring the oil to an acceptable temperature.
        timeout = 15 * 60 if command.on else 1
        asyncio.create_task(self.controller.write_done(command))
        return timeout

    def do_power_main_pump(self, command):
        self.main_pump_on = command.on

    def do_power_oil(self, command):
        self.oil_on = command.on
