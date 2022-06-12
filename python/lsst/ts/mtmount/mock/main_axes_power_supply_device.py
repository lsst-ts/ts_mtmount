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

__all__ = ["MainAxesPowerSupplyDevice"]


from lsst.ts import utils
from lsst.ts.idl.enums.MTMount import System
from .base_device import BaseDevice


class MainAxesPowerSupplyDevice(BaseDevice):
    """Main power supply.

    If this turns off and azimuth or elevation is on,
    stop that device and send it to fault state.

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
        if not command.on:
            for system_id in (
                System.AZIMUTH,
                System.ELEVATION,
            ):
                axis_device = self.controller.device_dict[system_id]
                if axis_device.power_on:
                    axis_device.stop_motion(command=command, gently=False)
                    self.alarm_on = True
                    self.power_on = False
                    self.enabled = False

        # The real system takes about 2 minutes to turn on.
        timeout = 120 if command.on else 0
        return timeout, utils.make_done_future()
