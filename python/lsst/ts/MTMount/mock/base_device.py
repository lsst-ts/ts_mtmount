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

__all__ = ["BaseDevice"]

from .. import enums
from .. import commands


class BaseDevice:
    """Mock device that supports the POWER and RESET_ALARM commands.

    Parameters
    ----------
    controller : `MockController`
        Mock controller.
    device_id : `DeviceId`
        Device ID.

    Notes
    -----
    Each kind of device must define a (synchronous) do_command method
    for each supported low-level command and obey these rules:

    The "command" part of the do_command method name must match a
    `CommandCode` name, but lowercase and with the device prefix omitted.
    For example, for a mock device with device_id = `DeviceId.ELEVATION_AXIS`,
    the method ``do_reset_error`` implements the
    `CommandCode.ELEVATION_AXIS_RESET_ERROR` command.

    The do_command method must accept one argument: a `Command`.
    This may be assumed to be the correct command.

    If the command runs quickly then the do_command method should return
    `None` and the caller will write `AckReply` and (if the command
    is supposed to receive one) `DoneReply` for the command.
    If the command takes awhile to run then the do_command method
    must return a timeout (in seconds) and must itself report
    the final `DoneReply` when the command succeeds or `NoAckReply`
    if the command fails or is superseded or canceled after do_command returns.
    If the do_command method raises an exception the the caller will write
    `NoAckReply` for the command, to report the command as failed.
    """

    all_command_names = {code.name for code in enums.CommandCode}

    def __init__(self, controller, device_id):
        self.controller = controller
        self.device_id = enums.DeviceId(device_id)
        self._device_prefix = self.device_id.name
        self.log = controller.log.getChild(self._device_prefix)

        self.power_on = False
        self.alarm_on = False

        self.add_methods()

    def add_methods(self):
        """Add do_methods to the command dict
        """
        prefix = f"{self._device_prefix}_"
        prefix_len = len(prefix)
        # Iterate over commands that this CSC supports,
        # which is probably not all commands in `CommandCode`.
        for command_code in commands.CommandDict:
            if command_code.name.startswith(prefix):
                command_name_lower = command_code.name[prefix_len:].lower()
                do_method = getattr(self, f"do_{command_name_lower}")
                if command_code in self.controller.command_dict:
                    raise RuntimeError(
                        f"Command {command_code.name} already in command_dict; "
                        "have you added this same mock device twice?"
                    )
                self.controller.command_dict[command_code] = do_method
                self.log.debug(f"Add {command_code.name} command to command_dict")

    def do_power(self, command):
        self.power_on = command.on

    def do_reset_alarm(self, command):
        self.alarm_on = False