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

import random
import unittest

from lsst.ts import mtmount

random.seed(314)


class CommandTestCase(unittest.TestCase):
    def setUp(self):
        self.name = "otho"
        self.doc = f"a doc string for the field named {self.name}"

    def check_round_trip(self, command):
        """Check that Command.str_fields and <command_type>.from_str_fields
        are inverses.

        Convert a command to str fields, then back to a command
        and check that it matches the original.
        """
        str_fields = command.str_fields()
        command_round_trip = type(command).from_str_fields(str_fields)
        assert command == command_round_trip

    def check_command_type(self, command_type):
        """For a given command type: make several random commands and call
        check_round_trip.
        """
        for i in range(10):
            command = mtmount.testutils.make_random_command(command_type)
            self.check_round_trip(command)

            command2 = mtmount.testutils.make_random_command_with_defaults(command_type)
            self.check_round_trip(command2)

    def test_commands(self):
        for command_type in mtmount.commands.Commands:
            with self.subTest(command_type=command_type.__name__):
                self.check_command_type(command_type)
