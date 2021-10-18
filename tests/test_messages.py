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


class MessageTestCase(unittest.TestCase):
    def setUp(self):
        self.name = "otho"
        self.doc = f"a doc string for the field named {self.name}"

    def check_round_trip(self, message):
        str_fields = message.str_fields()
        message_round_trip = type(message).from_str_fields(str_fields)
        assert message == message_round_trip

    def check_message_type(self, message_type):
        for i in range(10):
            message = mtmount.testutils.make_random_message(message_type)
            self.check_round_trip(message)

            message2 = mtmount.testutils.make_random_message_with_defaults(message_type)
            self.check_round_trip(message2)

    def test_commands(self):
        for command_type in mtmount.commands.Commands:
            with self.subTest(command_type=command_type.__name__):
                self.check_message_type(command_type)

    def test_replies(self):
        for reply_type in mtmount.replies.Replies:
            with self.subTest(reply_type=reply_type.__name__):
                self.check_message_type(reply_type)


if __name__ == "__main__":
    unittest.main()
