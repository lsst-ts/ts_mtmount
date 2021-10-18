# This file is part of ts_mtmount.
#
# Developed for the LSST Telescope and Site Systems.
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

import unittest

import jsonschema
import pytest

from lsst.ts import salobj
from lsst.ts import mtmount


class ValidationTestCase(unittest.TestCase):
    """Test validation of the config schema."""

    def setUp(self):
        self.schema = mtmount.CONFIG_SCHEMA
        self.validator = salobj.DefaultingValidator(schema=self.schema)
        self.default = dict(
            host="ccw-mgmt.cp.lsst.org",
            connection_timeout=10,
            ack_timeout=10,
            camera_cable_wrap_advance_time=0.02,
            camera_cable_wrap_interval=0.1,
            max_rotator_position_error=0.1,
        )
        self.nondefault = dict(
            host="1.2.3.4",
            connection_timeout=3.4,
            ack_timeout=4.5,
            camera_cable_wrap_advance_time=0.13,
            camera_cable_wrap_interval=0.4,
            max_rotator_position_error=1.5,
        )

    def test_default(self):
        result = self.validator.validate(None)
        for field, expected_value in self.default.items():
            assert result[field] == expected_value

    def test_some_specified(self):
        for field, value in self.nondefault.items():
            one_field_data = {field: value}
            with self.subTest(one_field_data=one_field_data):
                result = self.validator.validate(one_field_data)
                for field, default_value in self.default.items():
                    if field in one_field_data:
                        assert result[field] == one_field_data[field]
                    else:
                        assert result[field] == default_value

    def test_all_specified(self):
        result = self.validator.validate(self.nondefault)
        for field, value in self.nondefault.items():
            assert result[field] == value

    def test_invalid_configs(self):
        for name, badval in (
            #  ("host", "invalid hostname"),  # jsonschema 3.0.1 doesn't raise
            ("ack_timeout", "1"),  # wrong type
            ("ack_timeout", 0),  # not positive
            ("camera_cable_wrap_interval", -1),  # < 0
            ("connection_timeout", "1"),  # wrong type
            ("connection_timeout", 0),  # not positive
            ("host", 5),  # wrong type
            ("max_rotator_position_error", "1"),  # wrong type
            ("max_rotator_position_error", 0),  # not positive
        ):
            bad_data = {name: badval}
            with self.subTest(bad_data=bad_data):
                with pytest.raises(jsonschema.exceptions.ValidationError):
                    self.validator.validate(bad_data)


if __name__ == "__main__":
    unittest.main()
