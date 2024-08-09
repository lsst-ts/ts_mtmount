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
from lsst.ts import mtmount, salobj


class ValidationTestCase(unittest.TestCase):
    """Test validation of the config schema."""

    def setUp(self):
        self.schema = mtmount.CONFIG_SCHEMA
        self.validator = salobj.StandardValidator(schema=self.schema)
        self.default = dict(
            host="ccw-mgmt.cp.lsst.org",
            telemetry_host="ccw-mgmt.cp.lsst.org",
            connection_timeout=10,
            ack_timeout=10,
            camera_cable_wrap_advance_time=0.02,
            camera_cable_wrap_interval=0.1,
            max_rotator_position_error=0.1,
            park_settings=[
                "ParkSettings",
            ],
            park_positions=dict(
                zenith=dict(
                    elevation=90.01,
                    azimuth=0.0,
                ),
                horizon=dict(
                    elevation=0.0,
                    azimuth=0.0,
                ),
            ),
        )
        self.nondefault = dict(
            host="1.2.3.4",
            telemetry_host="5.6.7.8",
            connection_timeout=3.4,
            ack_timeout=4.5,
            camera_cable_wrap_advance_time=0.13,
            camera_cable_wrap_interval=0.4,
            max_rotator_position_error=1.5,
            park_settings=[
                "ParkSettings",
            ],
            park_positions=dict(
                zenith=dict(
                    elevation=90.01,
                    azimuth=0.0,
                ),
                horizon=dict(
                    elevation=0.0,
                    azimuth=0.0,
                ),
            ),
        )

    def test_all_specified(self):
        self.validator.validate(self.default)
        self.validator.validate(self.nondefault)

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
