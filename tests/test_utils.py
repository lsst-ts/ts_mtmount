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

import itertools
import unittest

import pytest
from lsst.ts import mtmount
from lsst.ts.mtmount.utils import MAX_DOC_LENGTH


class UtilsTestCase(unittest.TestCase):
    """Test the utils subpackage."""

    def test_wrap_parameter_doc(self):
        # The following text was chosen so when duplicated and wrapped
        # some of the text reaches 79 columns.
        long_text = (
            "This is a long line of text that will reach column 79 when wrapped. " * 50
        )
        wrapped_text = mtmount.wrap_parameter_doc(long_text)
        lines = wrapped_text.split("\n")
        for line in lines:
            assert line.startswith("    ")
            assert len(line) <= MAX_DOC_LENGTH

    def test_truncate_value(self):
        descr = "an arbitrary description"
        limits = [-5, -3.2, -0.001, 0, 0.001, 42]
        for min_value, max_value in itertools.product(limits, limits):
            if min_value >= max_value:
                with pytest.raises(ValueError):
                    mtmount.truncate_value(
                        value=0, min_value=min_value, max_value=max_value, descr=descr
                    )
            else:
                for diff, core_value in itertools.product(
                    (-0.001, 0, 0.001), (min_value, max_value)
                ):
                    value = core_value + diff
                    truncated_value, message = mtmount.truncate_value(
                        value=value,
                        min_value=min_value,
                        max_value=max_value,
                        descr=descr,
                    )
                    if min_value <= value <= max_value:
                        assert truncated_value == value
                        assert message == ""
                    else:
                        if value < min_value:
                            assert truncated_value == min_value
                        else:
                            assert truncated_value == max_value
                        assert descr in message
