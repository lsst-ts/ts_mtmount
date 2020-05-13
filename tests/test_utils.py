# This file is part of ts_ATDomeTrajectory.
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

from lsst.ts import MTMount
from lsst.ts.MTMount.utils import MAX_DOC_LENGTH


class UtilsTestCase(unittest.TestCase):
    """Test the utils subpackage."""

    def test_wrap_parameter_doc(self):
        # The following text was chosen so when duplicated and wrapped
        # some of the text reaches 79 columns.
        long_text = (
            "This is a long line of text that will reach column 79 when wrapped. " * 50
        )
        wrapped_text = MTMount.wrap_parameter_doc(long_text)
        lines = wrapped_text.split("\n")
        for line in lines:
            self.assertTrue(line.startswith("    "))
            self.assertLessEqual(len(line), MAX_DOC_LENGTH)


if __name__ == "__main__":
    unittest.main()
