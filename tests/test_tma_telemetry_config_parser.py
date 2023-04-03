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

import pathlib
import shutil
import subprocess
import tempfile
import unittest

import pytest
from lsst.ts import mtmount

DATA_DIR = pathlib.Path(__file__).parent / "data" / "tma_telemetry_config"


def assert_same_xml_str(str1, str2):
    """Compare two XML strings, ignoring "." before </Description>."""
    trimmed_str1 = str1.replace(".</Description>", "</Description>")
    trimmed_str2 = str2.replace(".</Description>", "</Description>")
    assert trimmed_str1 == trimmed_str2


class TMATelemetryConfigParserParserTestCase(unittest.TestCase):
    """Test TMATelemetryConfigParser."""

    def test_good_config(self):
        processor = mtmount.TMATelemetryConfigParser(
            tma_config_path=DATA_DIR / "good_TelemetryTopicsConfiguration.ini"
        )
        topics = processor.process_file()
        # Check topics; the Boolean Signals section is ignored
        # because it only has boolean data, none of which is published.
        assert list(topics.keys()) == [
            "tel_azimuth",
            "tel_azimuthDrives",
            "tel_azimuthDrivesThermal",
            "tel_mirrorCover",
            "tel_topEndChiller",
            "tel_cooling",
        ]
        with tempfile.NamedTemporaryFile() as temp_outfile:
            outpath = temp_outfile.name
            processor.write_xml(xml_telemetry_path=outpath, topics=topics)
            with open(outpath, "r") as outfile:
                outdata = outfile.read()
        with open(DATA_DIR / "MTMount_Telemetry.xml", mode="r") as infile:
            expected_data = infile.read()
        assert_same_xml_str(outdata, expected_data)

    def test_bin_script(self):
        exe_name = "run_tma_telemetry_config_parser"
        exe_path = shutil.which(exe_name)
        if exe_path is None:
            raise AssertionError(
                f"Could not find bin script {exe_name}; did you setup or install this package?"
            )

        tma_config_path = DATA_DIR / "good_TelemetryTopicsConfiguration.ini"
        with tempfile.NamedTemporaryFile() as temp_outfile:
            outpath = temp_outfile.name
            args = [exe_name, str(tma_config_path), "--output", outpath]
            subprocess.run(args)
            with open(outpath, "r") as outfile:
                outdata = outfile.read()
        with open(DATA_DIR / "MTMount_Telemetry.xml", mode="r") as infile:
            expected_data = infile.read()
        assert_same_xml_str(outdata, expected_data)

    def test_invalid_configs(self):
        for bad_config_path in DATA_DIR.glob("bad_*.ini"):
            with self.subTest(bad_config_path=bad_config_path):
                processor = mtmount.TMATelemetryConfigParser(
                    tma_config_path=bad_config_path
                )
                with pytest.raises(RuntimeError):
                    processor.process_file()
