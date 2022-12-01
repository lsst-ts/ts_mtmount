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

import asyncio
import contextlib
import json
import logging
import time
import unittest

import numpy as np
from lsst.ts import mtmount, salobj, tcpip

# Standard timeout for TCP/IP messages (sec).
STD_TIMEOUT = 5

# Time to wait for a connection attempt (sec).
CONNECT_TIMEOUT = 5


class TelemetryClientTestCase(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.log = logging.getLogger()
        self.log.setLevel(logging.INFO)
        self.log.addHandler(logging.StreamHandler())
        salobj.set_random_lsst_dds_partition_prefix()

    @contextlib.asynccontextmanager
    async def make_all(self, fail_if_end_early):
        r"""Make a telemetry server, client, and remote.

        The client is run as a background process.

        Attributes
        ----------
        server : `lsst.ts.tcpip.OneClientServer`
            Telemetry server. Publishes telemetry over TCP/IP.
        remote : `lsst.ts.salobj.Remote`
            MTMount remote to read DDS telemetry.
        """
        self.server = tcpip.OneClientServer(
            name="TelemetryServer",
            host=salobj.LOCAL_HOST,
            port=0,
            log=logging.getLogger(),
            connect_callback=None,
        )
        # Wait for the server to start so the port is set
        await asyncio.wait_for(self.server.start_task, timeout=STD_TIMEOUT)

        args = [
            "run_mtmount_telemetry_client",
            f"--host={salobj.LOCAL_HOST}",
            f"--port={self.server.port}",
        ]
        domain = salobj.Domain()
        self.remote = salobj.Remote(domain=domain, name="MTMount")
        await asyncio.wait_for(self.remote.start_task, timeout=STD_TIMEOUT)
        self.telemetry_client_process = await asyncio.create_subprocess_exec(*args)
        await asyncio.wait_for(self.server.connected_task, timeout=STD_TIMEOUT)
        try:
            yield
        finally:
            process_ended_early = self.telemetry_client_process.returncode is not None
            if not process_ended_early:
                self.telemetry_client_process.terminate()
            await asyncio.gather(
                self.remote.close(),
                self.server.close(),
                domain.close(),
            )
            if process_ended_early and fail_if_end_early:
                self.fail("telemetry client exited early")

    async def test_telemetry_x(self):
        """Test all telemetry topics."""
        async with self.make_all(fail_if_end_early=True):
            # Arbitrary values that are suitable for both
            # the elevation and azimuth telemetry topics.
            desired_elaz_dds_data = dict(
                actualPosition=55.1,
                demandPosition=45.2,
                actualVelocity=1.3,
                demandVelocity=1.4,
                actualTorque=3.3,
                timestamp=time.time(),
            )
            # TODO DM-37115: remove this variable and use desired_elaz_dds_data
            # when the TMA azimuth has the correct sign.
            desired_az_dds_data = desired_elaz_dds_data.copy()
            for field in (
                "actualPosition",
                "demandPosition",
                "actualVelocity",
                "demandVelocity",
                "actualTorque",
            ):
                desired_az_dds_data[field] = -desired_az_dds_data[field]

            # topic_id is from telemetry_map.yaml
            azimuth_llv_data = self.convert_dds_data_to_llv(
                dds_data=desired_elaz_dds_data,
                topic_id=mtmount.TelemetryTopicId.AZIMUTH,
            )
            azimuth_llv_data["this_extra_field_should_be_ignored"] = 55.2
            await self.publish_data(azimuth_llv_data)
            await self.assert_next_telemetry(
                self.remote.tel_azimuth, desired_az_dds_data
            )

            elevation_llv_data = self.convert_dds_data_to_llv(
                dds_data=desired_elaz_dds_data,
                topic_id=mtmount.TelemetryTopicId.ELEVATION,
            )
            await self.publish_data(elevation_llv_data)
            await self.assert_next_telemetry(
                self.remote.tel_elevation, desired_elaz_dds_data
            )

            desired_azimuth_drives_dds_data = self.make_elaz_drives_dds_data(naxes=16)
            azimuth_drives_llv_data = self.convert_dds_data_to_llv(
                dds_data=desired_azimuth_drives_dds_data,
                topic_id=mtmount.TelemetryTopicId.AZIMUTH_DRIVE,
            )
            await self.publish_data(azimuth_drives_llv_data)
            await self.assert_next_telemetry(
                self.remote.tel_azimuthDrives, desired_azimuth_drives_dds_data
            )

            desired_elevation_drives_dds_data = self.make_elaz_drives_dds_data(naxes=12)
            elevation_drives_llv_data = self.convert_dds_data_to_llv(
                dds_data=desired_elevation_drives_dds_data,
                topic_id=mtmount.TelemetryTopicId.ELEVATION_DRIVE,
            )
            await self.publish_data(elevation_drives_llv_data)
            await self.assert_next_telemetry(
                self.remote.tel_elevationDrives, desired_elevation_drives_dds_data
            )

            desired_ccw_dds_data = dict(
                actualPosition=12.3,
                actualVelocity=-34.5,
                actualTorquePercentage=[0.11, 0.12],
                timestamp=time.time(),
            )
            ccw_llv_data = self.convert_dds_data_to_llv(
                dds_data=desired_ccw_dds_data,
                topic_id=mtmount.TelemetryTopicId.CAMERA_CABLE_WRAP,
            )
            await self.publish_data(ccw_llv_data)
            await self.assert_next_telemetry(
                self.remote.tel_cameraCableWrap, desired_ccw_dds_data
            )

    async def test_timeout(self):
        async with self.make_all(fail_if_end_early=False):
            await asyncio.wait_for(
                self.telemetry_client_process.wait(),
                timeout=mtmount.TELEMETRY_TIMEOUT + STD_TIMEOUT,
            )

    async def assert_next_telemetry(
        self, topic, desired_data, delta=1e-7, timeout=STD_TIMEOUT
    ):
        """Assert that the next telemetry topic matches the desired data.

        Parameters
        ----------
        topic : `lsst.ts.salobj.topics.RemoteTelemetry`
            Telemetry topic to read.
        desired_data : `dict`
            Desired telemetry data, as DDS field name: value.
            Float values are expected to agree to within ``delta``.
        delta : `float`, optional
            Maximum allowed difference for float values.
        timeout : `float`, optional
            Maximum time to wait for a message (sec).

        Returns
        -------
        message
            The telemetry message.
        """
        message = await topic.next(flush=False, timeout=timeout)
        data = vars(message)
        error_msgs = list()
        for key, desired_value in desired_data.items():
            value = data[key]
            if isinstance(value, float):
                if abs(value - desired_value) > delta:
                    error_msgs.append(
                        f"{key} {value} - {desired_value} = {value - desired_value}"
                    )
            elif isinstance(value, list):
                if isinstance(value[0], float):
                    np.testing.assert_allclose(value, desired_value, atol=delta)
                else:
                    assert value == desired_value
            else:
                if value != desired_value:
                    error_msgs.append(f"{key} {value} != {desired_value}")
        if error_msgs:
            self.fail(", ".join(error_msgs))
        return message

    def convert_dds_data_to_llv(self, dds_data, topic_id):
        """Convert a telemetry dict from DDS to low-level controller format.

        Parameters
        ----------
        dds_data : `dict`
            Dict of dds field name: value telemetry data.
        topic_id : `TelemetryTopicId`:
            Topic ID for low-level data.

        Returns
        -------
        llv_data : `dict`
            Dict of low-level field name: value telemetry data.
        """
        llv_data = dict(topicID=topic_id)
        field_dict = mtmount.TELEMETRY_MAP[topic_id][1]
        for key, value in dds_data.items():
            func = field_dict[key]
            llv_data.update(func.llv_dict_from_sal_value(value))
        return llv_data

    def make_elaz_drives_dds_data(self, naxes):
        """Make telemetry data for elevation or azimuth drives.

        Parameters
        ----------
        naxes : `int`
            Number of axes (12 for elevation, 16 for azimuth).

        Returns
        -------
        dds_data : `dict`
            DDS telemetry data, as field name: value.
        """
        return dict(
            current=[n * 0.1 for n in range(1, naxes + 1)],
            timestamp=time.time(),
        )

    async def publish_data(self, llv_data):
        """Write telemetry data to the telemetry TCP/IP port.

        Parameters
        ----------
        llv_data : `dict`
            Low-level controller telemetry data.
        """
        data_json = json.dumps(llv_data)
        self.server.writer.write(data_json.encode() + mtmount.LINE_TERMINATOR)
        await self.server.writer.drain()
