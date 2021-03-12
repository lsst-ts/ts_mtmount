# This file is part of ts_MTMount.
#
# Developed for Vera Rubin Observatory.
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

import asynctest
import numpy as np

from lsst.ts import salobj
from lsst.ts import hexrotcomm
from lsst.ts import MTMount

# Standard timeout for TCP/IP messages (sec).
STD_TIMEOUT = 5

# Time to wait for a connection attempt (sec).
CONNECT_TIMEOUT = 5


class TelemetryClientTestCase(asynctest.TestCase):
    async def setUp(self):
        self.log = logging.getLogger()
        self.log.setLevel(logging.INFO)
        self.log.addHandler(logging.StreamHandler())

        # List of (server name, is connected). A new item is appended
        # each time self.connect_callback is called.
        self.connect_callback_data = []

    @contextlib.asynccontextmanager
    async def make_all(self):
        r"""Make a telemetry server, client, and remote.

        The client is run as a background process.

        Attributes
        ----------
        server : `lsst.ts.hexrotcomm.OneClientServer`
            Telemetry server. Publishes telemetry over TCP/IP.
        remote : `lsst.ts.salobj.Remote`
            MTMount remote to read DDS telemetry.
        """
        self.server = hexrotcomm.OneClientServer(
            name="TelemetryServer",
            host=salobj.LOCAL_HOST,
            port=0,
            log=logging.getLogger(),
            connect_callback=None,
        )
        # Wait for the server to start so the port is set
        await self.server.start_task

        args = [
            "run_mtmount_telemetry_client.py",
            f"--host={salobj.LOCAL_HOST}",
            f"--port={self.server.port}",
        ]
        telemetry_client_process = await asyncio.create_subprocess_exec(*args)
        domain = salobj.Domain()
        self.remote = salobj.Remote(domain=domain, name="MTMount")
        await asyncio.gather(self.server.connected_task, self.remote.start_task)
        try:
            yield
        finally:
            process_ended_early = telemetry_client_process.returncode is not None
            if not process_ended_early:
                telemetry_client_process.terminate()
            await asyncio.gather(
                self.remote.close(), self.server.close(), domain.close(),
            )
            if process_ended_early:
                self.fail("telemetry client exited early")

    async def test_telemetry(self):
        """Test all telemetry topics.
        """
        async with self.make_all():
            # Arbitrary values that are suitable for both
            # the elevation and azimuth telemetry topics.
            desired_elaz_dds_data = dict(
                actualPosition=55.1,
                demandPosition=45.2,
                actualVelocity=1.3,
                demandVelocity=1.4,
                actualAcceleration=-0.5,
                actualTorque=3.3,
                timestamp=time.time(),
            )
            # topic_id is from telemetry_map.yaml
            azimuth_llv_data = self.convert_dds_data_to_llv(
                dds_data=desired_elaz_dds_data,
                topic_id=MTMount.TelemetryTopicId.AZIMUTH,
            )
            azimuth_llv_data["this_extra_field_should_be_ignored"] = 55.2
            await self.publish_data(azimuth_llv_data)
            await self.assert_next_telemetry(
                self.remote.tel_azimuth, desired_elaz_dds_data
            )

            elevation_llv_data = self.convert_dds_data_to_llv(
                dds_data=desired_elaz_dds_data,
                topic_id=MTMount.TelemetryTopicId.ELEVATION,
            )
            await self.publish_data(elevation_llv_data)
            await self.assert_next_telemetry(
                self.remote.tel_elevation, desired_elaz_dds_data
            )

            desired_azimuth_drives_dds_data = self.make_elaz_drives_dds_data(naxes=16)
            azimuth_drives_llv_data = self.convert_dds_data_to_llv(
                dds_data=desired_azimuth_drives_dds_data,
                topic_id=MTMount.TelemetryTopicId.AZIMUTH_DRIVE,
            )
            await self.publish_data(azimuth_drives_llv_data)
            await self.assert_next_telemetry(
                self.remote.tel_azimuthDrives, desired_azimuth_drives_dds_data
            )

            desired_elevation_drives_dds_data = self.make_elaz_drives_dds_data(naxes=12)
            elevation_drives_llv_data = self.convert_dds_data_to_llv(
                dds_data=desired_elevation_drives_dds_data,
                topic_id=MTMount.TelemetryTopicId.ELEVATION_DRIVE,
            )
            await self.publish_data(elevation_drives_llv_data)
            await self.assert_next_telemetry(
                self.remote.tel_elevationDrives, desired_elevation_drives_dds_data
            )

            desired_ccw_dds_data = dict(
                actualPosition=12.3,
                actualVelocity=-34.5,
                actualAcceleration=0.25,
                timestamp=time.time(),
            )
            ccw_llv_data = self.convert_dds_data_to_llv(
                dds_data=desired_ccw_dds_data,
                topic_id=MTMount.TelemetryTopicId.CAMERA_CABLE_WRAP,
            )
            await self.publish_data(ccw_llv_data)
            await self.assert_next_telemetry(
                self.remote.tel_cameraCableWrap, desired_ccw_dds_data
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
            Maximum time to wait for a message (seconds).

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
        field_dict = MTMount.TELEMETRY_MAP[topic_id][1]
        for key, value in dds_data.items():
            llv_key = field_dict[key]
            self.convert_dds_item_to_llv(
                llv_data=llv_data, llv_key=llv_key, value=value,
            )
        llv_data["topicID"] = topic_id
        return llv_data

    def convert_dds_item_to_llv(self, llv_data, llv_key, value):
        """Convert one telemetry item from DDS to low-level controller format.

        This usually adds a prefix and in some cases modifies the value.

        Parameters
        ----------
        llv_data : dict
            Dict of low-level data. Modified in place.
        llv_key : `str`
            Low-level field name (or field name prefix for array values).
        value : `str`
            DDS value.
        """
        if isinstance(value, list):
            for i, item in enumerate(value):
                llv_data[llv_key + str(i + 1)] = item
        else:
            llv_data[llv_key] = value

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
            current=[n * 0.1 for n in range(1, naxes + 1)], timestamp=time.time(),
        )

    async def publish_data(self, llv_data):
        """Write telemetry data to the telemetry TCP/IP port.

        Parameters
        ----------
        llv_data : `dict`
            Low-level controller telemetry data.
        """
        data_json = json.dumps(llv_data)
        self.server.writer.write(data_json.encode() + MTMount.LINE_TERMINATOR)
        await self.server.writer.drain()


if __name__ == "__main__":
    unittest.main()
