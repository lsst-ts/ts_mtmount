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
import logging
import time
import unittest

import numpy as np
import pytest
from lsst.ts import mtmount, salobj, tcpip, utils

# Standard timeout for TCP/IP messages (sec).
STD_TIMEOUT = 10

# Time to wait for a connection attempt (sec).
CONNECT_TIMEOUT = 5


class TelemetryClientTestCase(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.log = logging.getLogger()
        self.log.setLevel(logging.INFO)
        self.log.addHandler(logging.StreamHandler())
        salobj.set_test_topic_subname()

    @contextlib.asynccontextmanager
    async def make_all(self):
        r"""Make a telemetry server, client, and remote.

        The client is run as a background process.

        Attributes
        ----------
        remote : `lsst.ts.salobj.Remote`
            MTMount remote to read DDS telemetry.
        server : `lsst.ts.tcpip.OneClientServer`
            A minimal telemetry server that can be used to manually
            send data to the telemetry client process.
        telemetry_client : `TelemetryClient`
            The telemetry client.
        """
        self.server = tcpip.OneClientServer(
            name="TelemetryServer",
            host=salobj.LOCAL_HOST,
            port=0,
            log=logging.getLogger(),
            connect_callback=None,
            terminator=mtmount.LINE_TERMINATOR,
        )
        # Wait for the server to start so the port is set
        await asyncio.wait_for(self.server.start_task, timeout=STD_TIMEOUT)

        # Wait for the remote to start before starting the telemetry client,
        # so the telemetry client will not time out waiting for telemetry.
        domain = salobj.Domain()
        self.remote = salobj.Remote(domain=domain, name="MTMount")

        await asyncio.wait_for(self.remote.start_task, timeout=STD_TIMEOUT)
        self.telemetry_client = mtmount.TelemetryClient(
            host=salobj.LOCAL_HOST,
            port=self.server.port,
        )
        await asyncio.wait_for(self.server.connected_task, timeout=STD_TIMEOUT)
        yield

    async def test_clock_offset_event(self):
        """Test the clockOffset event."""
        azimuth_data = dict(
            topicID=6,
            actualPosition=1,
            actualTorque=0.1,
            actualVelocity=0,
            demandPosition=1,
            demandVelocity=0,
            timestamp=0,  # change this value before sending
        )
        async with self.make_all():
            if not hasattr(self.remote, "evt_clockOffset"):
                raise unittest.SkipTest("MTMount has no clockOffset event in ts_xml 14")
            desired_offset = 0.4
            start_time = utils.current_tai()
            azimuth_data["timestamp"] = start_time + desired_offset
            await self.publish_data(azimuth_data)
            data = await self.remote.evt_clockOffset.next(
                flush=False, timeout=STD_TIMEOUT
            )
            while data.private_sndStamp <= start_time:
                self.log.debug(f"Discarding sample {data}.")
                data = await self.remote.evt_clockOffset.next(
                    flush=False, timeout=STD_TIMEOUT
                )

            assert data.offset == pytest.approx(desired_offset, abs=0.1)

            # Further telemetry should not produce new evt_clockOffset
            # until the timer has expired
            next_event_tai = data.private_sndStamp + mtmount.CLOCK_OFFSET_EVENT_INTERVAL

            # Write data until within interval_buffer seconds of writing
            # the next clockOffset event.
            # Use no clock offset, so we can detect if any of these
            # telemetry messages triggers a clockOffset event (none should).
            interval_buffer = 0.1  # must be << TELEMETRY_TIMEOUT
            while True:
                curr_tai = utils.current_tai()
                if curr_tai > next_event_tai - interval_buffer:
                    break
                azimuth_data["timestamp"] = curr_tai
                await self.publish_data(azimuth_data)
                # Sleep a bit to avoid flooding the telemetry client;
                # the exact amount doesn't matter.
                await asyncio.sleep(0.1)

            # Wait until the timer expires, then send one more
            # telemetry with a big clock offset.
            await asyncio.wait_for(
                self.telemetry_client.next_clock_offset_task, timeout=STD_TIMEOUT
            )

            desired_offset = -0.70
            curr_tai = utils.current_tai()
            azimuth_data["timestamp"] = curr_tai + desired_offset
            await self.publish_data(azimuth_data)
            data = await self.remote.evt_clockOffset.next(
                flush=False, timeout=STD_TIMEOUT
            )
            assert data.offset == pytest.approx(desired_offset, abs=0.1)

    async def test_handle_telemetry(self):
        """Test all telemetry topics."""
        random = np.random.default_rng(47)
        backward_compatile_telemetry_attribute = dict(
            safetySystem={
                "brakePressureAz",
                "brakePressureAzTimestamp",
                "brakePressureEl",
                "brakePressureElTimestamp",
            },
            mainCabinetThermal={
                "auxPxiTemperature",
                "auxPxiTemperatureTimestamp",
                "axesPxiTemperature",
                "axesPxiTemperatureTimestamp",
                "backupTemperature",
                "backupTemperatureTimestamp",
                "tmaPxiTemperature",
                "tmaPxiTemperatureTimestamp",
                "valveFeedback",
                "valveFeedbackTimestamp",
            },
            oilSupplySystem={
                "setpointTemperatureCabinets",
                "setpointTemperatureCabinetsTimestamp",
            },
        )
        async with self.make_all():
            for topic_id, (
                sal_topic_name,
                field_len_dict,
            ) in mtmount.RAW_TELEMETRY_MAP.items():
                topic = getattr(self.remote, f"tel_{sal_topic_name}")

                desired_sal_data = dict()
                tma_data = dict(topicID=topic_id)
                null_sal_data = topic.DataType()
                for fieldname, fieldlen in field_len_dict.items():
                    try:
                        null_field_data = getattr(null_sal_data, fieldname)
                    except AttributeError:
                        if (
                            sal_topic_name in backward_compatile_telemetry_attribute
                            and fieldname
                            in backward_compatile_telemetry_attribute[sal_topic_name]
                        ):
                            continue
                        else:
                            raise
                    if isinstance(null_field_data, list):
                        dtype = type(null_field_data[0])
                    else:
                        dtype = type(null_field_data)
                    if dtype is float:
                        values = random.uniform(0, 90, fieldlen)
                    elif dtype is int:
                        # Note: using dtype=int doesn't work; json refuses
                        # to serialize the values.
                        values = [
                            int(value) for value in random.integers(0, 10000, fieldlen)
                        ]
                    elif dtype is bool:
                        values = [
                            bool(value) for value in random.integers(0, 10000, fieldlen)
                        ]
                    else:
                        raise RuntimeError(
                            f"Unrecognized {dtype=} for {sal_topic_name}.{fieldname}, {null_field_data=}"
                        )
                    if fieldlen == 1:
                        desired_sal_data[fieldname] = values[0]
                        tma_data[fieldname] = values[0]
                    else:
                        desired_sal_data[fieldname] = list(values)
                        for i in range(fieldlen):
                            tma_data[
                                (
                                    f"{fieldname}{i+1}"
                                    if not fieldname.endswith("Timestamp")
                                    else f"{fieldname[:-9]}{i+1}Timestamp"
                                )
                            ] = values[i]

                await self.publish_data(tma_data)
                await self.assert_next_telemetry(topic, desired_sal_data)

    async def test_heartbeat(self):
        async with self.make_all():
            if not hasattr(self.remote, "tel_telemetryClientHeartbeat"):
                raise unittest.SkipTest(
                    "No telemetryClientHeartbeat topic; ts_xml too old."
                )
            await self.assert_next_telemetry(
                topic=self.remote.tel_telemetryClientHeartbeat,
                desired_data=dict(),
            )

    async def test_timeout(self):
        """Test client timeout.

        Start the telemetry server and client, but do not send any telemetry.
        Wait for the telemetry client to time out and exit.
        """
        async with self.make_all():
            await asyncio.wait_for(
                self.telemetry_client.start_task, timeout=STD_TIMEOUT
            )
            await asyncio.wait_for(
                self.telemetry_client.done_task,
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
        try:
            llv_data = dict(topicID=topic_id)
            field_dict = mtmount.TELEMETRY_MAP[topic_id][1]
            for key, value in dds_data.items():
                func = field_dict[key]
                llv_data.update(func.llv_dict_from_sal_value(value))
            return llv_data
        except Exception as e:
            raise RuntimeError(f"Could not convert {dds_data=} for {topic_id=}: {e!r}")

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
        await self.server.write_json(llv_data)
