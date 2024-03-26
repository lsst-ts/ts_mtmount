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

__all__ = [
    "CLOCK_OFFSET_EVENT_INTERVAL",
    "TELEMETRY_TIMEOUT",
    "TelemetryTopicHandler",
    "TelemetryClient",
    "run_mtmount_telemetry_client",
]

import argparse
import asyncio
import logging
import signal
import time

from lsst.ts import salobj, tcpip, utils

from . import constants
from .telemetry_map import TELEMETRY_MAP

# Approximate interval between output of the clockOffset event (seconds).
# The clockOffset event is output when the first new telemetry is received
# after the timer has expired.
CLOCK_OFFSET_EVENT_INTERVAL = 1

# Timeout to read telemetry (sec). If no telemetry is seen
# in this time, the telemetry client logs an error and quits.
# 1 second is the maximum time MTRotator will wait for CCW telemetry.
TELEMETRY_TIMEOUT = 1


class TelemetryTopicHandler:
    """Coroutine functor that takes a telemetry message from the low-level
    controller and outputs the associated SAL telemetry message.

    Parameters
    ----------
    topic : `salobj.topics.ControllerTelemetry`
        SAL telemetry topic.
    field_dict : `dict`
        Dicts of SAL topic field name: low-level message field name
    preprocessor : callable or `None`, optional
        Function to preprocess the low-level message.
    """

    def __init__(self, topic, field_dict, preprocessor=None):
        self.topic = topic
        self.field_dict = field_dict
        self.preprocessor = preprocessor

    async def __call__(self, data_dict):
        """Process one low-level message.

        Parameters
        ----------
        data_dict : `dict`
            Dict of field name: value.
            Note: if there is preprocessor, this will be modified in place.
        """
        if self.preprocessor is not None:
            self.preprocessor(data_dict)
        sal_data = {
            sal_name: func.sal_value_from_llv_dict(data_dict)
            for sal_name, func in self.field_dict.items()
        }
        await self.topic.set_write(**sal_data)

    def __repr__(self):
        return (
            f"TopicHandler(topic={self.topic.name}; "
            f"field_dict={self.field_dict}; "
            f"preprocessor={self.preprocessor}"
        )


class TelemetryClient(tcpip.Client):
    """Read telemetry data from the TMA and publish as SAL messages.

    Paramaters
    ----------
    host : `str`
        Host IP address of TMA telemetry server.
    port : `int`
        Host IP port of TMA telemetry server.
    connection_timeout : `float`
        Connection timeout (sec).

    Notes
    -----
    Methods with the name "_preprocess_{sal_topic_name}" are used
    to tweak telemetry data. They receive one argument:
    the raw telemetry data as a dict, and must modify it "in place".
    """

    on_drive_states = frozenset(("Standstill", "Discrete Motion", "Stopping"))

    def __init__(
        self,
        host,
        port=constants.TELEMETRY_PORT,
        connection_timeout=10,
    ):
        self.controller = salobj.Controller(name="MTMount", write_only=True)

        # read_loop should output the clockOffset event
        # for the next telemetry received when this timer expires
        self.next_clock_offset_task = utils.make_done_future()

        self.connection_timeout = connection_timeout
        # dict of low-level controller topic ID: TelemetryTopicHandler
        self.topic_handlers = {
            topic_id: TelemetryTopicHandler(
                topic=getattr(self.controller, f"tel_{sal_topic_name}"),
                field_dict=field_dict,
                preprocessor=self.get_preprocessor(sal_topic_name),
            )
            for topic_id, (sal_topic_name, field_dict) in TELEMETRY_MAP.items()
        }
        # Keep track of unsupported topic IDs
        # in order to report new ones.
        self.unsupported_topic_ids = set()

        self.heartbeat_interval = 1
        self.heartbeat_task = utils.make_done_future()
        self.read_task = asyncio.Future()

        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, self.signal_handler)

        super().__init__(
            name="TelemetryClient",
            host=host,
            port=port,
            log=self.controller.log.getChild("TelemetryClient"),
            connect_callback=self.connect_callback,
            monitor_connection_interval=0,
            terminator=constants.LINE_TERMINATOR,
        )

    @classmethod
    async def amain(cls):
        parser = argparse.ArgumentParser("Run the MTMount telemetry client")
        parser.add_argument(
            "--host",
            help="Telemetry server host.",
        )
        parser.add_argument(
            "--port",
            type=int,
            default=constants.TELEMETRY_PORT,
            help="Telemetry server port.",
        )
        parser.add_argument(
            "--loglevel",
            type=int,
            default=logging.INFO,
            help="Log level (DEBUG=10, INFO=20, WARNING=30).",
        )
        namespace = parser.parse_args()
        if namespace.host is None:
            parser.error("--host is required")
        print(
            "MTMount telemetry client: "
            f"host={namespace.host}; "
            f"port={namespace.port}"
        )
        logging.basicConfig()
        telemetry_client = cls(host=namespace.host, port=namespace.port)
        telemetry_client.log.setLevel(namespace.loglevel)
        try:
            print("MTMount telemetry client starting")
            await telemetry_client.start_task
            print("MTMount telemetry client running")
            await telemetry_client.done_task
        except asyncio.CancelledError:
            print("MTMount telemetry client done")
        except Exception as e:
            print(f"MTMount telemetry client failed: {e!r}")

    async def start(self):
        """Connect to the telemetry port and start the read loop."""
        await self.controller.start_task
        await self.controller.evt_telemetryConnected.set_write(connected=self.connected)
        self.log.debug("connecting")
        if self.connected:
            raise RuntimeError("Already connected")
        try:
            await asyncio.wait_for(super().start(), timeout=self.connection_timeout)
            await self.controller.evt_telemetryConnected.set_write(
                connected=self.connected
            )
        except Exception as e:
            self.log.exception(
                f"Could not open connection to host={self.host}, port={self.port}: {e!r}"
            )
            if not self.done_task.done():
                self.done_task.set_exception(e)
            return

    def signal_handler(self):
        """Handle signals such as SIGTERM."""
        self.log.info("signal_handler")
        self.heartbeat_task.cancel()
        self.read_task.cancel()
        self.next_clock_offset_task.cancel()
        self.controller.salinfo.basic_close()
        self.controller.salinfo.domain.basic_close()
        asyncio.create_task(self.close())

    async def connect_callback(self, client):
        await self.controller.evt_telemetryConnected.set_write(connected=self.connected)
        if self.connected:
            if hasattr(self.controller, "tel_telemetryClientHeartbeat"):
                self.heartbeat_task = asyncio.create_task(self.heartbeat_loop())
            else:
                self.log.warning(
                    "Cannot run telemetry client heartbeat loop: "
                    "no telemetryClientHeartbeat telemetry topic (ts_xml is too old)"
                )
            self.read_task = asyncio.create_task(self.read_loop())
            self.log.info("running")
        else:
            self.log.info("disconnecting")
            self.heartbeat_task.cancel()
            self.read_task.cancel()
            self.next_clock_offset_task.cancel()
            await self.controller.close()

    def get_preprocessor(self, sal_topic_name):
        """Get the preprocessor for this topic, if it exists, else None.

        Parameters
        ----------
        sal_topic_name : str
            Name of SAL telemetry topic, e.g. "tel_azimuthDrives".
        """
        return getattr(self, f"_preprocess_{sal_topic_name}", None)

    async def heartbeat_loop(self):
        """Output telemetryClientHeartbeat telemetry at regular intervals."""
        self.log.info("Telemetry client heartbeat loop begins")
        try:
            while True:
                await self.controller.tel_telemetryClientHeartbeat.write()
                await asyncio.sleep(self.heartbeat_interval)
        except Exception as e:
            self.log.exception(f"Telemetry client heartbeat loop failed: {e!r}")

    async def read_loop(self):
        """Read and process status from the low-level controller."""
        self.log.info("telemetry client read loop begins")
        # IDs for some azimuth and elevation telemetry topics suitable
        # for detecting clock offset.
        azel_topic_ids = frozenset((5, 6, 14, 15))
        try:
            while self.connected:
                t0 = time.monotonic()
                data_dict = await asyncio.wait_for(
                    self.read_json(),
                    timeout=TELEMETRY_TIMEOUT,
                )
                dt = time.monotonic() - t0
                # The 0.1 provides a bit of margin, in case asyncio.wait_for
                # is working properly but did not quite hit its limit.
                if dt > TELEMETRY_TIMEOUT + 0.1:
                    raise asyncio.TimeoutError(
                        f"Timed out waiting for telemetry: {dt=:0.2f} > {TELEMETRY_TIMEOUT=}. "
                        "Detected by measuring the delay instead of asyncio.wait_for, "
                        "so the the event loop is overloaded."
                    )
                try:
                    topic_id = data_dict["topicID"]
                    if (
                        self.next_clock_offset_task.done()
                        and topic_id in azel_topic_ids
                    ):
                        clock_offset = data_dict["timestamp"] - utils.current_tai()
                        await self.controller.evt_clockOffset.set_write(
                            offset=clock_offset,
                        )
                        self.next_clock_offset_task = asyncio.create_task(
                            asyncio.sleep(CLOCK_OFFSET_EVENT_INTERVAL)
                        )
                    topic_handler = self.topic_handlers.get(topic_id)
                    if topic_handler is None:
                        if topic_id not in self.unsupported_topic_ids:
                            self.unsupported_topic_ids.add(topic_id)
                            self.log.info(
                                f"Ignoring unsupported topic ID {topic_id}; {data_dict=}"
                            )
                        continue
                    await topic_handler(data_dict)
                except Exception:
                    self.log.exception(
                        f"read_loop could not handle {data_dict}; continuing."
                    )
            self.log.info("Telemetry client read loop ends: not connected")
        except asyncio.CancelledError:
            self.log.info("Telemetry client read loop cancelled")
        except asyncio.TimeoutError:
            self.log.error(
                "Telemetry client timed out waiting for telemetry; giving up."
            )
            asyncio.ensure_future(self.close())
        except (ConnectionResetError, asyncio.IncompleteReadError):
            self.log.info("Telemetry client lost its connection; giving up.")
            asyncio.ensure_future(self.close())
        except Exception as e:
            self.log.exception(f"Telemetry client read loop failed; giving up: {e!r}")
            asyncio.ensure_future(self.close())


def run_mtmount_telemetry_client():
    """Run MTMount telemetry client."""
    asyncio.run(TelemetryClient.amain())
