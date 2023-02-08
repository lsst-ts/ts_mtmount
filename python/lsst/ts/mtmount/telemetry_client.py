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
import json
import logging
import signal

from lsst.ts import salobj, utils

from . import constants
from .telemetry_map import TELEMETRY_MAP

# Approximate interval between output of the clockOffset event (seconds).
# The clockOffset event is output when the first new telemetry is received
# after the timer has expired.
CLOCK_OFFSET_EVENT_INTERVAL = 1

# Timeout to read telemetry (sec). If no telemetry is seen
# in this time, the telemetry client logs an error and quits.
TELEMETRY_TIMEOUT = 2


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

    async def __call__(self, llv_data):
        """Process one low-level message.

        Parameters
        ----------
        llv_data : `dict`
            Dict of field name: value.
            Note: if there is preprocessor, this will be modified in place.
        """
        if self.preprocessor is not None:
            self.preprocessor(llv_data)
        sal_data = {
            sal_name: func.sal_value_from_llv_dict(llv_data)
            for sal_name, func in self.field_dict.items()
        }
        await self.topic.set_write(**sal_data)

    def __repr__(self):
        return (
            f"TopicHandler(topic={self.topic.name}; "
            f"field_dict={self.field_dict}; "
            f"preprocessor={self.preprocessor}"
        )


class TelemetryClient:
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
        self.host = host
        self.port = port
        self.controller = salobj.Controller(name="MTMount", write_only=True)
        self.log = self.controller.log.getChild("TelemetryClient")

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
        self.reader = None
        self.writer = None
        self.start_task = asyncio.create_task(self.start())
        self.read_task = asyncio.Future()
        self.done_task = asyncio.Future()

        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, self.signal_handler)

    @property
    def connected(self):
        if None in (self.reader, self.writer):
            return False
        return True

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
            connect_coro = asyncio.open_connection(host=self.host, port=self.port)
            self.reader, self.writer = await asyncio.wait_for(
                connect_coro, timeout=self.connection_timeout
            )
            await self.controller.evt_telemetryConnected.set_write(
                connected=self.connected
            )
        except Exception as e:
            err_msg = f"Could not open connection to host={self.host}, port={self.port}"
            self.log.exception(err_msg)
            self.done_task.set_exception(e)
            return

        self.read_task = asyncio.create_task(self.read_loop())
        self.log.info("running")

    def signal_handler(self):
        """Handle signals such as SIGTERM."""
        self.log.info("signal_handler")
        self.start_task.cancel()
        self.read_task.cancel()
        self.controller.salinfo.basic_close()
        self.controller.salinfo.domain.basic_close()
        writer = self.writer
        self.reader = None
        self.writer = None
        if writer:
            writer.close()
        if not self.done_task.done():
            self.done_task.set_result(None)

    async def close(self):
        """Disconnect from the TCP/IP controller."""
        self.log.info("disconnecting")
        self.start_task.cancel()
        self.read_task.cancel()
        self.next_clock_offset_task.cancel()
        writer = self.writer
        self.reader = None
        self.writer = None
        if writer:
            writer.close()
            try:
                await asyncio.wait_for(writer.wait_closed(), timeout=1)
            except asyncio.TimeoutError:
                self.log.warning(
                    "Timed out waiting for the writer to close; continuing"
                )
        await self.controller.evt_telemetryConnected.set_write(connected=self.connected)
        self.log.info("done")
        await self.controller.close()
        if not self.done_task.done():
            self.done_task.set_result(None)

    def get_preprocessor(self, sal_topic_name):
        """Get the preprocessor for this topic, if it exists, else None.

        Parameters
        ----------
        sal_topic_name : str
            Name of SAL telemetry topic, e.g. "tel_azimuthDrives".
        """
        return getattr(self, f"_preprocess_{sal_topic_name}", None)

    async def read_loop(self):
        """Read and process status from the low-level controller."""
        self.log.info("telemetry client read loop begins")
        try:
            while True:
                data = await asyncio.wait_for(
                    self.reader.readuntil(constants.LINE_TERMINATOR),
                    timeout=TELEMETRY_TIMEOUT,
                )
                try:
                    decoded_data = data.decode()
                    llv_data = json.loads(decoded_data)
                    if self.next_clock_offset_task.done():
                        clock_offset = llv_data["timestamp"] - utils.current_tai()
                        await self.controller.evt_clockOffset.set_write(
                            offset=clock_offset,
                        )
                        self.next_clock_offset_task = asyncio.create_task(
                            asyncio.sleep(CLOCK_OFFSET_EVENT_INTERVAL)
                        )
                    topic_id = llv_data["topicID"]
                    topic_handler = self.topic_handlers.get(topic_id)
                    if topic_handler is None:
                        if topic_id not in self.unsupported_topic_ids:
                            self.unsupported_topic_ids.add(topic_id)
                            self.log.info(
                                f"Ignoring unsupported topic ID {topic_id}; data={data}"
                            )
                        continue
                    await topic_handler(llv_data)
                except Exception:
                    self.log.exception(
                        f"read_loop could not handle {data}; continuing."
                    )
        except asyncio.CancelledError:
            self.log.info("telemetry client read loop cancelled")
        except asyncio.TimeoutError:
            self.log.error("Timed out waiting for telemetry; giving up.")
            asyncio.ensure_future(self.close())
        except (ConnectionResetError, asyncio.IncompleteReadError):
            self.log.info("Reader disconnected; giving up.")
            asyncio.ensure_future(self.close())
        except Exception:
            self.log.exception("read_loop failed; giving up.")
            asyncio.ensure_future(self.close())


def run_mtmount_telemetry_client():
    """Run MTMount telemetry client."""
    asyncio.run(TelemetryClient.amain())
