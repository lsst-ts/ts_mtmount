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

__all__ = [
    "AxisStateDict",
    "DriveStateDict",
    "TelemetryTopicHandler",
    "TelemetryClient",
]

import argparse
import asyncio
import json
import logging
import math
import pathlib

import yaml

from lsst.ts.idl.enums.MTMount import AxisState, DriveState
from lsst.ts import salobj
from . import constants


AxisStateDict = {
    "Unknown": AxisState.UNKNOWN,
    "Off": AxisState.OFF,
    "Enabled": AxisState.ENABLED,
    "Fault": AxisState.FAULT,
}


DriveStateDict = {
    "Unknown": DriveState.UNKNOWN,
    "Off": DriveState.OFF,
    "Standstill": DriveState.STOPPED,
    "Discrete Motion": DriveState.MOVING,
    "Stopping": DriveState.STOPPING,
    "Fault": DriveState.FAULT,
}


class TelemetryTopicHandler:
    """Functor that takes a telemetry message from the low-level controller
    and outputs the associated SAL telemetry message.

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

    def __call__(self, llv_data):
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
            sal_name: llv_data[llv_name]
            for sal_name, llv_name in self.field_dict.items()
        }
        self.topic.set_put(**sal_data)

    def __repr__(self):
        return (
            f"TopicHandler(topic={self.topic.name}; "
            f"field_dict={self.field_dict}; "
            f"preprocessor={self.preprocessor}"
        )


class TelemetryClient:
    on_drive_states = frozenset(("Standstill", "Discrete Motion", "Stopping"))

    def __init__(
        self, host, port=constants.TELEMETRY_PORT, connection_timeout=10,
    ):
        self.host = host
        self.port = port
        self.controller = salobj.Controller(name="MTMount")
        # Cancel the controller read loop; do not want this controller
        # to acknowledge commands and we do not need the read loop
        # in order to write messages.
        self.controller.start_task.cancel()
        self.log = self.controller.log.getChild("TelemetryClient")

        self.connection_timeout = connection_timeout
        telemetry_map_path = (
            pathlib.Path(__file__).parents[4] / "data" / "telemetry_map.yaml"
        )
        with open(telemetry_map_path, "r") as f:
            raw_translation_data = f.read()
        translation_dict = yaml.safe_load(raw_translation_data)
        # dict of low-level controller topic ID: TelemetryTopicHandler
        self.topic_handlers = {
            topic_id: TelemetryTopicHandler(
                topic=getattr(self.controller, f"tel_{sal_topic_name}"),
                field_dict=field_dict,
                preprocessor=self.get_preprocessor(sal_topic_name),
            )
            for topic_id, (sal_topic_name, field_dict) in translation_dict.items()
        }
        # Keep track of unsupported topic IDs
        # in order to report new ones.
        self.unsupported_topic_ids = set()
        self.reader = None
        self.writer = None
        self.start_task = asyncio.create_task(self.start())
        self.read_task = asyncio.Future()
        self.done_task = asyncio.Future()

    @property
    def connected(self):
        if None in (self.reader, self.writer):
            return False
        return True

    @classmethod
    async def amain(cls):
        parser = argparse.ArgumentParser("Run the MTMount telemetry client")
        parser.add_argument(
            "--host", help="Telemetry server host.",
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
        telemetry_client = cls(host=namespace.host, port=namespace.port,)
        telemetry_client.log.setLevel(namespace.loglevel)
        try:
            print("MTMount telemetry client starting")
            await telemetry_client.start_task
            print("MTMount telemetry client running")
            await telemetry_client.done_task
        except asyncio.CancelledError:
            print("MTMount telemetry client done")
        except Exception as e:
            print(f"MTMount telemetry client failed: {e}")

    async def start(self):
        """Connect to the telemetry port and start the read loop.
        """
        self.log.debug("start")
        if self.connected:
            raise RuntimeError("Already connected")
        try:
            connect_coro = asyncio.open_connection(host=self.host, port=self.port)
            self.reader, self.writer = await asyncio.wait_for(
                connect_coro, timeout=self.connection_timeout
            )
            self.log.debug("connected")
        except Exception as e:
            err_msg = f"Could not open connection to host={self.host}, port={self.port}"
            self.log.exception(err_msg)
            self.done_task.set_exception(e)
            return

        self.read_task = asyncio.create_task(self.read_loop())

    async def close(self):
        """Disconnect from the TCP/IP controller.
        """
        self.log.debug("disconnect")
        self.start_task.cancel()
        self.read_task.cancel()
        await self.controller.close()
        writer = self.writer
        self.reader = None
        self.writer = None
        if writer:
            writer.close()
            await writer.wait_closed()
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
        """Read and process status from the low-level controller.
        """
        while True:
            try:
                data = await self.reader.readuntil(b"\r\n")
            except asyncio.CancelledError:
                return
            except (ConnectionResetError, asyncio.IncompleteReadError):
                asyncio.ensure_future(self.close())
                self.log.info("Reader disconnected; giving up.")
                return
            except Exception:
                asyncio.ensure_future(self.close())
                self.log.exception("read_loop failed; giving up.")
                return
            try:
                decoded_data = data.decode()
                llv_data = json.loads(decoded_data)
                topic_id = llv_data["topicID"]
                topic_handler = self.topic_handlers.get(topic_id)
                if topic_handler is None:
                    if topic_id not in self.unsupported_topic_ids:
                        self.unsupported_topic_ids.add(topic_id)
                        self.log.debug(f"Ignoring unsupported topic ID {topic_id}")
                    continue
                topic_handler(llv_data)
            except Exception:
                self.log.exception(f"read_loop could not handle {data}; continuing.")

    def _convert_drive_measurements(self, llv_data, keys, ndrives):
        """Convert per-drive measurements or other items numbered from 1

        Parameters
        ----------
        llv_data : dict
            Data from the low-level controller. Modified in place.
        keys : list [str]
            Prefix for named items; full names have the drive number appended.
            For example "azCurrent" refers to "azCurrent1" - "azCurrent16".
        ndrives : int
            The number of drives.
        """
        for key in keys:
            llv_data[key] = [
                llv_data.pop(f"{key}{n}", math.nan) for n in range(1, ndrives + 1)
            ]

    def _convert_drive_states(self, llv_data, key, ndrives):
        """Convert drive state strings.

        Parameters
        ----------
        llv_data : dict
            Data from the low-level controller. Modified in place.
        key : str
            Prefix for named items; one of "azStatusDrive",
            "cCWStatusDrive", or "elStatusDrive".
        ndrives : int
            The number of drives.
        """
        llv_data[key] = [
            DriveStateDict.get(llv_data.pop(f"{key}{n}", "Unknown"), DriveState.UNKNOWN)
            for n in range(1, ndrives + 1)
        ]

    def _preprocess_azimuthDrives(self, llv_data):
        """Preprocess status for the tel_azimuthDrives topic."""
        self._convert_drive_measurements(
            llv_data=llv_data, keys=["azCurrent"], ndrives=16
        )

    def _preprocess_cameraCableWrap(self, llv_data):
        """Preprocess status for the tel_cameraCableWrap topic.

        This method is rather complicated because the data
        output by the CCW as of 2021-01-11 does not look much like
        the data that will be output by the final TMA.
        For now the CCW outputs cCWAngle1, cCWAngle2, cCWSpeed1, and cCWSpeed2,
        where the 1 values are only valid if drive 1 is on,
        and the 2 values are only valid if drive 2 is on.
        The positions are moderately close, the speeds less so.
        For now I report the 1 values if drive 1 is on and the 2 values
        otherwise, because we tend to use drive 2.
        """
        drive_state1 = llv_data.get("cCWStatusDrive1", None)
        if drive_state1 in self.on_drive_states:
            llv_data["angleActual"] = llv_data["cCWAngle1"]
            llv_data["velocityActual"] = llv_data["cCWSpeed1"]
        else:
            llv_data["angleActual"] = llv_data["cCWAngle2"]
            llv_data["velocityActual"] = llv_data["cCWSpeed2"]
        llv_data["accelerationActual"] = math.nan
        llv_data["angleSet"] = math.nan
        llv_data["velocitySet"] = math.nan

    def _preprocess_elevationDrives(self, llv_data):
        """Preprocess status for the tel_elevationDrives topic."""
        self._convert_drive_measurements(
            llv_data=llv_data, keys=["elCurrent"], ndrives=12
        )
