# This file is part of ts_MTMount.
#
# Developed for the LSST Data Management System.
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
import unittest

import asynctest

from lsst.ts import salobj
from lsst.ts import MTMount

# Standard timeout for TCP/IP messages (sec).
STD_TIMEOUT = 0.01

# Time to wait for a connection attempt (sec).
CONNECT_TIMEOUT = 5


class ClientServerPairTestCase(asynctest.TestCase):
    """Test `lsst.ts.MTMount.ClientServerPair` by connecting two together.
    """

    async def setUp(self):
        self.log = logging.getLogger()
        self.log.setLevel(logging.INFO)
        self.log.addHandler(logging.StreamHandler())

        # List of (server name, is connected). A new item is appended
        # each time self.connect_callback is called.
        self.connect_callback_data = []

    @contextlib.asynccontextmanager
    async def make_pairs(self, connect=True, use_connect_callback=True):
        """Make two ``lsst.ts.MTMount.ClientServerPairs``, ``self.pair1`` and
        ``self.pair2``, that talk to each other.

        Parameters
        ----------
        connect : `bool`, optional
            Connect the pairs to each other?
        use_connect_callback : `bool`, optional
            Specify a connect_callback?
            If True then use self.connect_callback.
        """
        connect_callback = self.connect_callback if use_connect_callback else None

        self.pair1 = MTMount.ClientServerPair(
            name="pair1",
            client_host=None,
            client_port=0,  # Not yet known
            server_host=salobj.LOCAL_HOST,
            server_port=0,  # Pick random
            log=self.log,
            connect=False,
            connect_callback=connect_callback,
        )
        self.assertFalse(self.pair1.server_connected)
        self.assertFalse(self.pair1.client_connected)
        self.assertFalse(self.pair1.connected)
        # Wait for pair1 server to start, so its port is assigned
        await asyncio.wait_for(self.pair1.start_task, timeout=STD_TIMEOUT)
        self.assertNotEqual(self.pair1.server_port, 0)
        self.assertFalse(self.pair1.server_connected)
        self.assertFalse(self.pair1.client_connected)
        self.assertFalse(self.pair1.connected)

        self.pair2 = MTMount.ClientServerPair(
            name="pair2",
            client_host=None,
            client_port=self.pair1.server_port,
            server_host=salobj.LOCAL_HOST,
            server_port=0,
            log=self.log,
            connect=False,
            connect_callback=connect_callback,
        )
        # Wait for pair2 server to start
        self.assertFalse(self.pair2.server_connected)
        self.assertFalse(self.pair2.client_connected)
        self.assertFalse(self.pair2.connected)
        await asyncio.wait_for(self.pair2.start_task, timeout=STD_TIMEOUT)
        self.assertNotEqual(self.pair2.server_port, 0)
        self.assertFalse(self.pair2.server_connected)
        self.assertFalse(self.pair2.client_connected)
        self.assertFalse(self.pair2.connected)

        if connect:
            print("connect pair1 client to pair2 server")
            task1 = asyncio.create_task(
                self.pair1.connect(port=self.pair2.server_port),
            )
            await asyncio.wait_for(
                self.pair1.client_connected_task, timeout=CONNECT_TIMEOUT
            )
            self.assertFalse(self.pair1.server_connected)
            self.assertTrue(self.pair1.client_connected)
            self.assertFalse(self.pair1.connected)

            print("connect pair2 client to pair1 server")
            await asyncio.wait_for(self.pair2.connect(), timeout=CONNECT_TIMEOUT)
            self.assertTrue(task1.done())
            print("all connected")
            self.assertTrue(self.pair1.server_connected)
            self.assertTrue(self.pair1.client_connected)
            self.assertTrue(self.pair1.connected)
            self.assertTrue(self.pair2.server_connected)
            self.assertTrue(self.pair2.client_connected)
            self.assertTrue(self.pair2.connected)
            if use_connect_callback:
                self.assertEqual(len(self.connect_callback_data), 4)
            else:
                self.assertEqual(self.connect_callback_data, [])
        try:
            yield
        finally:
            await asyncio.gather(self.pair1.close(), self.pair2.close())

    def connect_callback(self, client_server):
        print(
            f"ClientServer({client_server.name}): "
            f"client connected: {client_server.client_connected}; "
            f"server connected: {client_server.server_connected}"
        )
        self.connect_callback_data.append((client_server.name, client_server.connected))

    async def test_basic_communication(self):
        for use_connect_callback in (False, True):
            with self.subTest(use_connect_callback=use_connect_callback):
                async with self.make_pairs(use_connect_callback=use_connect_callback):
                    await self.check_basic_communication(
                        reader=self.pair1.client_reader, writer=self.pair2.server_writer
                    )
                    await self.check_basic_communication(
                        reader=self.pair1.server_reader, writer=self.pair2.client_writer
                    )
                    await self.check_basic_communication(
                        reader=self.pair2.client_reader, writer=self.pair1.server_writer
                    )
                    await self.check_basic_communication(
                        reader=self.pair2.server_reader, writer=self.pair1.client_writer
                    )

    async def check_basic_communication(self, reader, writer):
        """Check that we can write string data to write_pair client
        and read it from read_pair server.
        """
        eol_bytes = b"\r\n"
        for data in (
            "a string",
            r"a\nstring\inwith\nnewlines",
            r"a long string " * 500,
        ):
            writer.write(data.encode())
            writer.write(eol_bytes)
            await writer.drain()
            read_bytes = await reader.readuntil(eol_bytes)
            read_data = read_bytes.decode()
            # Use [:-2] on the read data to strip the trailing eol string.
            self.assertEqual(read_data[:-2], data)


if __name__ == "__main__":
    unittest.main()
