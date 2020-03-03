# This file is part of ts_MTMount.
#
# Developed for the LSST Data Management System.
# This product includes software developed by the LSST Project
# (https://www.lsst.org).
# See the COPYRIGHT file at the top-level directory of this distribution
# for details of code ownership.
#
# This program is free software: you can redistribute it and/or modify
# it under the eol_bytess of the GNU General Public License as published by
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
import copy
import contextlib
import logging
import time
import unittest

import asynctest
import astropy.time

# from lsst.ts import hexrotcomm
from lsst.ts import MTMount

# Standard timeout for TCP/IP messages (sec).
STD_TIMEOUT = 0.01

# Time to wait for a connection attempt (sec).
CONNECT_TIMEOUT = 5


class CommunicatorTestCase(asynctest.TestCase):
    """Test `lsst.ts.MTMount.Communicator` by connecting two together.

    Also test `lsst.ts.MTMount.Message`.
    """

    async def setUp(self):
        self.log = logging.getLogger()
        self.log.setLevel(logging.INFO)
        self.log.addHandler(logging.StreamHandler())

        # List of (server name, is connected). A new item is appended
        # each time self.server_connect_callback is called.
        self.server_connect_callback_data = []

    @contextlib.asynccontextmanager
    async def make_communicators(
        self, connect_clients=True, use_server_connect_callback=True
    ):
        r"""Make two `lsst.ts.MTMount.Communicator`\ s, ``self.comm1``
        and ``self.comm2``, that talk to each other.

        Parameters
        ----------
        connect_clients : `bool` (optional)
            Connect the clients to the servers?
        use_server_connect_callback : `bool` (optional)
            Specify a server_connect_callback?
            If True then use self.server_connect_callback.
        """
        server_connect_callback = (
            self.server_connect_callback if use_server_connect_callback else None
        )

        self.comm1 = MTMount.Communicator(
            name="comm1",
            client_host=None,
            client_port=0,  # Not yet known
            server_host=MTMount.LOCAL_HOST,
            server_port=0,  # Pick random
            log=self.log,
            connect_client=False,
            server_connect_callback=server_connect_callback,
        )
        self.assertFalse(self.comm1.server_connected)
        self.assertFalse(self.comm1.client_connected)
        self.assertFalse(self.comm1.connected)
        # Wait for comm1 server to start, so its port is assigned
        await self.comm1.wait_server_port()
        self.assertNotEqual(self.comm1.server_port, 0)
        self.assertFalse(self.comm1.server_connected)
        self.assertFalse(self.comm1.client_connected)
        self.assertFalse(self.comm1.connected)

        self.comm2 = MTMount.Communicator(
            name="comm2",
            client_host=None,
            client_port=self.comm1.server_port,
            server_host=MTMount.LOCAL_HOST,
            server_port=0,
            log=self.log,
            connect_client=False,
            server_connect_callback=server_connect_callback,
        )
        # Wait for comm2 server to start
        self.assertFalse(self.comm2.server_connected)
        self.assertFalse(self.comm2.client_connected)
        self.assertFalse(self.comm2.connected)
        await self.comm2.wait_server_port()
        self.assertNotEqual(self.comm2.server_port, 0)
        self.assertFalse(self.comm2.server_connected)
        self.assertFalse(self.comm2.client_connected)
        self.assertFalse(self.comm2.connected)

        if connect_clients:
            print("connect comm1 client to comm2 server")
            await asyncio.wait_for(
                self.comm1.connect(port=self.comm2.server_port), timeout=CONNECT_TIMEOUT
            )
            self.assertFalse(self.comm1.server_connected)
            self.assertTrue(self.comm1.client_connected)
            self.assertFalse(self.comm1.connected)

            print("connect comm2 client to comm1 server")
            await asyncio.wait_for(self.comm2.connect(), timeout=CONNECT_TIMEOUT)
            print("all connected")
            self.assertTrue(self.comm1.server_connected)
            self.assertTrue(self.comm1.client_connected)
            self.assertTrue(self.comm1.connected)
            self.assertTrue(self.comm2.server_connected)
            self.assertTrue(self.comm2.client_connected)
            self.assertTrue(self.comm2.connected)
            if use_server_connect_callback:
                self.assertEqual(len(self.server_connect_callback_data), 2)
            else:
                self.assertEqual(self.server_connect_callback_data, [])
        try:
            yield
        finally:
            await asyncio.gather(self.comm1.close(), self.comm2.close())

    def server_connect_callback(self, server):
        self.server_connect_callback_data.append((server.name, server.connected))

    async def test_basic_communication(self):
        for use_server_connect_callback in (False, True):
            with self.subTest(use_server_connect_callback=use_server_connect_callback):
                async with self.make_communicators(
                    use_server_connect_callback=use_server_connect_callback
                ):
                    await self.check_basic_communication(
                        reader=self.comm1, writer=self.comm2
                    )
                    await self.check_basic_communication(
                        reader=self.comm2, writer=self.comm1
                    )

    async def check_basic_communication(self, reader, writer):
        """Check that we can write messages to a writer
        and read them from a reader.

        Parameters
        ----------
        reader : `lsst.ts.MTMount.Communicator`
            Communicator to read from.
        writer : `lsst.ts.MTMount.Communicator`
            Communicator to write to.
        """
        for parameters in (
            (),
            ("param1",),
            ("param1", "param2", "param3"),
            ("many parameters",) * 500,
        ):
            message = MTMount.Message(
                code=1,
                sequence_id=2,
                origin_id=3,
                source=4,
                timestamp=astropy.time.Time.now(),
                parameters=parameters,
            )
            await writer.write(message)
            read_message = await reader.read()
            self.assertEqual(message, read_message)

    async def test_message_constructor_invalid(self):
        good_kwargs = dict(
            code=1,
            sequence_id=2,
            origin_id=3,
            source=4,
            timestamp=astropy.time.Time.now(),
            parameters=("foo", "bar"),
        )
        message = MTMount.Message(**good_kwargs)
        self.assertEqual(message.code, good_kwargs["code"])
        self.assertEqual(message.sequence_id, good_kwargs["sequence_id"])
        self.assertEqual(message.origin_id, good_kwargs["origin_id"])
        self.assertEqual(message.timestamp, good_kwargs["timestamp"])
        self.assertEqual(message.parameters, good_kwargs["parameters"])

        for param_name in ("code", "sequence_id", "origin_id", "source"):
            with self.subTest(param_name=param_name):
                bad_kwargs = copy.copy(good_kwargs)
                bad_kwargs[param_name] = "not an integer"
                with self.assertRaises(ValueError):
                    MTMount.Message(**bad_kwargs)

                bad_kwargs[param_name] = 53.2  # Not an integer
                with self.assertRaises(ValueError):
                    MTMount.Message(**bad_kwargs)

        for bad_timestamp in (time.time(), 12, 7.6, "2010:05:06T12:00:00"):
            with self.subTest(bad_timestamp=bad_timestamp):
                bad_kwargs = copy.copy(good_kwargs)
                bad_kwargs["timestamp"] = bad_timestamp
                with self.assertRaises(ValueError):
                    MTMount.Message(**bad_kwargs)

        for bad_parameters in ("not a tuple or list", 1, 1.3, None):
            with self.subTest(bad_parameters=bad_parameters):
                bad_kwargs = copy.copy(good_kwargs)
                bad_kwargs["parameters"] = bad_parameters
                with self.assertRaises(ValueError):
                    MTMount.Message(**bad_kwargs)


if __name__ == "__main__":
    unittest.main()
