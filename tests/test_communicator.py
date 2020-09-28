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


class CommunicatorTestCase(asynctest.TestCase):
    """Test `lsst.ts.MTMount.Communicator` by connecting two together.

    Also test `lsst.ts.MTMount.Message`.
    """

    async def setUp(self):
        self.log = logging.getLogger()
        self.log.setLevel(logging.INFO)
        self.log.addHandler(logging.StreamHandler())

        # List of (server name, is connected). A new item is appended
        # each time self.connect_callback is called.
        self.connect_callback_data = []

    @contextlib.asynccontextmanager
    async def make_communicators(self, connect_clients=True, use_connect_callback=True):
        r"""Make two `lsst.ts.MTMount.Communicator`\ s, ``self.comm1``
        and ``self.comm2``, that talk to each other.
        ``comm1`` sends commands and ``comm2`` sends replies.

        Parameters
        ----------
        connect_clients : `bool`, optional
            Connect the clients to the servers?
        use_connect_callback : `bool`, optional
            Specify a connect_callback?
            If True then use self.connect_callback.
        """
        connect_callback = self.connect_callback if use_connect_callback else None

        self.comm1 = MTMount.Communicator(
            name="comm1",
            client_host=None,
            client_port=0,  # Not yet known
            server_host=salobj.LOCAL_HOST,
            server_port=0,  # Pick random
            log=self.log,
            read_replies=True,
            connect=False,
            connect_callback=connect_callback,
        )
        self.assertFalse(self.comm1.server_connected)
        self.assertFalse(self.comm1.client_connected)
        self.assertFalse(self.comm1.connected)
        # Wait for comm1 server to start, so its port is assigned
        await asyncio.wait_for(self.comm1.wait_server_port(), timeout=CONNECT_TIMEOUT)
        self.assertNotEqual(self.comm1.server_port, 0)
        self.assertFalse(self.comm1.server_connected)
        self.assertFalse(self.comm1.client_connected)
        self.assertFalse(self.comm1.connected)

        self.comm2 = MTMount.Communicator(
            name="comm2",
            client_host=None,
            client_port=self.comm1.server_port,
            server_host=salobj.LOCAL_HOST,
            server_port=0,
            log=self.log,
            read_replies=False,
            connect=False,
            connect_callback=connect_callback,
        )
        # Wait for comm2 server to start
        self.assertFalse(self.comm2.server_connected)
        self.assertFalse(self.comm2.client_connected)
        self.assertFalse(self.comm2.connected)
        await asyncio.wait_for(self.comm2.wait_server_port(), timeout=CONNECT_TIMEOUT)
        self.assertNotEqual(self.comm2.server_port, 0)
        self.assertFalse(self.comm2.server_connected)
        self.assertFalse(self.comm2.client_connected)
        self.assertFalse(self.comm2.connected)

        if connect_clients:
            print("connect comm1 client to comm2 server")
            task1 = asyncio.create_task(
                self.comm1.connect(port=self.comm2.server_port),
            )
            await asyncio.wait_for(
                self.comm1.client_connected_task, timeout=CONNECT_TIMEOUT
            )
            self.assertFalse(self.comm1.server_connected)
            self.assertTrue(self.comm1.client_connected)
            self.assertFalse(self.comm1.connected)

            print("connect comm2 client to comm1 server")
            await asyncio.wait_for(self.comm2.connect(), timeout=CONNECT_TIMEOUT)
            self.assertTrue(task1.done())
            print("all connected")
            self.assertTrue(self.comm1.server_connected)
            self.assertTrue(self.comm1.client_connected)
            self.assertTrue(self.comm1.connected)
            self.assertTrue(self.comm2.server_connected)
            self.assertTrue(self.comm2.client_connected)
            self.assertTrue(self.comm2.connected)
            if use_connect_callback:
                self.assertEqual(len(self.connect_callback_data), 4)
            else:
                self.assertEqual(self.connect_callback_data, [])
        try:
            yield
        finally:
            await asyncio.gather(self.comm1.close(), self.comm2.close())

    def connect_callback(self, communicator):
        print(
            f"Communicator({communicator.name}): "
            f"client connected: {communicator.client_connected}; "
            f"server connected: {communicator.server_connected}"
        )
        self.connect_callback_data.append((communicator.name, communicator.connected))

    async def test_basic_communication(self):
        tai = 1589372384.123
        # A representative sampling of commands, including
        # bool, int, float, and time fields.
        commands = (
            MTMount.commands.ElevationAxisTrack(
                sequence_id=1, position=12, velocity=0.3, tai=tai
            ),
            MTMount.commands.AzimuthAxisTrack(
                sequence_id=2, position=12, velocity=0.3, tai=tai
            ),
            MTMount.commands.MirrorCoversPower(sequence_id=3, drive=2, on=True),
        )
        replies = (
            MTMount.replies.AckReply(sequence_id=1, timeout_ms=3500),
            MTMount.replies.DoneReply(sequence_id=2),
            MTMount.replies.ErrorReply(
                on=True,
                active=False,
                code=47,
                subsystem=f"{MTMount.SubsystemId.MIRROR_COVERS}. MyTopVI/MyNextVI/MyNextNextVI",
                what="test error",
            ),
            MTMount.replies.InPositionReply(what=1, in_position=True),
        )

        for use_connect_callback in (False, True):
            with self.subTest(use_connect_callback=use_connect_callback):
                async with self.make_communicators(
                    use_connect_callback=use_connect_callback
                ):
                    await self.check_basic_communication(
                        reader=self.comm1, writer=self.comm2, messages=replies
                    )
                    await self.check_basic_communication(
                        reader=self.comm2, writer=self.comm1, messages=commands
                    )

    async def check_basic_communication(self, reader, writer, messages):
        """Check that we can write messages to a writer
        and read them from a reader.

        Parameters
        ----------
        reader : `lsst.ts.MTMount.Communicator`
            Communicator to read from.
        writer : `lsst.ts.MTMount.Communicator`
            Communicator to write to.
        messages : `List` [`lsst.ts.MTMount.BaseMessage`]
            Messages to send.
            If the writer is ``self.comm1`` then these must be commands.
            If the writer is ``self.comm2`` then these must be replies.
        """
        for message in messages:
            await writer.write(message)
            read_message = await reader.read()
            self.assertEqual(message, read_message)


if __name__ == "__main__":
    unittest.main()
