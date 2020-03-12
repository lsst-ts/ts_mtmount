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

__all__ = ["MockController"]

import asyncio

from lsst.ts import salobj
from . import commands
from . import replies
from . import communicator


class MockController:
    """Simulate the most basic responses from the Operation Manager.

    Acknowledge all commands and mark as done.
    Also output a few other replies, to exercise the code.

    Parameters
    ----------
    reply_port : `int`
        Port for writing replies (that the CSC reads).
        The command port is one greater than the reply port.
    log : `logging.Logger`
        Logger.
    """

    def __init__(self, reply_port, log):
        self.log = log.getChild("MockController")
        self.communicator = communicator.Communicator(
            name="MockController",
            client_host=salobj.LOCAL_HOST,
            client_port=reply_port,
            server_host=salobj.LOCAL_HOST,
            server_port=reply_port + 1,
            log=self.log,
            read_replies=False,
            connect_client=False,
            connect_callback=self.connect_callback,
        )
        # Queue of commands, for unit testing
        self.command_queue = None
        self.read_loop_task = asyncio.Future()
        self.start_task = asyncio.create_task(self.start())

    def set_command_queue(self, maxsize=0):
        """Create the command queue.

        This sets attribute ``self.command_queue`` to an `asyncio.Queue`
        and uses it to record commands as they are received.

        Parameters
        ----------
        maxsize : `int` (optional)
            Maximum number of items on the queue.
            If <= 0 then no limit.
            If the queue gets full then the newest items are dropped.
        """
        self.command_queue = asyncio.Queue(maxsize=maxsize)

    def delete_command_queue(self):
        """Delete the command queue, if present.

        This sets ``self.command_queue=None``.
        """
        self.command_code = None

    async def start(self):
        self.log.debug("start: connecting")
        await self.communicator.connect()
        self.log.debug("start: connected")
        self.read_loop_task = asyncio.create_task(self.read_loop())

    async def disconnect(self):
        await self.communicator.disconnect()

    async def close(self):
        self.read_loop_task.cancel()
        await self.communicator.close()

    async def read_loop(self):
        self.log.debug("read_loop begins")
        try:
            while self.communicator.connected:
                command = await self.communicator.read()
                if self.command_queue and not self.command_queue.full():
                    self.command_queue.put_nowait(command)
                asyncio.create_task(self.reply_to_command(command))
        except asyncio.CancelledError:
            return
        except asyncio.streams.IncompleteReadError:
            self.log.error("Connection lost")
            self.close()
        self.log.debug("read_loop ends")

    async def reply_to_command(self, command):
        try:
            await self.communicator.write(
                replies.AckReply(sequence_id=command.sequence_id, timeout_ms=1000)
            )
            if command.command_code not in commands.AckOnlyCommandCodes:
                await asyncio.sleep(0.1)
                await self.communicator.write(
                    replies.DoneReply(sequence_id=command.sequence_id)
                )
        except Exception:
            self.log.exception(f"reply_to_command({command}) failed")
            raise

    def connect_callback(self, server):
        state_str = "connected to" if server.connected else "disconnected from"
        self.log.info(f"Mock controller {state_str} the CSC")
