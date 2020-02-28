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

__all__ = ["Communicator"]

from . import client_server_pair
from . import commands
from . import replies


class Communicator(client_server_pair.ClientServerPair):
    r"""Read and write `BaseMessage`\ s using Tekniker's
    communication protocol.

    Parameters
    ----------
    name : `str`
        Name used for error messages.
    client_host : `str`
        IP address for the output client socket.
    client_port : `int`
        IP port for the output client socket.
    server_host : `str` or `None`
        IP address for the input socket server.
        If `None` then use all interfaces.
    server_port : `int`
        IP port for the input socket server.
        If 0 then use a random port.
    log : `logging.Logger`
        Logger.
    read_replies : `bool`
        If True then read replies, else read commands.
    connect_client : `bool` (optional)
        Connect the client at construction time?
    server_connect_callback : callable (optional)
        Function to call when a connection is made to the socket server.

    Notes
    -----
    Tekniker's OperationManager software connects to each component
    (PXI, EUI and HHD) using two TCP/IP sockets:

    * A client socket for output; data is only written to this socket.
        For the PXI this is used to send commands.
        For EUI and HDD this is used to send replies.
    * A server socket for input; data is only read from this socket.
        For the PXI this is used to read replies.
        For EUI and HDD this is used to read commands.
    """

    def __init__(
        self,
        name,
        client_host,
        client_port,
        server_host,
        server_port,
        log,
        read_replies,
        connect_client=True,
        server_connect_callback=None,
    ):
        super().__init__(
            name=name,
            client_host=client_host,
            client_port=client_port,
            server_host=server_host,
            server_port=server_port,
            log=log,
            connect_client=connect_client,
            server_connect_callback=server_connect_callback,
        )
        self.eol_bytes = "\r\n".encode()
        if read_replies:
            self.parse_read_fields = replies.parse_reply
        else:
            self.parse_read_fields = commands.parse_command

    async def write(self, message):
        """Write a message.

        Parameters
        ----------
        message : `BaseMessage`
            Message to write.
        """
        self.client_writer.write(message.encode())
        await self.client_writer.drain()

    async def read(self):
        """Read and return a message. Waits indefinitely.

        Returns
        -------
        message : `BaseMessage`
            The message read.
        """
        read_bytes = await self.server_reader.readuntil(self.eol_bytes)
        read_str = read_bytes.decode()[:-2]
        fields = read_str.split("\n")
        return self.parse_read_fields(fields)
