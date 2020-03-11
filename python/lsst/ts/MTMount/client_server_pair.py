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

__all__ = ["ClientServerPair"]

import asyncio

from lsst.ts import hexrotcomm


class ClientServerPair:
    """A TCP/IP client and server combined.

    This class exists because Tekniker's OperationManager software connects
    to each component using a client and server pair of sockets.
    See `Communicator` for a higher level abstraction.

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
    connect_client : `bool`
        Connect the client at construction time?
    server_connect_callback : callable or `None`
        Function to call when a connection is made to the socket server.
        It receives one argument: the server, as an
        `lsst.ts.hexrotcomm.OneClientServer`.

    Notes
    -----
    **Attributes**

    * ``client_host``: client host name, a `str`.
    * ``client_port``:  client port, an `int`.
    * ``client_writer``: client socket writer, an `asyncio.StreamWriter`,
      or `None` if not connected.
    * ``client_reader``: client socket reader, an `asyncio.StreamReader`
      or `None` if not connected.
    """

    connect_retry_interval = 0.1
    """Interval between client connection retries (sec)."""

    def __init__(
        self,
        name,
        client_host,
        client_port,
        server_host,
        server_port,
        log,
        connect_client=True,
        server_connect_callback=None,
    ):
        self.name = name
        self.client_host = client_host
        self.client_port = client_port
        self.client_reader = None
        self.client_writer = None
        self.log = log
        self._server_connect_callback = server_connect_callback
        self._server = hexrotcomm.OneClientServer(
            name=name,
            host=server_host,
            port=server_port,
            log=log,
            connect_callback=server_connect_callback,
        )
        if connect_client:
            self.connect_task = asyncio.create_task(self.connect())
        else:
            self.connect_task = asyncio.Future()

    @property
    def client_connected(self):
        """Is the client connected?"""
        return not (
            self.client_writer is None
            or self.client_writer.is_closing()
            or self.client_reader.at_eof()
        )

    @property
    def server_reader(self):
        """Server reader, an `asyncio.StreamReader` or `None` if not connected.
        """
        return self._server.reader

    @property
    def server_writer(self):
        """Server writer, an `asyncio.StreamWriter` or `None` if not connected.
        """
        return self._server.writer

    @property
    def server_port(self):
        return self._server.port

    async def wait_server_port(self):
        """Wait for the server to start, then return the port.

        Useful when you have specified port=0, e.g. for unit tests.
        """
        await self._server.start_task
        return self.server_port

    @property
    def server_host(self):
        return self._server.host

    @property
    def server_connected(self):
        """Is the server connected?"""
        return self._server.connected

    @property
    def connected(self):
        """Are both the client and server connected?"""
        return self.client_connected and self.server_connected

    async def close(self):
        """Close both the server and the client."""
        await self._server.close()
        await self.close_client()

    async def close_client(self):
        """Close the client."""
        if self.client_writer is not None and not self.client_writer.is_closing():
            self.client_writer.close()

    async def connect(self, port=None):
        """Connect the client socket and wait for a connection to the server.

        Parameters
        ----------
        port : `int` or `None` (optional)
            TCP/IP port. If `None` use the port specified in the constructor.
            Being able to specify a port here is primarily for unit tests.

        Notes
        -----
        This will wait forever for a connection.
        """
        if self.client_writer is not None and not self.client_writer.is_closing():
            self.client_writer.close()
        if port is not None:
            self.client_port = port
        if not self.client_port:
            raise ValueError(f"client_port={self.client_port!r} must be nonzero")

        while True:
            try:
                self.log.debug(
                    f"connect: connect to host={self.client_host}, port={self.client_port}"
                )
                self.client_reader, self.client_writer = await asyncio.open_connection(
                    host=self.client_host, port=self.client_port
                )
                return
            except Exception as e:
                self.log.warning(f"connect failed with {e}; retrying")
                await asyncio.sleep(self.connect_retry_interval)
        await self._server.connect_task
