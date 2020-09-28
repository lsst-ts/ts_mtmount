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
    The client only writes data and the server only reads data.
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
    connect : `bool`
        Connect the client at construction time?
        (The server will automatically wait for connections.)
    connect_callback : callable or `None`
        Synchronous function to call when a connection is made or dropped.
        It receives one argument: this ClientServerPair.

    Attributes
    ----------
    client_host : `str`
        Client host name.
    client_port : `int`
        Client port.
    client_writer : `asyncio.StreamWriter` or `None`
      Client socket writer, or `None` if not connected.
    client_reader : `asyncio.StreamReader` or `None`
        Client socket reader,  or `None` if not connected.
    client_connected_task : `asyncio.Task`
        An `asyncio.Future` that is set done when the client is connected.
    connect_task : `asyncio.Task`
        An `asyncio.Task` that is set done when connected.
        This attribute is only available if ``connect`` true.
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
        connect=True,
        connect_callback=None,
    ):
        self.name = name
        self.client_host = client_host
        self.client_port = client_port
        self.client_reader = None
        self.client_writer = None
        self.log = log.getChild(f"ClientServerPair({self.name})")
        self.connect_callback = connect_callback
        self.client_connected_task = asyncio.Future()
        self._server = hexrotcomm.OneClientServer(
            name=name,
            host=server_host,
            port=server_port,
            log=log,
            connect_callback=self.call_connect_callback,
        )
        if connect:
            self.connect_task = asyncio.create_task(self.connect())
        self.log.debug(
            "Constructed with "
            f"client_port={self.client_port}; "
            f"client_host={self.client_host}; "
            f"server_port={self.server_port}; "
            f"server_host={self.server_host}; "
        )

    @property
    def client_connected(self):
        """Is the client connected?"""
        return not (
            self.client_writer is None
            or self.client_writer.is_closing()
            or self.client_reader.at_eof()
        )

    @property
    def connected(self):
        """Are both the client and server connected?"""
        return self.client_connected and self.server_connected

    @property
    def server_connected(self):
        """Is the server connected?"""
        return self._server.connected

    @property
    def server_host(self):
        return self._server.host

    @property
    def server_port(self):
        return self._server.port

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

    def call_connect_callback(self, *args, **kwargs):
        """Call the connect_callback if it exists. Any arguments are ignored.
        """
        if self.connect_callback is None:
            return
        try:
            self.connect_callback(self)
        except Exception:
            self.log.exception(f"Connect callback {self.connect_callback} failed")

    async def close(self):
        """Close both the server and the client, to clean up when finished.

        Set the server done_task done.
        """
        await self.close_client()  # Does not call connect_callback.
        await self._server.close()  # Does call connect_callback.

    async def close_client(self):
        """Close the client.

        Warning: does NOT call the connect_callback.
        """
        if self.client_writer is not None and not self.client_writer.is_closing():
            self.client_writer.close()

    async def connect(self, port=None):
        """Connect the client socket and wait for a connection to the server.

        Parameters
        ----------
        port : `int` or `None`, optional
            TCP/IP port. If `None` use the port specified in the constructor.
            Being able to specify a port here is primarily for unit tests.

        Notes
        -----
        This will wait forever for a connection.
        """
        await self.close_client()
        if port is not None:
            self.client_port = port
        if not self.client_port:
            raise ValueError(f"client_port={self.client_port!r} must be nonzero")

        self.log.debug("ClientServerPair waiting for client to connect")
        while True:
            try:
                self.log.info(
                    f"Connect to host={self.client_host}, port={self.client_port}"
                )
                self.client_reader, self.client_writer = await asyncio.open_connection(
                    host=self.client_host, port=self.client_port
                )
                self.call_connect_callback()
                break
            except Exception as e:
                self.log.debug(f"Connect failed; retrying. Error: {e}")
                await asyncio.sleep(self.connect_retry_interval)
        self.client_connected_task.set_result(None)
        await self._server.connected_task

    async def wait_server_port(self):
        """Wait for the server to start, then return the port.

        Useful when you have specified port=0, e.g. for unit tests.
        """
        await self._server.start_task
        return self.server_port
