# This file is part of ts_MTMount.
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

__all__ = ["CommandFutures"]

import asyncio

from lsst.ts import salobj


class CommandFutures:
    """asyncio futures to track the progress of a low-level controller command.

    Attributes
    ----------
    ack : `asyncio.Future`
        Future which ends as follows:

        * result = timeout (in sec) when the command is acknowledged
          (an Ack reply is read).
        * exception = `lsst.ts.salobj.ExpectedError` if the command fails
          (a NoAck reply is read) before the command is acknowledged.
    done : `asyncio.Future`
        Future which ends as follows:

        * result = `None` when the command finishes successfully
          (a Done reply is read).
        * exception = `lsst.ts.salobj.ExpectedError` if the command fails.
    """

    def __init__(self):
        self.ack = asyncio.Future()
        self.done = asyncio.Future()

    def setack(self, timeout):
        """Report a command as started.

        Parameters
        ----------
        timeout : `float`
            Max time for command to complete (sec).
        """
        if not self.ack.done():
            self.ack.set_result(timeout)

    def setnoack(self, explanation):
        """Report a command as failed.

        Parameters
        ----------
        explanation : `str`
            Explanation of what went wrong.

        Notes
        -----
        Sets the ``ack`` (if not done) else ``done`` future to
        ``lsst.ts.salobj.ExpectedError`` exception.

        Do not set both, to avoid "exception was never retrieved" warnings.
        """
        if not self.ack.done():
            self.ack.set_exception(salobj.ExpectedError(explanation))
        elif not self.done.done():
            self.done.set_exception(salobj.ExpectedError(explanation))

    def setdone(self):
        """Report a command as done."""
        if not self.ack.done():
            self.ack.set_result(0)
        if not self.done.done():
            self.done.set_result(None)

    @property
    def timeout(self):
        """Return the timeout, in seconds.

        Return None if command not acknowledged.
        Raise an exception if the command failed before being acknowledged.

        Raises
        ------
        InvalidStateError
            If ``ack`` is not done.
        CancelledError
            If ``ack`` was cancelled.
        The exception set by `asyncio.Future.set_exception`
            If ``ack`` was set to an exception.
        """
        return self.ack.result()
