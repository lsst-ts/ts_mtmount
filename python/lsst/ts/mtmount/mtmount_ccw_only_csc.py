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

__all__ = ["MTMountCcwOnlyCsc", "run_mtmount_ccw_only"]

import asyncio

from .mtmount_csc import MTMountCsc


class MTMountCcwOnlyCsc(MTMountCsc):
    """MTMount CSC commanding only the CCW.

    See `MTMountCsc`.
    """

    async def do_clearError(self, data):
        """Handle the clearError command.

        Raises
        ------
        NotImplementedError
            This CSC is for testing the CCW with the Rotator only. All MTMount
            methods have been disabled.
        """
        raise NotImplementedError()

    async def do_closeMirrorCovers(self, data):
        """Handle the closeMirrorCovers command.

        Raises
        ------
        NotImplementedError
            This CSC is for testing the CCW with the Rotator only. All MTMount
            methods have been disabled.
        """
        raise NotImplementedError()

    async def do_homeBothAxes(self, data):
        """Handle the homeBothAxes command.

        Raises
        ------
        NotImplementedError
            This CSC is for testing the CCW with the Rotator only. All MTMount
            methods have been disabled.
        """
        raise NotImplementedError()

    async def do_openMirrorCovers(self, data):
        """Handle the openMirrorCovers command.

        Raises
        ------
        NotImplementedError
            This CSC is for testing the CCW with the Rotator only. All MTMount
            methods have been disabled.
        """
        raise NotImplementedError()

    async def do_moveToTarget(self, data):
        """Handle the moveToTarget command.

        Raises
        ------
        NotImplementedError
            This CSC is for testing the CCW with the Rotator only. All MTMount
            methods have been disabled.
        """
        raise NotImplementedError()

    async def do_setThermal(self, data):
        """Handle the setThermal command.

        Raises
        ------
        NotImplementedError
            This CSC is for testing the CCW with the Rotator only. All MTMount
            methods have been disabled.
        """
        raise NotImplementedError()

    async def do_trackTarget(self, data):
        """Handle the trackTarget command.

        Raises
        ------
        NotImplementedError
            This CSC is for testing the CCW with the Rotator only. All MTMount
            methods have been disabled.
        """
        raise NotImplementedError()

    async def do_startTracking(self, data):
        """Handle the startTracking command.

        Raises
        ------
        NotImplementedError
            This CSC is for testing the CCW with the Rotator only. All MTMount
            methods have been disabled.
        """
        raise NotImplementedError()

    async def do_stop(self, data):
        """Handle the stop command.

        Raises
        ------
        NotImplementedError
            This CSC is for testing the CCW with the Rotator only. All MTMount
            methods have been disabled.
        """
        raise NotImplementedError()

    async def do_stopTracking(self, data):
        """Handle the stopTracking command.

        Raises
        ------
        NotImplementedError
            This CSC is for testing the CCW with the Rotator only. All MTMount
            methods have been disabled.
        """
        raise NotImplementedError()


def run_mtmount_ccw_only() -> None:
    """Run the MTMountCcwOnly CSC."""
    asyncio.run(MTMountCcwOnlyCsc.amain(index=None))
