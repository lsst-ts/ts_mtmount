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
    "CSC_COMMAND_PORT",
    "HHD_COMMAND_PORT",
    "TELEMETRY_PORT",
    "MIRROR_COVER_DRIVES",
    "LINE_TERMINATOR",
    "AxisStateDict",
    "DriveStateDict",
]
from lsst.ts.idl.enums.MTMount import AxisState, DriveState

CSC_COMMAND_PORT = 30005
HHD_COMMAND_PORT = 40005

TELEMETRY_PORT = 50035

# We probably don't need this, because -1 means "all drives".
MIRROR_COVER_DRIVES = (0, 1, 2, 3)

# TCP/IP line terminator (bytes)
LINE_TERMINATOR = b"\r\n"

# Axis state from the low-level controller: AxisState enum value.
AxisStateDict = {
    "Unknown": AxisState.UNKNOWN,
    "Idle": AxisState.OFF,
    "On Enable": AxisState.ENABLED,
    "On DiscreteMove": AxisState.DISCRETE_MOVE,
    "On JogMove": AxisState.JOG_MOVE,
    "On Tracking": AxisState.TRACKING,
    "On Stopping": AxisState.STOPPING,
    "Fault": AxisState.FAULT,
}

# Drive state from the low-level controller: DriveState enum value.
DriveStateDict = {
    "Unknown": DriveState.UNKNOWN,
    "Off": DriveState.OFF,
    "Standstill": DriveState.STOPPED,
    "Discrete Motion": DriveState.MOVING,
    "Stopping": DriveState.STOPPING,
    "Fault": DriveState.FAULT,
}
