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


LOCAL_HOST = "127.0.0.1"

# Engineering user interface (EUI) TCP/IP default configuration
EUI_CONNECTION_PORT = 60006
EUI_LISTENING_PORT = 60005
EUI_HOST = "127.0.0.1"  # FIXME

# Handheld device (HHD) TCP/IP default configuration
HHD_CONNECTION_PORT = 40006
HHD_LISTENING_PORT = 40005
HHD_HOST = "192.168.0.1"

# PXI low-level controller TCP/IP default configuration
PXI_CONNECTION_PORT = 50006
PXI_LISTENING_PORT = 50005
PXI_HOST = "127.0.0.1"  # FIXME

MIRROR_COVER_DRIVES = (0, 1, 2, 3)
