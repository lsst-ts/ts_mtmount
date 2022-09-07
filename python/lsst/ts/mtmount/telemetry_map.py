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

__all__ = ["TELEMETRY_MAP"]

import yaml

# Mapping to translate telemetry from the low-level controller
# to SAL telemetry topics.

# The structure is:
# low-level controller topic ID: [SAL topic name, field translation dict]
# where field translation dict is a dict of
# SAL field name: low-level controller field name.
# These IDs must match the entries in the `TelemetryTopicId` enum class.
#
# If the value is a set of scalars in the low-level data and an array in SAL,
# then list it without the indices here, and update TelemetryClient to add
# _preprocess_x method to convert the scalars to an array.
# For example: the ``current`` field for the elevationDrives topic
# is set from id=14 fields ``elCurrent1, elCurrent2, ... elCurrent12``;
# list this as ``current: elCurrent`` in this file and add method
# ``_preprocess_elevationDrives`` to `TelemetryClient` to convert the data.

TELEMETRY_MAP = yaml.safe_load(
    """

# Disable azimuth and elevation telemetry. The hacked-up CSC reports
# "azimuth" and "elevation" telemetry topics and does not bother
# to report "azimuthDrives" and "elevationDrives".
# 6: # fields are in flux
# - azimuth
# - actualPosition: angleActual
#   demandPosition: angleSet
#   actualVelocity: velocityActual
#   demandVelocity: velocitySet
#   actualTorque: torqueActual
#   timestamp: timestamp

# 5:
# - azimuthDrives
# - current: azCurrent
#   timestamp: timestamp

# 15: # fields are in flux
# - elevation
# - actualPosition: angleActual
#   demandPosition: angleSet
#   actualVelocity: velocityActual
#   demandVelocity: velocitySet
#   actualTorque: torqueActual
#   timestamp: timestamp

# 14:
# - elevationDrives
# - current: elCurrent
#   timestamp: timestamp

8:
- cameraCableWrap
- actualPosition: angle
  actualVelocity: speed
  actualTorquePercentage: torquePercentage
  timestamp: timestamp
"""
)
