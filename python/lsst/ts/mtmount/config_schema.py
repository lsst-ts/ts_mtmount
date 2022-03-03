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

__all__ = ["CONFIG_SCHEMA"]

import yaml

CONFIG_SCHEMA = yaml.safe_load(
    """
$schema: http://json-schema.org/draft-07/schema#
$id: https://github.com/lsst-ts/ts_mtmount/blob/master/python/lsst/ts/mtmount/config_schema.py
# title must end with one or more spaces followed by the schema version, which must begin with "v"
title: MTMount v2
description: Schema for MTMount configuration files
type: object
properties:
  host:
    description: IP address of the Operation Manager.
    type: string
    format: hostname
  telemetry_host:
    description: IP address of the telemetry server.
    type: string
    format: hostname
  connection_timeout:
    description: Time limit for connecting to the TCP/IP command interface (sec)
    type: number
    exclusiveMinimum: 0
  ack_timeout:
    description: Time limit for reading a command acknowledgement from the TCP/IP interface (sec)
    type: number
    exclusiveMinimum: 0
  camera_cable_wrap_advance_time:
    description: >-
      How far in advance of the current time to make the tai time field of camera cable wrap
      tracking commands (sec). All tracking commands must sent to the low-level controller in advance,
      so it can interpolate between pairs of tracking command. If a tracking command is late then
      the controller will halt the axis. Set this value large enough that camera cable wrap
      commands reliably arrive in time, despite variation in speed of the Python CSC.
      Avoid making it much larger than required, because it introduces a delay in motion
      of the camera cable wrap, which can cause too large an error between the camera rotator
      and the camera cable wrap when the rotator is offset or slewed to a new field.
    type: number
  camera_cable_wrap_interval:
    description: >-
      Interval between camera cable wrap tracking commands (seconds).
      The actual time between commans will be this interval
      plus the time it takes to compute and issue the tracking command.
    type: number
    minimum: 0
  max_rotator_position_error:
    description: >-
      The maximum difference (in degrees) between camera rotator actual position and demand position
      beyond which the camera cable wrap will follow the camera rotator actual position and velocity,
      rather than the usual demand position and velocity.
    type: number
    exclusiveMinimum: 0
required:
  - host
  - connection_timeout
  - ack_timeout
  - camera_cable_wrap_advance_time
  - max_rotator_position_error
additionalProperties: false
"""
)
