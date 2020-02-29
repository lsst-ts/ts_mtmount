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

__all__ = [
    "ElevationAxisPower",
    "ElevationAxisStop",
    "ElevationAxisMove",
    "ElevationAxisTracking",
    "AzimuthAxisPower",
    "AzimuthAxisStop",
    "AzimuthAxisMove",
    "AzimuthAxisTracking",
    "MainPowerSupplyPower",
    "MirrorCoverPower",
    "MirrorCoverStop",
    "MirrorCoverOpen",
    "MirrorCoverClose",
    "CameraCableWrapPower",
    "CameraCableWrapStop",
    "CameraCableWrapMove",
    "CameraCableWrapDriveEnable",
    "CameraCableWrapTrackCamera",
    "CameraCableWrapEnableTrackCamera",
    "Commands",
    "CommandDict",
    "parse_command",
]

from .base_message import BaseMessage
from . import enums
from . import field_info


def make_command_field_infos(command_code, parameters=()):
    """Make field_infos for a command.
    """
    for param in parameters:
        if not isinstance(param, field_info.BaseFieldInfo):
            raise ValueError(
                f"parameters={parameters} is not a sequence of field_info.BaseFieldInfo"
            )
    return (
        field_info.IntFieldInfo(
            name="sequence_id",
            doc="Sequence number; used to identify replies to this command.",
        ),
        field_info.IntFieldInfo(
            name="sal_sequence_id",
            doc="SAL command sequence number. Probably not used.",
            default=0,
        ),
        field_info.CommandCodeFieldInfo(command_code),
        field_info.SourceFieldInfo(what="command"),
        field_info.TimestampFieldInfo(),
    ) + tuple(parameters)


_TrackingParameters = (
    field_info.FloatFieldInfo(name="position", doc="Target position (deg) at tai_time"),
    field_info.FloatFieldInfo(
        name="velocity", doc="Target velocity (deg/sec) at tai_time"
    ),
    field_info.TimeFieldInfo(
        name="tai_time", doc="Target TAI time for position and velocity", scale="tai"
    ),
)

_MoveParameters = (
    field_info.FloatFieldInfo(name="position", doc="Target position (deg)."),
    field_info.FloatFieldInfo(name="velocity", doc="Maximum velocity (deg/second)."),
    field_info.FloatFieldInfo(
        name="acceleration", doc="Maximum acceleration (deg/second2)."
    ),
    field_info.FloatFieldInfo(name="jerk", doc="Maximum jerk (deg/second3)."),
)

_PowerParameters = (
    field_info.BoolFieldInfo(name="on", doc="Turn the power on (True) or off (False)"),
)

"""
Supported elevation and azimuth axis commands:

* <AXIS>_AXIS_POWER
* <AXIS>_AXIS_STOP
* <AXIS>_AXIS_MOVE
* <AXIS>_AXIS_TRACKING

Other elevation and azimuth axis commands:

* <AXIS>_AXIS_MOVE_VELOCITY
* <AXIS>_AXIS_HOME
* <AXIS>_AXIS_RESET_ALARM
"""


class ElevationAxisPower(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_AXIS_POWER, _PowerParameters
    )


class ElevationAxisStop(BaseMessage):
    field_infos = make_command_field_infos(enums.CommandCode.ELEVATION_AXIS_STOP)


class ElevationAxisMove(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_AXIS_MOVE, _MoveParameters
    )


class ElevationAxisTracking(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_AXIS_TRACKING, _TrackingParameters
    )


class AzimuthAxisPower(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_AXIS_POWER, _PowerParameters
    )


class AzimuthAxisStop(BaseMessage):
    field_infos = make_command_field_infos(enums.CommandCode.AZIMUTH_AXIS_STOP)


class AzimuthAxisMove(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_AXIS_MOVE, _MoveParameters
    )


class AzimuthAxisTracking(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_AXIS_TRACKING, _TrackingParameters
    )


class MainPowerSupplyPower(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.MAIN_POWER_SUPPLY_POWER, _PowerParameters
    )


"""
Supported mirror cover commands:

* MIRROR_COVER_POWER = 901
* MIRROR_COVER_STOP = 902
* MIRROR_COVER_OPEN = 905
* MIRROR_COVER_CLOSE = 906

Other mirror cover commands:

* MIRROR_COVER_MOVE = 903
* MIRROR_COVER_MOVE_VELOCITY = 904
* MIRROR_COVER_RESET_ALARM = 907
"""


class MirrorCoverPower(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVER_POWER,
        (field_info.IntFieldInfo(name="drive", doc="Drive index: one of 0, 1, 2, 3"),)
        + _PowerParameters,
    )


class MirrorCoverStop(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVER_STOP,
        (field_info.IntFieldInfo(name="drive", doc="Drive index: one of 0, 1, 2, 3"),),
    )


class MirrorCoverOpen(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVER_OPEN,
        (field_info.IntFieldInfo(name="drive", doc="Drive index: one of 0, 1, 2, 3"),),
    )


class MirrorCoverClose(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVER_CLOSE,
        (field_info.IntFieldInfo(name="drive", doc="Drive index: one of 0, 1, 2, 3"),),
    )


"""
Supported camera cable wrap commands:

* CAMERA_CABLE_WRAP_POWER = 1001
* CAMERA_CABLE_WRAP_STOP = 1002
* CAMERA_CABLE_WRAP_MOVE = 1003
* CAMERA_CABLE_WRAP_TRACK_CAMERA = 1004
* CAMERA_CABLE_WRAP_DRIVE_ENABLE = 1006
* CAMERA_CABLE_WRAP_ENABLE_TRACK_CAMERA = 1009

Other camera cable wrap commands:

* CAMERA_CABLE_WRAP_RESET_ALARM = 1005
* CAMERA_CABLE_WRAP_DRIVE_RESET = 1007
* CAMERA_CABLE_WRAP_MOVE_VELOCITY = 1008
"""


class CameraCableWrapPower(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_POWER, _PowerParameters
    )


class CameraCableWrapStop(BaseMessage):
    field_infos = make_command_field_infos(enums.CommandCode.CAMERA_CABLE_WRAP_STOP)


class CameraCableWrapMove(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_MOVE, _MoveParameters
    )


class CameraCableWrapTrackCamera(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_TRACK_CAMERA, _TrackingParameters
    )


class CameraCableWrapDriveEnable(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_DRIVE_ENABLE,
        (field_info.IntFieldInfo(name="drive", doc="Drive index"),) + _PowerParameters,
    )


class CameraCableWrapEnableTrackCamera(BaseMessage):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_ENABLE_TRACK_CAMERA,
        (
            field_info.BoolFieldInfo(
                name="on",
                doc="True to make the camera cable wrap automatically track the camera rotator.",
            ),
        ),
    )


Commands = (
    ElevationAxisPower,
    ElevationAxisStop,
    ElevationAxisMove,
    ElevationAxisTracking,
    AzimuthAxisPower,
    AzimuthAxisStop,
    AzimuthAxisMove,
    AzimuthAxisTracking,
    MainPowerSupplyPower,
    MirrorCoverPower,
    MirrorCoverStop,
    MirrorCoverOpen,
    MirrorCoverClose,
    CameraCableWrapPower,
    CameraCableWrapStop,
    CameraCableWrapMove,
    CameraCableWrapDriveEnable,
    CameraCableWrapTrackCamera,
    CameraCableWrapEnableTrackCamera,
)


def _make_command_dict():
    """Make a dict of command_code: CommandClass from `Commands`.

    Check that no command codes are duplicated.
    """
    command_dict = {}
    for command in Commands:
        command_code = command.field_infos[2].default
        if command_code in command_dict:
            raise RuntimeError(
                "The command code appears twice: "
                f"once in {command_dict[command_code].__name__}; "
                f"and once in {command.__name__}"
            )
        command_dict[command_code] = command
    return command_dict


# Dict of CommandCode: CommandClass
CommandDict = _make_command_dict()


def parse_command(fields):
    """Return a Command from a bytes string.

    Parameters
    ----------
    fields : `List` [`str`]
        Fields from a read message.
        The fields should not be terminated with ``\n``.

    Raises
    ------
    ValueError
        If the data cannot be parsed.
    """
    if len(fields) < 5:
        raise ValueError(f"A command has at least 5 fields; only got {len(fields)}")
    command_code = enums.CommandCode(int(fields[2]))
    try:
        CommandClass = CommandDict[command_code]
    except ValueError:
        raise RuntimeError(f"Unsupported command_code={command_code}")
    return CommandClass.from_str_fields(fields)