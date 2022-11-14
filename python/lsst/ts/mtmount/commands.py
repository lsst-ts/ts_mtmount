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

__all__ = [
    "NUM_HEADER_FIELDS",
    "AckOnlyCommandCodes",
    "Command",
    "AskForCommand",
    "AzimuthDriveEnable",
    "AzimuthDriveReset",
    "AzimuthEnableTracking",
    "AzimuthHome",
    "AzimuthMove",
    "AzimuthPower",
    "AzimuthResetAlarm",
    "AzimuthStop",
    "AzimuthTrackTarget",
    "BothAxesEnableTracking",
    "BothAxesHome",
    "BothAxesMove",
    "BothAxesPower",
    "BothAxesResetAlarm",
    "BothAxesStop",
    "BothAxesTrackTarget",
    "CameraCableWrapDriveEnable",
    "CameraCableWrapDriveReset",
    "CameraCableWrapEnableTracking",
    "CameraCableWrapMove",
    "CameraCableWrapPower",
    "CameraCableWrapResetAlarm",
    "CameraCableWrapStop",
    "CameraCableWrapTrackTarget",
    "Disable",
    "Enable",
    "ElevationDriveEnable",
    "ElevationDriveReset",
    "ElevationEnableTracking",
    "ElevationHome",
    "ElevationMove",
    "ElevationPower",
    "ElevationResetAlarm",
    "ElevationStop",
    "ElevationTrackTarget",
    "GetActualSettings",
    "Heartbeat",
    "MainAxesPowerSupplyPower",
    "MainAxesPowerSupplyResetAlarm",
    "MainCabinetThermalResetAlarm",
    "MirrorCoverSystemDeploy",
    "MirrorCoverSystemRetract",
    "MirrorCoverLocksMoveAll",
    "MirrorCoverLocksPower",
    "MirrorCoverLocksResetAlarm",
    "MirrorCoverLocksStop",
    "MirrorCoversDeploy",
    "MirrorCoversRetract",
    "MirrorCoversPower",
    "MirrorCoversResetAlarm",
    "MirrorCoversStop",
    "OilSupplySystemPower",
    "OilSupplySystemPowerCirculationPump",
    "OilSupplySystemPowerCooling",
    "OilSupplySystemPowerMainPump",
    "OilSupplySystemResetAlarm",
    "OilSupplySystemSetMode",
    "SafetyReset",
    "StateInfo",
    "TopEndChillerPower",
    "TopEndChillerResetAlarm",
    "TopEndChillerTrackAmbient",
    "Commands",
    "CommandDict",
    "parse_command",
]

from lsst.ts import utils

from . import base_message, enums, field_info
from .utils import wrap_parameter_doc

MAX_SEQUENCE_ID = (1 << 31) - 1

# Number of required fields, before the optional arguments start
NUM_HEADER_FIELDS = 4


# Command that are done when CMD_ACKNOWLEDGED is received
AckOnlyCommandCodes = set(
    (
        enums.CommandCode.BOTH_AXES_TRACK_TARGET,
        enums.CommandCode.AZIMUTH_TRACK_TARGET,
        enums.CommandCode.ELEVATION_TRACK_TARGET,
        enums.CommandCode.AZIMUTH_CABLE_WRAP_TRACK_TARGET,
        enums.CommandCode.CAMERA_CABLE_WRAP_TRACK_TARGET,
    )
)


class Command(base_message.BaseMessage):
    """Base class for commands.

    Set sequence_id to an incrementing integer (that eventually
    wraps around), by default.
    """

    sequence_id_generator = utils.index_generator(imax=MAX_SEQUENCE_ID)

    def __init__(self, **kwargs):
        if kwargs.get("sequence_id") is None:
            kwargs["sequence_id"] = next(self.sequence_id_generator)
        return super().__init__(**kwargs)


def make_command_doc(cls):
    """Make and attach a doc string to a command class."""
    param_strings = []
    for finfo in cls.field_infos:
        param_doc = wrap_parameter_doc(finfo.doc)
        is_optional = finfo.default is not None or finfo.name == "sequence_id"
        optional_str = ", optional" if is_optional else ""
        param_strings.append(
            f"{finfo.name} : `{finfo.dtype.__name__}{optional_str}`\n{param_doc}"
        )
    param_block = "\n".join(param_strings)
    cls.__doc__ = f"""{cls.__name__} command.

Parameters
----------
{param_block}
"""


def make_command_field_infos(command_code, parameters=()):
    """Make field_infos for a command.

    Parameters
    ----------
    command_code : `CommandCode`
        Command code.
    parameters : `List` [`FieldInfo`], optional
        Field information for command parameters, if any.
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
        field_info.CommandCodeFieldInfo(command_code),
        field_info.SourceFieldInfo(what="command"),
        field_info.TimestampFieldInfo(),
    ) + tuple(parameters)


_TrackingParameters = (
    field_info.FloatFieldInfo(name="position", doc="Target position (deg) at tai"),
    field_info.FloatFieldInfo(name="velocity", doc="Target velocity (deg/sec) at tai"),
    field_info.FloatFieldInfo(
        name="tai",
        doc="Target TAI time for position and velocity (unix seconds)",
    ),
)

_MoveParameters = (
    field_info.FloatFieldInfo(name="position", doc="Target position (deg)."),
    field_info.FloatFieldInfo(
        name="velocity",
        default=0,
        doc="Maximum velocity; 0 for the default value (deg/second).",
    ),
    field_info.FloatFieldInfo(
        name="acceleration",
        default=0,
        doc="Maximum acceleration; 0 for the default value (deg/second2).",
    ),
    field_info.FloatFieldInfo(
        name="jerk",
        default=0,
        doc="Maximum jerk; 0 for the default value (deg/second3).",
    ),
)

_OnOffParameters = (
    field_info.BoolFieldInfo(name="on", doc="Turn on (True) or off (False)"),
)

"""
Supported elevation and azimuth axis commands:

* <AXIS>_AXIS_POWER
* <AXIS>_AXIS_STOP
* <AXIS>_AXIS_MOVE
* <AXIS>_AXIS_TRACK

Other elevation and azimuth axis commands:

* <AXIS>_AXIS_MOVE_VELOCITY
* <AXIS>_AXIS_HOME
* <AXIS>_AXIS_RESET_ALARM
"""


class AskForCommand(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.ASK_FOR_COMMAND,
        (
            field_info.EnumFieldInfo(
                name="commander",
                doc="Who should have command",
                dtype=enums.Source,
                default=enums.Source.CSC,
            ),
        ),
    )


class AzimuthDriveEnable(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_DRIVE_ENABLE,
        (
            (
                field_info.IntFieldInfo(
                    name="drive", doc="Drive index: one of -1 (all), ?", default=-1
                ),
            )
            + _OnOffParameters
        ),
    )


class AzimuthDriveReset(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_DRIVE_RESET,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), ?", default=-1
            ),
        ),
    )


class AzimuthEnableTracking(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_ENABLE_TRACKING, _OnOffParameters
    )


class AzimuthHome(Command):
    field_infos = make_command_field_infos(enums.CommandCode.AZIMUTH_HOME)


class AzimuthMove(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_MOVE, _MoveParameters
    )


class AzimuthPower(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_POWER, _OnOffParameters
    )


class AzimuthResetAlarm(Command):
    field_infos = make_command_field_infos(enums.CommandCode.AZIMUTH_RESET_ALARM)


class AzimuthStop(Command):
    field_infos = make_command_field_infos(enums.CommandCode.AZIMUTH_STOP)


class AzimuthTrackTarget(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_TRACK_TARGET, _TrackingParameters
    )


class BothAxesEnableTracking(Command):
    field_infos = make_command_field_infos(enums.CommandCode.BOTH_AXES_ENABLE_TRACKING)


class BothAxesHome(Command):
    field_infos = make_command_field_infos(enums.CommandCode.BOTH_AXES_HOME)


class BothAxesMove(Command):
    """Move both axes to a specified position and stop."""

    field_infos = make_command_field_infos(
        enums.CommandCode.BOTH_AXES_MOVE,
        (
            field_info.FloatFieldInfo(name="azimuth", doc="Desired azimuth (deg)"),
            field_info.FloatFieldInfo(name="elevation", doc="Desired elevation (deg)"),
            field_info.FloatFieldInfo(
                name="azimuth_velocity",
                default=0,
                doc="Maximum azimuth velocity (0 for the default value) (deg/sec)",
            ),
            field_info.FloatFieldInfo(
                name="elevation_velocity",
                default=0,
                doc="Maximum elevation velocity (0 for the default value) (deg/sec)",
            ),
            field_info.FloatFieldInfo(
                name="azimuth_acceleration",
                default=0,
                doc="Maximum azimuth acceleration (0 for the default value) (deg/sec^2)",
            ),
            field_info.FloatFieldInfo(
                name="elevation_acceleration",
                default=0,
                doc="Maximum elevation acceleration (0 for the default value) (deg/sec^2)",
            ),
            field_info.FloatFieldInfo(
                name="azimuth_jerk",
                default=0,
                doc="Maximum azimuth jerk (0 for the default value) (deg/sec^3)",
            ),
            field_info.FloatFieldInfo(
                name="elevation_jerk",
                default=0,
                doc="Maximum elevation jerk (0 for the default value) (deg/sec^3)",
            ),
        ),
    )


class BothAxesPower(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.BOTH_AXES_POWER, _OnOffParameters
    )


class BothAxesResetAlarm(Command):
    field_infos = make_command_field_infos(enums.CommandCode.BOTH_AXES_RESET_ALARM)


class BothAxesStop(Command):
    """Stop both axes."""

    field_infos = make_command_field_infos(enums.CommandCode.BOTH_AXES_STOP)


class BothAxesTrackTarget(Command):
    """Specify the tracking target for both axes."""

    field_infos = make_command_field_infos(
        enums.CommandCode.BOTH_AXES_TRACK_TARGET,
        (
            field_info.FloatFieldInfo(
                name="azimuth", doc="Target azimuth at tai (deg)"
            ),
            field_info.FloatFieldInfo(
                name="elevation", doc="Target elevation at tai (deg)"
            ),
            field_info.FloatFieldInfo(
                name="azimuth_velocity",
                doc="Target azimuth velocity at tai (deg)",
            ),
            field_info.FloatFieldInfo(
                name="elevation_velocity",
                doc="Target elevation velocity at tai (deg)",
            ),
            field_info.FloatFieldInfo(
                name="tai",
                doc="Target TAI time for position and velocity (unix seconds)",
            ),
        ),
    )


class CameraCableWrapDriveEnable(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_DRIVE_ENABLE,
        (
            field_info.IntFieldInfo(
                name="drive",
                doc="Drive index; just one drive can be enabled, so enabling one disables the other",
            ),
        )
        + _OnOffParameters,
    )


class CameraCableWrapDriveReset(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_DRIVE_RESET,
        (field_info.IntFieldInfo(name="drive", doc="Drive index; one of -1=all, ?"),),
    )


class CameraCableWrapEnableTracking(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_ENABLE_TRACKING, _OnOffParameters
    )


class CameraCableWrapMove(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_MOVE, _MoveParameters
    )


class CameraCableWrapPower(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_POWER, _OnOffParameters
    )


class CameraCableWrapResetAlarm(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_RESET_ALARM,
    )


class CameraCableWrapStop(Command):
    field_infos = make_command_field_infos(enums.CommandCode.CAMERA_CABLE_WRAP_STOP)


class CameraCableWrapTrackTarget(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_TRACK_TARGET, _TrackingParameters
    )


class Disable(Command):
    field_infos = make_command_field_infos(enums.CommandCode.DISABLE)


class Enable(Command):
    field_infos = make_command_field_infos(enums.CommandCode.ENABLE)


class ElevationDriveEnable(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_DRIVE_ENABLE,
        (
            (
                field_info.IntFieldInfo(
                    name="drive", doc="Drive index: one of -1 (all), ?", default=-1
                ),
            )
            + _OnOffParameters
        ),
    )


class ElevationDriveReset(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_DRIVE_RESET,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), ?", default=-1
            ),
        ),
    )


class ElevationEnableTracking(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_ENABLE_TRACKING, _OnOffParameters
    )


class ElevationHome(Command):
    field_infos = make_command_field_infos(enums.CommandCode.ELEVATION_HOME)


class ElevationMove(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_MOVE, _MoveParameters
    )


class ElevationPower(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_POWER, _OnOffParameters
    )


class ElevationResetAlarm(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_RESET_ALARM,
    )


class ElevationStop(Command):
    field_infos = make_command_field_infos(enums.CommandCode.ELEVATION_STOP)


class ElevationTrackTarget(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_TRACK_TARGET, _TrackingParameters
    )


class GetActualSettings(Command):
    field_infos = make_command_field_infos(enums.CommandCode.GET_ACTUAL_SETTINGS)


class Heartbeat(Command):
    field_infos = make_command_field_infos(enums.CommandCode.HEARTBEAT)


class MainAxesPowerSupplyPower(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.MAIN_AXES_POWER_SUPPLY_POWER, _OnOffParameters
    )


class MainAxesPowerSupplyResetAlarm(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.MAIN_AXES_POWER_SUPPLY_RESET_ALARM
    )


class MainCabinetThermalResetAlarm(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.MAIN_CABINET_THERMAL_RESET_ALARM
    )


class MirrorCoverSystemDeploy(Command):
    field_infos = make_command_field_infos(enums.CommandCode.MIRROR_COVER_SYSTEM_DEPLOY)


class MirrorCoverSystemRetract(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVER_SYSTEM_RETRACT
    )


"""
Unsupported mirror cover locks commands:

* MIRROR_COVER_LOCKS_MOVE = 903
* MIRROR_COVER_LOCKS_MOVE_VELOCITY = 904
"""


class MirrorCoverLocksMoveAll(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVER_LOCKS_MOVE_ALL,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
            field_info.BoolFieldInfo(
                name="deploy", doc="Deploy (True) or retract (False)?"
            ),
        ),
    )


class MirrorCoverLocksPower(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVER_LOCKS_POWER,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
        )
        + _OnOffParameters,
    )


class MirrorCoverLocksResetAlarm(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVER_LOCKS_RESET_ALARM,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
        ),
    )


class MirrorCoverLocksStop(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVER_LOCKS_STOP,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
        ),
    )


"""
Unsupported mirror cover commands:

* MIRROR_COVERS_MOVE = 903
* MIRROR_COVERS_MOVE_VELOCITY = 904
"""


class MirrorCoversDeploy(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVERS_DEPLOY,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
        ),
    )


class MirrorCoversRetract(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVERS_RETRACT,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
        ),
    )


class MirrorCoversPower(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVERS_POWER,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
        )
        + _OnOffParameters,
    )


class MirrorCoversResetAlarm(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVERS_RESET_ALARM,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
        ),
    )


class MirrorCoversStop(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVERS_STOP,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
        ),
    )


class OilSupplySystemPower(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.OIL_SUPPLY_SYSTEM_POWER,
        _OnOffParameters,
    )


class OilSupplySystemPowerCirculationPump(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.OIL_SUPPLY_SYSTEM_POWER_CIRCULATION_PUMP,
        _OnOffParameters,
    )


class OilSupplySystemPowerCooling(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.OIL_SUPPLY_SYSTEM_POWER_COOLING,
        _OnOffParameters,
    )


class OilSupplySystemPowerMainPump(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.OIL_SUPPLY_SYSTEM_POWER_MAIN_PUMP,
        _OnOffParameters,
    )


class OilSupplySystemResetAlarm(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.OIL_SUPPLY_SYSTEM_RESET_ALARM,
    )


class OilSupplySystemSetMode(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.OIL_SUPPLY_SYSTEM_SET_MODE,
        (
            field_info.BoolFieldInfo(
                name="auto", doc="Automatic mode = true, manual mode = false"
            ),
        ),
    )


class SafetyReset(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.SAFETY_RESET,
        (
            field_info.StrFieldInfo(
                name="what",
                doc="Which systems to reset; a colon-separated list of integers.",
            ),
        ),
    )


class StateInfo(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.STATE_INFO,
    )


class TopEndChillerPower(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.TOP_END_CHILLER_POWER,
        _OnOffParameters,
    )


class TopEndChillerResetAlarm(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.TOP_END_CHILLER_RESET_ALARM,
    )


class TopEndChillerTrackAmbient(Command):
    field_infos = make_command_field_infos(
        enums.CommandCode.TOP_END_CHILLER_TRACK_AMBIENT,
        _OnOffParameters + (field_info.FloatFieldInfo(name="temperature", doc="???"),),
    )


Commands = (
    AskForCommand,
    AzimuthDriveEnable,
    AzimuthDriveReset,
    AzimuthEnableTracking,
    AzimuthHome,
    AzimuthMove,
    AzimuthPower,
    AzimuthResetAlarm,
    AzimuthStop,
    AzimuthTrackTarget,
    BothAxesEnableTracking,
    BothAxesHome,
    BothAxesMove,
    BothAxesPower,
    BothAxesResetAlarm,
    BothAxesStop,
    BothAxesTrackTarget,
    CameraCableWrapDriveEnable,
    CameraCableWrapDriveReset,
    CameraCableWrapEnableTracking,
    CameraCableWrapMove,
    CameraCableWrapPower,
    CameraCableWrapResetAlarm,
    CameraCableWrapStop,
    CameraCableWrapTrackTarget,
    Disable,
    Enable,
    ElevationDriveEnable,
    ElevationDriveReset,
    ElevationEnableTracking,
    ElevationHome,
    ElevationMove,
    ElevationPower,
    ElevationResetAlarm,
    ElevationStop,
    ElevationTrackTarget,
    GetActualSettings,
    Heartbeat,
    MainAxesPowerSupplyPower,
    MainAxesPowerSupplyResetAlarm,
    MainCabinetThermalResetAlarm,
    MirrorCoverSystemDeploy,
    MirrorCoverSystemRetract,
    MirrorCoverLocksMoveAll,
    MirrorCoverLocksPower,
    MirrorCoverLocksResetAlarm,
    MirrorCoverLocksStop,
    MirrorCoversDeploy,
    MirrorCoversRetract,
    MirrorCoversPower,
    MirrorCoversResetAlarm,
    MirrorCoversStop,
    OilSupplySystemPower,
    OilSupplySystemPowerCirculationPump,
    OilSupplySystemPowerCooling,
    OilSupplySystemPowerMainPump,
    OilSupplySystemResetAlarm,
    OilSupplySystemSetMode,
    SafetyReset,
    StateInfo,
    TopEndChillerPower,
    TopEndChillerResetAlarm,
    TopEndChillerTrackAmbient,
)

for command in Commands:
    make_command_doc(command)


def _make_command_dict():
    """Make a dict of command_code: CommandClass from `Commands`.

    Check that no command codes are duplicated.
    """
    command_dict = {}
    for command in Commands:
        command_code = command.field_infos[1].default
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


def parse_command(command_str):
    """Parse string as a `Command`.

    Parameters
    ----------
    command_str : `str`
        Command encoded as a sequence of ``\n``-separated fields.
        Leading and trailing whitespace, ``\n``, and/or ``\r`` are ignored.

    Returns
    -------
    command : `Command`
        The parsed command.

    Raises
    ------
    ValueError
        If the string cannot be parsed.
    """
    fields = command_str.strip().split("\n")
    if len(fields) < NUM_HEADER_FIELDS:
        raise ValueError(
            f"A command has at least {NUM_HEADER_FIELDS} fields; only got {len(fields)}"
        )
    command_code = enums.CommandCode(int(fields[1]))
    try:
        CommandClass = CommandDict[command_code]
    except ValueError:
        raise RuntimeError(f"Unsupported command_code={command_code}")
    return CommandClass.from_str_fields(fields)
