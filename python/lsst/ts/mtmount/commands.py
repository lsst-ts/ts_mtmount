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
    "AskForCommand",
    "AzimuthDriveEnable",
    "AzimuthDriveReset",
    "AzimuthDrivesThermalControlMode",
    "AzimuthDrivesThermalPower",
    "AzimuthDrivesThermalResetAlarm",
    "AzimuthEnableTracking",
    "AzimuthHome",
    "AzimuthMove",
    "AzimuthPower",
    "AzimuthResetAlarm",
    "AzimuthStop",
    "AzimuthTrackTarget",
    "ApplySettingsSet",
    "BothAxesEnableTracking",
    "BothAxesHome",
    "BothAxesMove",
    "BothAxesPower",
    "BothAxesResetAlarm",
    "BothAxesStop",
    "BothAxesTrackTarget",
    "Cabinet0101ThermalControlMode",
    "Cabinet0101ThermalPower",
    "Cabinet0101ThermalResetAlarm",
    "CameraCableWrapDriveEnable",
    "CameraCableWrapDriveReset",
    "CameraCableWrapEnableTracking",
    "CameraCableWrapMove",
    "CameraCableWrapPower",
    "CameraCableWrapResetAlarm",
    "CameraCableWrapStop",
    "CameraCableWrapTrackTarget",
    "Disable",
    "ElevationDriveEnable",
    "ElevationDriveReset",
    "ElevationDrivesThermalControlMode",
    "ElevationDrivesThermalPower",
    "ElevationDrivesThermalResetAlarm",
    "ElevationEnableTracking",
    "ElevationHome",
    "ElevationMove",
    "ElevationPower",
    "ElevationResetAlarm",
    "ElevationStop",
    "ElevationTrackTarget",
    "Enable",
    "GetActualSettings",
    "Heartbeat",
    "MainAxesPowerSupplyPower",
    "MainAxesPowerSupplyResetAlarm",
    "MainCabinetThermalResetAlarm",
    "MainCabinetThermalTrackAmbient",
    "MirrorCoverLocksMoveAll",
    "MirrorCoverLocksPower",
    "MirrorCoverLocksResetAlarm",
    "MirrorCoverLocksStop",
    "MirrorCoversDeploy",
    "MirrorCoversPower",
    "MirrorCoversResetAlarm",
    "MirrorCoversRetract",
    "MirrorCoversStop",
    "MirrorCoverSystemDeploy",
    "MirrorCoverSystemRetract",
    "AuxiliaryCabinetsThermalFanPower",
    "AuxiliaryCabinetsThermalResetAlarm",
    "AuxiliaryCabinetsThermalSetpoint",
    "OilSupplySystemCabinetsThermalSetpoint",
    "OilSupplySystemPower",
    "OilSupplySystemPowerCirculationPump",
    "OilSupplySystemPowerCooling",
    "OilSupplySystemPowerMainPump",
    "OilSupplySystemResetAlarm",
    "OilSupplySystemSetMode",
    "RestoreDefaultSettings",
    "SafetyReset",
    "StateInfo",
    "TopEndChillerPower",
    "TopEndChillerResetAlarm",
    "TopEndChillerTrackAmbient",
    "Commands",
    "CommandDict",
    "parse_command",
]

from . import enums, field_info
from .base_command import BaseCommand

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
            default=0,  # MTMountCsc sets this just before sending a command
        ),
        field_info.CommandCodeFieldInfo(command_code),
        field_info.SourceFieldInfo(what="command"),
        field_info.FloatFieldInfo(
            name="timestamp",
            doc="Time at which the message was sent.",
            default=0,  # MTMountCsc sets this just before sending the command
        ),
    ) + tuple(parameters)


_DriveParameter = (
    field_info.IntFieldInfo(
        name="drive", doc="Drive index: one of -1 (all), ?", default=-1
    ),
)

_ItemParameter = (
    field_info.IntFieldInfo(
        name="drive", doc="Items to control: one of -1 (all), ?", default=-1
    ),
)

_TrackingParameters = (
    field_info.FloatFieldInfo(name="position", doc="Target position (deg) at tai"),
    field_info.FloatFieldInfo(name="velocity", doc="Target velocity (deg/sec) at tai"),
    field_info.FloatFieldInfo(
        name="tai",
        doc="Target TAI time for position and velocity (unix seconds)",
    ),
)

_ApplySettingsSetParameters = (
    field_info.StrFieldInfo(
        name="settings", doc="The name of the setting set to be applied."
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

_OnOffParameter = (
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


class AskForCommand(BaseCommand):
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


class AzimuthDriveEnable(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_DRIVE_ENABLE,
        _DriveParameter + _OnOffParameter,
    )


class AzimuthDriveReset(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_DRIVE_RESET,
        _DriveParameter,
    )


class AzimuthDrivesThermalPower(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_DRIVES_THERMAL_POWER,
        _ItemParameter + _OnOffParameter,
    )


class AzimuthDrivesThermalControlMode(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_DRIVES_THERMAL_CONTROL_MODE,
        _ItemParameter
        + (
            field_info.EnumFieldInfo(
                name="mode",
                doc="Control mode",
                dtype=enums.ThermalMode,
                default=enums.ThermalMode.TRACK_SETPOINT,
            ),
            field_info.FloatFieldInfo(
                name="setpoint",
                doc="""Set point; the meaning depends on the mode:
• TRACK_AMBIENT: ignored
• TRACK_SETPOINT: the desired temperature (C)
• MANAGE_VALVE_SETPOINT: valve setpoint (%)
• MANAGE_AUTO_TUNE: start=1, stop=0""",
            ),
        ),
    )


class AzimuthDrivesThermalResetAlarm(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_DRIVES_THERMAL_RESET_ALARM,
        _ItemParameter,
    )


class AzimuthEnableTracking(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_ENABLE_TRACKING, _OnOffParameter
    )


class AzimuthHome(BaseCommand):
    field_infos = make_command_field_infos(enums.CommandCode.AZIMUTH_HOME)


class AzimuthMove(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_MOVE, _MoveParameters
    )


class AzimuthPower(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_POWER, _OnOffParameter
    )


class AzimuthResetAlarm(BaseCommand):
    field_infos = make_command_field_infos(enums.CommandCode.AZIMUTH_RESET_ALARM)


class AzimuthStop(BaseCommand):
    field_infos = make_command_field_infos(enums.CommandCode.AZIMUTH_STOP)


class AzimuthTrackTarget(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.AZIMUTH_TRACK_TARGET, _TrackingParameters
    )


class ApplySettingsSet(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.APPLY_SETTINGS_SET, _ApplySettingsSetParameters
    )


class BothAxesEnableTracking(BaseCommand):
    field_infos = make_command_field_infos(enums.CommandCode.BOTH_AXES_ENABLE_TRACKING)


class BothAxesHome(BaseCommand):
    field_infos = make_command_field_infos(enums.CommandCode.BOTH_AXES_HOME)


class BothAxesMove(BaseCommand):
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


class BothAxesPower(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.BOTH_AXES_POWER, _OnOffParameter
    )


class BothAxesResetAlarm(BaseCommand):
    field_infos = make_command_field_infos(enums.CommandCode.BOTH_AXES_RESET_ALARM)


class BothAxesStop(BaseCommand):
    """Stop both axes."""

    field_infos = make_command_field_infos(enums.CommandCode.BOTH_AXES_STOP)


class BothAxesTrackTarget(BaseCommand):
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


class Cabinet0101ThermalControlMode(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.CABINET_0101_THERMAL_CONTROL_MODE,
        _ItemParameter
        + (
            field_info.EnumFieldInfo(
                name="mode",
                doc="Control mode",
                dtype=enums.ThermalMode,
                default=enums.ThermalMode.TRACK_SETPOINT,
            ),
            field_info.FloatFieldInfo(
                name="setpoint",
                doc="""Set point; the meaning depends on the mode:
• TRACK_AMBIENT: ignored
• TRACK_SETPOINT: the desired temperature (C)
• MANAGE_VALVE_SETPOINT: valve setpoint (%)
• MANAGE_AUTO_TUNE: start=1, stop=0""",
            ),
        ),
    )


class Cabinet0101ThermalPower(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.CABINET_0101_THERMAL_POWER,
        _ItemParameter + _OnOffParameter,
    )


class Cabinet0101ThermalResetAlarm(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.CABINET_0101_THERMAL_RESET_ALARM,
        _ItemParameter,
    )


class CameraCableWrapDriveEnable(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_DRIVE_ENABLE,
        (
            field_info.IntFieldInfo(
                name="drive",
                doc="Drive index; just one drive can be enabled, so enabling one disables the other",
            ),
        )
        + _OnOffParameter,
    )


class CameraCableWrapDriveReset(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_DRIVE_RESET,
        _DriveParameter,
    )


class CameraCableWrapEnableTracking(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_ENABLE_TRACKING, _OnOffParameter
    )


class CameraCableWrapMove(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_MOVE, _MoveParameters
    )


class CameraCableWrapPower(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_POWER, _OnOffParameter
    )


class CameraCableWrapResetAlarm(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_RESET_ALARM,
    )


class CameraCableWrapStop(BaseCommand):
    field_infos = make_command_field_infos(enums.CommandCode.CAMERA_CABLE_WRAP_STOP)


class CameraCableWrapTrackTarget(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.CAMERA_CABLE_WRAP_TRACK_TARGET, _TrackingParameters
    )


class Disable(BaseCommand):
    field_infos = make_command_field_infos(enums.CommandCode.DISABLE)


class Enable(BaseCommand):
    field_infos = make_command_field_infos(enums.CommandCode.ENABLE)


class ElevationDriveEnable(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_DRIVE_ENABLE,
        _DriveParameter + _OnOffParameter,
    )


class ElevationDriveReset(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_DRIVE_RESET,
        _DriveParameter,
    )


class ElevationDrivesThermalPower(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_DRIVES_THERMAL_POWER,
        _ItemParameter + _OnOffParameter,
    )


class ElevationDrivesThermalControlMode(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_DRIVES_THERMAL_CONTROL_MODE,
        _ItemParameter
        + (
            field_info.EnumFieldInfo(
                name="mode",
                doc="Control mode",
                dtype=enums.ThermalMode,
                default=enums.ThermalMode.TRACK_SETPOINT,
            ),
            field_info.FloatFieldInfo(
                name="setpoint",
                doc="""Set point; the meaning depends on the mode:
• TRACK_AMBIENT: ignored
• TRACK_SETPOINT: the desired temperature (C)
• MANAGE_VALVE_SETPOINT: valve setpoint (%)
• MANAGE_AUTO_TUNE: start=1, stop=0""",
            ),
        ),
    )


class ElevationDrivesThermalResetAlarm(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_DRIVES_THERMAL_RESET_ALARM,
        _ItemParameter,
    )


class ElevationEnableTracking(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_ENABLE_TRACKING, _OnOffParameter
    )


class ElevationHome(BaseCommand):
    field_infos = make_command_field_infos(enums.CommandCode.ELEVATION_HOME)


class ElevationMove(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_MOVE, _MoveParameters
    )


class ElevationPower(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_POWER, _OnOffParameter
    )


class ElevationResetAlarm(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_RESET_ALARM,
    )


class ElevationStop(BaseCommand):
    field_infos = make_command_field_infos(enums.CommandCode.ELEVATION_STOP)


class ElevationTrackTarget(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.ELEVATION_TRACK_TARGET, _TrackingParameters
    )


class GetActualSettings(BaseCommand):
    field_infos = make_command_field_infos(enums.CommandCode.GET_ACTUAL_SETTINGS)


class Heartbeat(BaseCommand):
    field_infos = make_command_field_infos(enums.CommandCode.HEARTBEAT)


class MainAxesPowerSupplyPower(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.MAIN_AXES_POWER_SUPPLY_POWER, _OnOffParameter
    )


class MainAxesPowerSupplyResetAlarm(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.MAIN_AXES_POWER_SUPPLY_RESET_ALARM
    )


class MainCabinetThermalResetAlarm(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.MAIN_CABINET_THERMAL_RESET_ALARM
    )


class MainCabinetThermalTrackAmbient(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.MAIN_CABINET_THERMAL_TRACK_AMBIENT,
        (
            field_info.BoolFieldInfo(
                name="track_ambient",
                doc="What to track: ambient if true, setpoint if false",
            ),
            field_info.FloatFieldInfo(
                name="setpoint",
                doc="Temperature setpoint (C); ignored if track is false",
            ),
        ),
    )


class MirrorCoverSystemDeploy(BaseCommand):
    field_infos = make_command_field_infos(enums.CommandCode.MIRROR_COVER_SYSTEM_DEPLOY)


class MirrorCoverSystemRetract(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVER_SYSTEM_RETRACT
    )


"""
Unsupported mirror cover locks commands:

* MIRROR_COVER_LOCKS_MOVE = 903
* MIRROR_COVER_LOCKS_MOVE_VELOCITY = 904
"""


class MirrorCoverLocksMoveAll(BaseCommand):
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


class MirrorCoverLocksPower(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVER_LOCKS_POWER,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
        )
        + _OnOffParameter,
    )


class MirrorCoverLocksResetAlarm(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVER_LOCKS_RESET_ALARM,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
        ),
    )


class MirrorCoverLocksStop(BaseCommand):
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


class MirrorCoversDeploy(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVERS_DEPLOY,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
        ),
    )


class MirrorCoversRetract(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVERS_RETRACT,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
        ),
    )


class MirrorCoversPower(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVERS_POWER,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
        )
        + _OnOffParameter,
    )


class MirrorCoversResetAlarm(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVERS_RESET_ALARM,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
        ),
    )


class MirrorCoversStop(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.MIRROR_COVERS_STOP,
        (
            field_info.IntFieldInfo(
                name="drive", doc="Drive index: one of -1 (all), 0, 1, 2, 3", default=-1
            ),
        ),
    )


class AuxiliaryCabinetsThermalFanPower(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.AUXILIARY_CABINETS_THERMAL_FAN_POWER,
        _ItemParameter + _OnOffParameter,
    )


class AuxiliaryCabinetsThermalResetAlarm(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.AUXILIARY_CABINETS_THERMAL_RESET_ALARM,
        (
            field_info.IntFieldInfo(
                name="cabinet", doc="Cabinet index: one of -1 (all), ?", default=-1
            ),
        ),
    )


class AuxiliaryCabinetsThermalSetpoint(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.AUXILIARY_CABINETS_THERMAL_SETPOINT,
        (
            field_info.IntFieldInfo(
                name="cabinet", doc="Cabinet index: one of -1 (all), ?", default=-1
            ),
            field_info.FloatFieldInfo(name="setpoint", doc="Temperature setpoint (C)"),
        ),
    )


class OilSupplySystemCabinetsThermalSetpoint(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.OIL_SUPPLY_SYSTEM_CABINETS_THERMAL_SETPOINT,
        (field_info.FloatFieldInfo(name="setpoint", doc="Temperature setpoint (C)"),),
    )


class OilSupplySystemPower(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.OIL_SUPPLY_SYSTEM_POWER,
        _OnOffParameter,
    )


class OilSupplySystemPowerCirculationPump(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.OIL_SUPPLY_SYSTEM_POWER_CIRCULATION_PUMP,
        _OnOffParameter,
    )


class OilSupplySystemPowerCooling(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.OIL_SUPPLY_SYSTEM_POWER_COOLING,
        _OnOffParameter,
    )


class OilSupplySystemPowerMainPump(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.OIL_SUPPLY_SYSTEM_POWER_MAIN_PUMP,
        _OnOffParameter,
    )


class OilSupplySystemResetAlarm(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.OIL_SUPPLY_SYSTEM_RESET_ALARM,
    )


class OilSupplySystemSetMode(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.OIL_SUPPLY_SYSTEM_SET_MODE,
        (
            field_info.BoolFieldInfo(
                name="auto", doc="Automatic mode = true, manual mode = false"
            ),
        ),
    )


class RestoreDefaultSettings(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.RESTORE_DEFAULT_SETTINGS,
    )


class SafetyReset(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.SAFETY_RESET,
        (
            field_info.StrFieldInfo(
                name="what",
                doc="Which systems to reset; a colon-separated list of integers.",
            ),
        ),
    )


class StateInfo(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.STATE_INFO,
    )


class TopEndChillerPower(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.TOP_END_CHILLER_POWER,
        _OnOffParameter,
    )


class TopEndChillerResetAlarm(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.TOP_END_CHILLER_RESET_ALARM,
    )


class TopEndChillerTrackAmbient(BaseCommand):
    field_infos = make_command_field_infos(
        enums.CommandCode.TOP_END_CHILLER_TRACK_AMBIENT,
        (
            field_info.BoolFieldInfo(
                name="track_ambient",
                doc="What to track: ambient if true, setpoint if false",
            ),
            field_info.FloatFieldInfo(name="setpoint", doc="desired setpoint (C)"),
        ),
    )


Commands = (
    ApplySettingsSet,
    AskForCommand,
    AzimuthDriveEnable,
    AzimuthDriveReset,
    AzimuthEnableTracking,
    AzimuthHome,
    AzimuthMove,
    AzimuthPower,
    AzimuthResetAlarm,
    AzimuthStop,
    AzimuthDrivesThermalPower,
    AzimuthDrivesThermalControlMode,
    AzimuthDrivesThermalResetAlarm,
    AzimuthTrackTarget,
    BothAxesEnableTracking,
    BothAxesHome,
    BothAxesMove,
    BothAxesPower,
    BothAxesResetAlarm,
    BothAxesStop,
    BothAxesTrackTarget,
    Cabinet0101ThermalControlMode,
    Cabinet0101ThermalPower,
    Cabinet0101ThermalResetAlarm,
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
    ElevationDrivesThermalPower,
    ElevationDrivesThermalControlMode,
    ElevationDrivesThermalResetAlarm,
    ElevationTrackTarget,
    GetActualSettings,
    Heartbeat,
    MainAxesPowerSupplyPower,
    MainAxesPowerSupplyResetAlarm,
    MainCabinetThermalResetAlarm,
    MainCabinetThermalTrackAmbient,
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
    AuxiliaryCabinetsThermalFanPower,
    AuxiliaryCabinetsThermalResetAlarm,
    AuxiliaryCabinetsThermalSetpoint,
    OilSupplySystemCabinetsThermalSetpoint,
    OilSupplySystemPower,
    OilSupplySystemPowerCirculationPump,
    OilSupplySystemPowerCooling,
    OilSupplySystemPowerMainPump,
    OilSupplySystemResetAlarm,
    OilSupplySystemSetMode,
    RestoreDefaultSettings,
    SafetyReset,
    StateInfo,
    TopEndChillerPower,
    TopEndChillerResetAlarm,
    TopEndChillerTrackAmbient,
)

for command in Commands:
    command.make_command_doc()


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
            f"A command must have at least {NUM_HEADER_FIELDS} fields; got {fields=}"
        )
    command_code = enums.CommandCode(int(fields[1]))
    try:
        CommandClass = CommandDict[command_code]
    except ValueError:
        raise RuntimeError(f"Unsupported command_code={command_code}")
    return CommandClass.from_str_fields(fields)
