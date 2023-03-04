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
    "CommandCode",
    "CscErrorCode",
    "EnabledState",
    "ReplyId",
    "Source",
    "ThermalMode",
]

import enum


class CommandCode(enum.IntEnum):
    """Command codes for Command.command_code.

    The values and names are from COMMAND_TYPE in types.h,
    with abbreviations expanded.

    Commands 108 and 204 are not yet implemented
    but Tekniker says they plan to add them.
    """

    MOVE_TO_TARGET = 1
    TRACK_TARGET = 2
    ENABLE_CAMERA_WRAP = 3
    DISABLE_CAMERA_WRAP = 4
    # Control both the mirror cover and mirror cover lock.
    OPEN_MIRROR_COVER = 5
    CLOSE_MIRROR_COVER = 6
    STOP_MOUNT = 7
    START = 8
    STAND_BY = 9
    ENABLE = 10
    DISABLE = 11
    EXIT = 12
    CLEAR_ERRORS = 13
    ENTER_CONTROL = 14
    SYSTEM_READY = 15
    ENTER_PUBLISHONLY = 16
    BOTH_AXES_POWER = 31
    BOTH_AXES_STOP = 32  # HMI_STOP in types.h
    BOTH_AXES_MOVE = 33  # HMI_MOVE_TO_TARGET in types.h
    BOTH_AXES_TRACK_TARGET = 35  # HMI_TRACK_TARGET in types.h
    BOTH_AXES_HOME = 36
    BOTH_AXES_RESET_ALARM = 37  # If one fails, will still reset the other
    BOTH_AXES_ENABLE_TRACKING = 38
    MIRROR_COVER_SYSTEM_DEPLOY = 41  # Handle mirror covers and locks
    MIRROR_COVER_SYSTEM_RETRACT = 42  # Handle mirror covers and locks
    AZIMUTH_POWER = 101
    AZIMUTH_STOP = 102
    AZIMUTH_MOVE = 103
    AZIMUTH_MOVE_VELOCITY = 104
    # AZ_AXIS_TRACKING in types.h
    AZIMUTH_TRACK_TARGET = 105
    AZIMUTH_HOME = 106
    AZIMUTH_RESET_ALARM = 107
    # Not sure what the name will be; not yet implemented
    AZIMUTH_ENABLE_TRACKING = 108
    AZIMUTH_DRIVE_RESET = 201
    AZIMUTH_DRIVE_ENABLE = 202
    AZIMUTH_CABLE_WRAP_POWER = 301
    AZIMUTH_CABLE_WRAP_STOP = 302
    AZIMUTH_CABLE_WRAP_MOVE = 303
    AZIMUTH_CABLE_WRAP_MOVE_VELOCITY = 304
    # AZ_CW_TRACK_AZIMUTH in types.h
    AZIMUTH_CABLE_WRAP_TRACK_TARGET = 305
    AZIMUTH_CABLE_WRAP_RESET_ALARM = 306
    AZIMUTH_CABLE_WRAP_DRIVE_RESET = 307
    AZIMUTH_CABLE_WRAP_DRIVE_ENABLE = 308
    # AZ_CW_ENABLE_TRACK_AZIMUTH in types.h
    AZIMUTH_CABLE_WRAP_ENABLE_TRACKING = 309
    ELEVATION_POWER = 401
    ELEVATION_STOP = 402
    ELEVATION_MOVE = 403
    ELEVATION_MOVE_VELOCITY = 404
    # EL_AXIS_TRACKING in types.h
    ELEVATION_TRACK_TARGET = 405
    ELEVATION_HOME = 406
    ELEVATION_RESET_ALARM = 407
    ELEVATION_ENABLE_TRACKING = 408
    ELEVATION_DRIVE_RESET = 501
    ELEVATION_DRIVE_ENABLE = 502
    # The main axes power supply must be on
    # in order to turn on power to elevation and azimuth.
    # While off you cannot usefullly reset alarms for these axes
    # (the command succeeds but has no effect).
    MAIN_AXES_POWER_SUPPLY_POWER = 601
    MAIN_AXES_POWER_SUPPLY_RESET_ALARM = 602
    ENCODER_INTERFACE_BOX_POWER = 701
    ENCODER_INTERFACE_BOX_REFERENCE = 702
    ENCODER_INTERFACE_BOX_RESET = 703
    ENCODER_INTERFACE_BOX_RESET_ERROR = 704
    ENCODER_INTERFACE_BOX_CLEAR_POSITION_ERROR = 705
    ENCODER_INTERFACE_BOX_EXIT = 706
    OIL_SUPPLY_SYSTEM_POWER = 801
    OIL_SUPPLY_SYSTEM_POWER_COOLING = 802
    OIL_SUPPLY_SYSTEM_POWER_CIRCULATION_PUMP = 803
    OIL_SUPPLY_SYSTEM_POWER_MAIN_PUMP = 804
    OIL_SUPPLY_SYSTEM_RESET_ALARM = 805
    OIL_SUPPLY_SYSTEM_SET_MODE = 806
    OIL_SUPPLY_SYSTEM_ABORT_POWERING = 807
    OIL_SUPPLY_SYSTEM_CABINETS_THERMAL_SETPOINT = 808
    MIRROR_COVERS_POWER = 901
    MIRROR_COVERS_STOP = 902
    MIRROR_COVERS_MOVE = 903
    MIRROR_COVERS_MOVE_VELOCITY = 904
    # NOTES:
    # * deploy is what we call closing the mirror covers
    # * retract is what we call opening the mirror covers
    MIRROR_COVERS_DEPLOY = 905
    MIRROR_COVERS_RETRACT = 906
    MIRROR_COVERS_RESET_ALARM = 907
    CAMERA_CABLE_WRAP_POWER = 1001
    CAMERA_CABLE_WRAP_STOP = 1002
    CAMERA_CABLE_WRAP_MOVE = 1003
    CAMERA_CABLE_WRAP_TRACK_TARGET = 1004
    CAMERA_CABLE_WRAP_RESET_ALARM = 1005
    CAMERA_CABLE_WRAP_DRIVE_ENABLE = 1006
    CAMERA_CABLE_WRAP_DRIVE_RESET = 1007
    CAMERA_CABLE_WRAP_MOVE_VELOCITY = 1008
    CAMERA_CABLE_WRAP_ENABLE_TRACKING = 1009
    BALANCE_POWER = 1101
    BALANCE_STOP = 1102
    BALANCE_MOVE = 1103
    BALANCE_RESET_ALARM = 1104
    DEPLOYABLE_PLATFORMS_POWER = 1201
    DEPLOYABLE_PLATFORMS_STOP = 1202
    DEPLOYABLE_PLATFORMS_MOVE_VELOCITY = 1204
    DEPLOYABLE_PLATFORMS_RESET_ALARM = 1205
    DEPLOYABLE_PLATFORMS_LOCK_EXTENSION = 1206
    DEPLOYABLE_PLATFORMS_EXTEND_RETRACT = 1207
    # Main cabinet temperature controller.
    MAIN_CABINET_THERMAL_TRACK_AMBIENT = 1301
    MAIN_CABINET_THERMAL_RESET_ALARM = 1302
    MAIN_CABINET_THERMAL_SET_TEMPERATURE = 1303
    LOCKING_PINS_POWER = 1401
    LOCKING_PINS_STOP = 1402
    LOCKING_PINS_MOVE = 1403
    LOCKING_PINS_MOVE_VELOCITY = 1404
    LOCKING_PINS_RESET_ALARM = 1405
    LOCKING_PINS_MOVE_ALL = 1406
    # The "locks" actually act as guides and supports, not locks.
    MIRROR_COVER_LOCKS_POWER = 1501
    MIRROR_COVER_LOCKS_STOP = 1502
    MIRROR_COVER_LOCKS_MOVE = 1503
    MIRROR_COVER_LOCKS_MOVE_VELOCITY = 1504
    MIRROR_COVER_LOCKS_RESET_ALARM = 1505
    MIRROR_COVER_LOCKS_MOVE_ALL = 1506
    MIRROR_COVER_LOCKS_LOCK = 1507
    MIRROR_COVER_LOCKS_UNLOCK = 1508
    AZIMUTH_DRIVES_THERMAL_POWER = 1601
    AZIMUTH_DRIVES_THERMAL_CONTROL_MODE = 1602
    AZIMUTH_DRIVES_THERMAL_RESET_ALARM = 1603
    ELEVATION_DRIVES_THERMAL_POWER = 1701
    ELEVATION_DRIVES_THERMAL_CONTROL_MODE = 1702
    ELEVATION_DRIVES_THERMAL_RESET_ALARM = 1703
    # Reset safety interlock.
    SAFETY_RESET = 1801
    # Override safety interlock.
    OVERRIDE_CAUSES = 1802
    CABINET_0101_THERMAL_POWER = 1901
    CABINET_0101_THERMAL_CONTROL_MODE = 1902
    CABINET_0101_THERMAL_RESET_ALARM = 1903
    STATE_OF_OPERATION_MANAGER = 2001  # NOT ACKED!
    # Ask the Operation Manager to exit
    APPLICATION_EXIT = 2002
    # "Internal command for Operation Manager"
    ASK_FOR_COMMAND = 2103
    # The top end chiller commands may change,
    # once control is implemented.
    TOP_END_CHILLER_POWER = 2201
    TOP_END_CHILLER_TRACK_AMBIENT = 2202
    TOP_END_CHILLER_RESET_ALARM = 2203
    TRANSFER_FUNCTION_AZIMUTH_EXCITATION = 2301
    TRANSFER_FUNCTION_ELEVATION_EXCITATION = 2302
    # Get actual set of settings to send them to TCS.
    GET_AVAILABLE_SETTING_SETS = 2401
    GET_ACTUAL_SETTINGS = 2402
    APPLY_SETTINGS_SET = 2403
    STATE_INFO = 2502
    # Modbus temperature controller.
    MODBUS_CABINETS_THERMAL_RESET_ALARM = 2601
    MODBUS_CABINETS_THERMAL_SETPOINT = 2602
    MODBUS_CABINETS_THERMAL_FAN_POWER = 2603
    # Send HEARTBEAT once a second or so. It will get no ack of any kind.
    # This command is called CLOCK in Tekniker's documentation.
    HEARTBEAT = 3000


class CscErrorCode(enum.IntEnum):
    COULD_NOT_CONNECT = 1  # Could not connect to low-level controller
    CONNECTION_LOST = 2  # Lost connection to low-level controller
    TELEMETRY_CLIENT_ERROR = 3  # Telemetry client failed
    AXIS_FAULT = 4  # One or more axes faulted
    TRACK_TARGET_TIMED_OUT = 5  # A main axes tracking command timed out
    DISABLE_FAILED = 6  # Could not disable the CSC
    COMMAND_LOST = 7  # Control taken away from the CSC by the EUI or HHD
    MOCK_CONTROLLER_ERROR = 98
    INTERNAL_ERROR = 99


class EnabledState(enum.IntEnum):
    """Status of enabling/disabling subsystems."""

    DISABLED = enum.auto()
    ENABLED = enum.auto()
    ENABLING = enum.auto()
    DISABLING = enum.auto()
    DISABLE_FAILED = enum.auto()
    ENABLE_FAILED = enum.auto()


class ReplyId(enum.IntEnum):
    """Reply codes for messages read from the low-level controller.

    The values and names are from email from Julen 2021-02-19.
    There will presumably be something in a .h file.
    """

    CMD_ACKNOWLEDGED = 1
    """Command accepted and begun.

    For commands in AckOnlyCommandCodes: the command succeeded
    and you will see no more CMD_x replies for it.
    For all other commands: you will see one more CMD_x reply.
    """

    CMD_REJECTED = 2
    """Command failed before being acknowledged."""

    CMD_SUCCEEDED = 3
    """Command succeeded."""

    CMD_FAILED = 4
    """Command failed after being acknowledged."""

    CMD_SUPERSEDED = 5
    """Command superseded after being acknowledged."""

    WARNING = 10
    ERROR = 11  # Called alarm in the TMA docs
    COMMANDER = 20
    SAFETY_INTERLOCKS = 30
    DETAILED_SETTINGS_APPLIED = 40
    AVAILABLE_SETTINGS = 41

    POWER_STATE = 100
    AXIS_MOTION_STATE = 101
    OIL_SUPPLY_SYSTEM_STATE = 102
    CHILLER_STATE = 103
    MOTION_CONTROLLER_STATE = 104
    IN_POSITION = 200
    ELEVATION_LOCKING_PIN_MOTION_STATE = 201
    MIRROR_COVERS_MOTION_STATE = 202
    MIRROR_COVER_LOCKS_MOTION_STATE = 203
    DEPLOYABLE_PLATFORMS_MOTION_STATE = 204
    HOMED = 205
    LIMITS = 300
    SPECIAL_LIMITS = 301
    AZIMUTH_TOPPLE_BLOCK = 304
    AZIMUTH_CABLE_WRAP_SWITCHES = 305


class Source(enum.IntEnum):
    """Values for the ``source`` field of a message and the AskForCommand
    command."""

    NONE = 0
    CSC = 1
    EUI = 2
    HHD = 3
    PXI = 100


class ThermalMode(enum.IntEnum):
    """Thermal control mode for azimuth, elevation, and cabinet0101.

    The modes used in the following commands:

    * AZIMUTH_DRIVES_THERMAL_CONTROL_MODE,
    * ELEVATION_DRIVES_THERMAL_CONTROL_MODE
    * CABINET_0101_THERMAL_CONTROL_MODE

    The modes are as follows, along with the additional parameters
    needed for the command:

    * TRACK_AMBIENT: track ambient temperature measured by the subsystem.
      No additional parameters.
    * TRACK_SETPOINT: track a specified temperature.
      One additional parameter: the desired temperature (C).
    * SET_VALVE_POSITION: set valve position
      One additional parameter: valve position (%).
    * SET_AUTOTUNE: start or stop auto-tune.
      One additional parameter: 1 = start, 0 = stop.

    """

    TRACK_AMBIENT = 0
    TRACK_SETPOINT = 1
    SET_VALVE_POSITION = 2
    SET_AUTOTUNE = 3
