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
    "CommandCode",
    "CscErrorCode",
    "DeviceId",
    "EnabledState",
    "ReplyCode",
    "Commander",
    "Source",
    "SubsystemId",
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
    # Control both the mirror cover and mirror cover lock
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
    BOTH_AXES_MOVE = 30  # HMI_MOVE_TO_TARGET in types.h
    BOTH_AXES_TRACK = 31  # HMI_TRACK_TARGET in types.h
    BOTH_AXES_STOP = 32  # HMI_STOP in types.h
    AZIMUTH_AXIS_POWER = 101
    AZIMUTH_AXIS_STOP = 102
    AZIMUTH_AXIS_MOVE = 103
    AZIMUTH_AXIS_MOVE_VELOCITY = 104
    # AZ_AXIS_TRACKING in types.h
    AZIMUTH_AXIS_TRACK = 105
    AZIMUTH_AXIS_HOME = 106
    AZIMUTH_AXIS_RESET_ALARM = 107
    # Not sure what the name will be; not yet implemented
    AZIMUTH_AXIS_ENABLE_TRACKING = 108
    AZIMUTH_AXIS_DRIVE_RESET = 201
    AZIMUTH_AXIS_DRIVE_ENABLE = 202
    AZIMUTH_CABLE_WRAP_POWER = 301
    AZIMUTH_CABLE_WRAP_STOP = 302
    AZIMUTH_CABLE_WRAP_MOVE = 303
    AZIMUTH_CABLE_WRAP_MOVE_VELOCITY = 304
    # AZ_CW_TRACK_AZIMUTH in types.h
    AZIMUTH_CABLE_WRAP_TRACK = 305
    AZIMUTH_CABLE_WRAP_RESET_ALARM = 306
    AZIMUTH_CABLE_WRAP_DRIVE_RESET = 307
    AZIMUTH_CABLE_WRAP_DRIVE_ENABLE = 308
    # AZ_CW_ENABLE_TRACK_AZIMUTH in types.h
    AZIMUTH_CABLE_WRAP_ENABLE_TRACKING = 309
    ELEVATION_AXIS_POWER = 401
    ELEVATION_AXIS_STOP = 402
    ELEVATION_AXIS_MOVE = 403
    ELEVATION_AXIS_MOVE_VELOCITY = 404
    # EL_AXIS_TRACKING in types.h
    ELEVATION_AXIS_TRACK = 405
    ELEVATION_AXIS_HOME = 406
    ELEVATION_AXIS_RESET_ALARM = 407
    # Not sure what the name will be; not yet implemented
    ELEVATION_AXIS_ENABLE_TRACKING = 408
    ELEVATION_AXIS_DRIVE_RESET = 501
    ELEVATION_AXIS_DRIVE_ENABLE = 502
    MAIN_POWER_SUPPLY_POWER = 601
    MAIN_POWER_SUPPLY_RESET_ALARM = 602
    ENCODER_INTERFACE_BOX_POWER = 701
    ENCODER_INTERFACE_BOX_REFERENCE = 702
    ENCODER_INTERFACE_BOX_RESET = 703
    ENCODER_INTERFACE_BOX_RESET_ERROR = 704
    ENCODER_INTERFACE_BOX_CLEAR_POSITION_ERROR = 705
    ENCODER_INTERFACE_BOX_EXIT = 706
    OIL_SUPPLY_SYSTEM_POWER = 801
    OIL_SUPPLY_SYSTEM_POWER_COOLING = 802
    OIL_SUPPLY_SYSTEM_POWER_OIL = 803
    OIL_SUPPLY_SYSTEM_POWER_MAIN_PUMP = 804
    OIL_SUPPLY_SYSTEM_RESET_ALARM = 805
    MIRROR_COVERS_POWER = 901
    MIRROR_COVERS_STOP = 902
    MIRROR_COVERS_MOVE = 903
    MIRROR_COVERS_MOVE_VELOCITY = 904
    # WARNING: Tekniker's code presently uses:
    # OPEN to mean DEPLOY, which we call closing the mirror covers
    # CLOSE to mean RETRACT, which we call opening the mirror covers.
    # Tekniker will update their code to use deploy and retract.
    MIRROR_COVERS_DEPLOY = 905
    MIRROR_COVERS_RETRACT = 906
    MIRROR_COVERS_RESET_ALARM = 907
    CAMERA_CABLE_WRAP_POWER = 1001
    CAMERA_CABLE_WRAP_STOP = 1002
    CAMERA_CABLE_WRAP_MOVE = 1003
    CAMERA_CABLE_WRAP_TRACK = 1004
    CAMERA_CABLE_WRAP_RESET_ALARM = 1005
    CAMERA_CABLE_WRAP_DRIVE_ENABLE = 1006
    CAMERA_CABLE_WRAP_DRIVE_RESET = 1007
    CAMERA_CABLE_WRAP_MOVE_VELOCITY = 1008
    CAMERA_CABLE_WRAP_ENABLE_TRACKING = 1009
    BALANCE_POWER = 1101
    BALANCE_STOP = 1102
    BALANCE_MOVE = 1103
    BALANCE_RESET_ALARM = 1104
    TOP_END_CHILLER_POWER = 2201
    TOP_END_CHILLER_TRACK_AMBIENT = 2202
    TOP_END_CHILLER_RESET_ALARM = 2203
    DEPLOYABLE_PLATFORM_POWER = 1201
    DEPLOYABLE_PLATFORM_STOP = 1202
    DEPLOYABLE_PLATFORM_MOVE_VELOCITY = 1204
    DEPLOYABLE_PLATFORM_RESET_ALARM = 1205
    DEPLOYABLE_PLATFORM_LOCK_EXTENSION = 1206
    # Main cabinet temperature controller
    CABINET_TRACK_AMBIENT = 1301
    CABINET_RESET_ALARM = 1302
    LOCKING_PINS_POWER = 1401
    LOCKING_PINS_STOP = 1402
    LOCKING_PINS_MOVE = 1403
    LOCKING_PINS_MOVE_VELOCITY = 1404
    LOCKING_PINS_RESET_ALARM = 1405
    LOCKING_PINS_MOVE_ALL = 1406
    # These are more guides or supports than actual locks
    MIRROR_COVER_LOCKS_POWER = 1501
    MIRROR_COVER_LOCKS_STOP = 1502
    MIRROR_COVER_LOCKS_MOVE = 1503
    MIRROR_COVER_LOCKS_MOVE_VELOCITY = 1504
    MIRROR_COVER_LOCKS_RESET_ALARM = 1505
    MIRROR_COVER_LOCKS_MOVE_ALL = 1506
    AZIMUTH_THERMAL_POWER = 1601
    AZIMUTH_THERMAL_TRACK_AMBIENT = 1602
    AZIMUTH_THERMAL_RESET_ALARM = 1603
    ELEVATION_THERMAL_POWER = 1701
    ELEVATION_THERMAL_TRACK_AMBIENT = 1702
    ELEVATION_THERMAL_RESET_ALARM = 1703
    # Reset safety interlock
    SAFETY_RESET = 1801
    # Override safety interlock
    OVERRIDE_CAUSES = 1802
    STATE_INFO = 2001
    # Ask the Operation Manager to exit
    APPLICATION_EXIT = 2002
    # "Internal command for Operation Manager"
    ASK_FOR_COMMAND = 2101
    TRANSFER_FUNCTION_AZIMUTH_EXCITATION = 2301
    TRANSFER_FUNCTION_ELEVATION_EXCITATION = 2302
    # Get actual set of settings to send them to TCS.
    ASK_FOR_SET_OF_SETTINGS = 2401
    PXI_COMMAND_DONE = -1
    ERROR = -100


class CscErrorCode(enum.IntEnum):
    COULD_NOT_CONNECT = 1
    CONNECTION_LOST = 2
    ACTUATOR_ENABLE_ERROR = 3
    ACTUATOR_DISABLE_ERROR = 4
    MOCK_CONTROLLER_ERROR = 98
    INTERNAL_ERROR = 99


class DeviceId(enum.Enum):
    """Devices that have POWER and RESET_ALARM commands.

    These are for mock devices. The values do not match
    anything in Tekniker's code.

    We could use SubsystemId, instead, but that enum class includes entries
    that have no POWER or RESET_ALARM commands.
    """

    ELEVATION_AXIS = enum.auto()
    AZIMUTH_AXIS = enum.auto()
    AZIMUTH_CABLE_WRAP = enum.auto()
    CAMERA_CABLE_WRAP = enum.auto()
    BALANCE = enum.auto()
    DEPLOYABLE_PLATFORM = enum.auto()
    ENCODER_ = enum.auto()
    TOP_END_CHILLER = enum.auto()
    LOCKING_PIN = enum.auto()
    MAIN_POWER_SUPPLY = enum.auto()
    MIRROR_COVERS = enum.auto()
    MIRROR_COVER_LOCKS = enum.auto()
    OIL_SUPPLY_SYSTEM = enum.auto()


class EnabledState(enum.IntEnum):
    """Status of enabling/disabling subsystems.
    """

    DISABLED = enum.auto()
    ENABLED = enum.auto()
    ENABLING = enum.auto()
    DISABLING = enum.auto()
    DISABLE_FAILED = enum.auto()
    ENABLE_FAILED = enum.auto()


class ReplyCode(enum.IntEnum):
    """Reply codes for messages read from the low-level controller.

    The values and names are from LL_MESSAGE in ll_commands.h.
    except that ON_STATE_INFO is hard-coded in ``eui_communication.cpp``
    in ``EUICommunication::onStateInfo`` and replaces the obsolete
    SETTINGS from ll_commands.h.
    """

    ACK = 0
    NOACK = 1
    DONE = 2
    ERROR = 3
    WARNING = 4
    ON_STATE_INFO = 5
    IN_POSITION = 6


class Commander(enum.IntEnum):
    """Values for the AskForCommand command.

    Note: these values may change in Tekniker's upgrade.
    1 was originally TCS (Tekniker's CSC).
    """

    NONE = 0
    CSC = 1
    EUI = 2
    HHD = 3


class Source(enum.IntEnum):
    """Values for the ``source`` field of a message.

    Note: these values may change in Tekniker's upgrade.
    1 was originally TCS (Tekniker's CSC).
    """

    CSC = 1
    EUI = 2
    HHD = 3
    PXI = 100


class SubsystemId(enum.IntEnum):
    """Subsystem ID numbers.

    Used in errors and warnings.
    """

    AZIMUTH_AXIS = 100
    AZIMUTH_DRIVE = 200
    AZIMUTH_CABLE_WRAP = 300
    ELEVATION_AXIS = 400
    ELEVATION_DRIVE = 500
    MAIN_POWER_SUPPLY = 600
    ENCODER_INTERFACE_BOX = 700
    OIL_SUPPLY_SYSTEM = 800
    MIRROR_COVERS = 900
    CAMERA_CABLE_WRAP = 1000
    BALANCE = 1100
    DEPLOYABLE_PLATFORM = 1200
    CABINET = 1300
    LOCKING_PINS = 1400
    MIRROR_COVER_LOCKS = 1500
    AZIMUTH_THERMAL = 1600
    ELEVATION_THERMAL = 1700
    INTERLOCK = 1800
    TOP_END_CHILLER = 2200
