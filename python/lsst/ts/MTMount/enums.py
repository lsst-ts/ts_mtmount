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

__all__ = ["CableWrap", "CommandCode", "Device", "ReplyCode", "Source"]

import enum


class CableWrap(enum.IntEnum):
    """Cable wrap directions."""

    CW = 0
    CCW = 1


class CommandCode(enum.IntEnum):
    """Command codes for Command.command_code.

    The values and names are from COMMAND_TYPE in types.h,
    with abbreviations expanded.
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
    HUMAN_MACHINE_INTERFACE_MOVE_TO_TARGET = 30
    HUMAN_MACHINE_INTERFACE_TRACK_TARGET = 31
    HUMAN_MACHINE_INTERFACE_STOP = 32
    AZIMUTH_AXIS_POWER = 101
    AZIMUTH_AXIS_STOP = 102
    AZIMUTH_AXIS_MOVE = 103
    AZIMUTH_AXIS_MOVE_VELOCITY = 104
    AZIMUTH_AXIS_TRACKING = 105
    AZIMUTH_AXIS_HOME = 106
    AZIMUTH_AXIS_RESET_ALARM = 107
    AZIMUTH_DRIVE_RESET = 201
    AZIMUTH_DRIVE_ENABLE = 202
    AZIMUTH_CABLE_WRAP_POWER = 301
    AZIMUTH_CABLE_WRAP_STOP = 302
    AZIMUTH_CABLE_WRAP_MOVE = 303
    AZIMUTH_CABLE_WRAP_MOVE_VELOCITY = 304
    AZIMUTH_CABLE_WRAP_TRACK_AZIMUTH = 305
    AZIMUTH_CABLE_WRAP_RESET_ALARM = 306
    AZIMUTH_CABLE_WRAP_DRIVE_RESET = 307
    AZIMUTH_CABLE_WRAP_DRIVE_ENABLE = 308
    AZIMUTH_CABLE_WRAP_ENABLE_TRACK_AZIMUTH = 309
    ELEVATION_AXIS_POWER = 401
    ELEVATION_AXIS_STOP = 402
    ELEVATION_AXIS_MOVE = 403
    ELEVATION_AXIS_MOVE_VELOCITY = 404
    ELEVATION_AXIS_TRACKING = 405
    ELEVATION_AXIS_HOME = 406
    ELEVATION_AXIS_RESET_ALARM = (407,)
    ELEVATION_DRIVE_RESET = 501
    ELEVATION_DRIVE_ENABLE = 502
    MAIN_POWER_SUPPLY_POWER = 601
    MAIN_POWER_SUPPLY_RESET_ALARM = 602
    ENCODER__POWER = 701
    ENCODER__REFERENCE = 702
    ENCODER__RESET = 703
    ENCODER__RESET_ERROR = 704
    ENCODER__CLEAR_POSITION_ERROR = (705,)
    ENCODER__EXIT = (706,)
    OIL_SUPPLY_SYSTEM_POWER = 801
    OIL_SUPPLY_SYSTEM_POWER_COOLING = 802
    OIL_SUPPLY_SYSTEM_POWER_OIL = 803
    OIL_SUPPLY_SYSTEM_POWER_MAIN_PUMP = 804
    OIL_SUPPLY_SYSTEM_RESET_ALARM = 805
    MIRROR_COVER_POWER = 901
    MIRROR_COVER_STOP = 902
    MIRROR_COVER_MOVE = 903
    MIRROR_COVER_MOVE_VELOCITY = 904
    MIRROR_COVER_OPEN = 905
    MIRROR_COVER_CLOSE = 906
    MIRROR_COVER_RESET_ALARM = 907
    CAMERA_CABLE_WRAP_POWER = 1001
    CAMERA_CABLE_WRAP_STOP = 1002
    CAMERA_CABLE_WRAP_MOVE = 1003
    CAMERA_CABLE_WRAP_TRACK_CAMERA = 1004
    CAMERA_CABLE_WRAP_RESET_ALARM = 1005
    CAMERA_CABLE_WRAP_DRIVE_ENABLE = 1006
    CAMERA_CABLE_WRAP_DRIVE_RESET = 1007
    CAMERA_CABLE_WRAP_MOVE_VELOCITY = 1008
    CAMERA_CABLE_WRAP_ENABLE_TRACK_CAMERA = 1009
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
    MIRROR_COVER_LOCK_POWER = 1501
    MIRROR_COVER_LOCK_STOP = 1502
    MIRROR_COVER_LOCK_MOVE = 1503
    MIRROR_COVER_LOCK_MOVE_VELOCITY = 1504
    MIRROR_COVER_LOCK_RESET_ALARM = 1505
    MIRROR_COVER_LOCK_MOVE_ALL = 1506
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


class Device(enum.Enum):
    """Devices that have POWER and RESET_ALARM commands.
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
    MIRROR_COVER = enum.auto()
    MIRROR_COVER_LOCK = enum.auto()
    OIL_SUPPLY_SYSTEM = enum.auto()


class ReplyCode(enum.IntEnum):
    """Reply codes for messages read from the Operation Manager.

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


class Source(enum.IntEnum):
    """Values for the ``source`` field of a message.
    """

    TCS = 1
    MCS = 2
    HHD = 3
    PXI = 100
