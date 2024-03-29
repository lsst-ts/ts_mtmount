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

__all__ = ["detailed_settings"]

import yaml

detailed_settings = yaml.safe_load(
    r"""
setName: Default
allSettingsMatchSet: true
Balancing:
  Balancing1:
    BalancePosition: 400.02
    DefaultAcceleration: 1500
    DefaultJerk: 3000
    DefaultSpeed: 25
    FirstSidePosition: -4
    IdString: 0. X-
    LastSidePosition: 1295
    MaxCriticalSpeed: 32
    MaxPosition: 1300
    MaxPositionLimit: 1300
    MaxSpeed: 31
    MinPosition: -6
    MinPositionLimit: -6
    MotorId: 15
    MovevelocityTrnsTimeout: -1
    MoveTransTimemargin: 150
    OfftransTimeout: 10000
    OntransTimeout: 10000
    Resettingtime: 3000
    Slewthreshold: 0
    SoftwareLimitEnable: true
    SpeedLimit: 31
    StoptransTimeout: 3000
    Waitafterreset: 0
    Systemsource: 1100.BAL
  Balancing2:
    BalancePosition: 300
    DefaultAcceleration: 1500
    DefaultJerk: 3000
    DefaultSpeed: 25
    FirstSidePosition: -4
    IdString: 0. X+
    LastSidePosition: 1298
    MaxCriticalSpeed: 32
    MaxPosition: 1300
    MaxPositionLimit: 1300
    MaxSpeed: 31
    MinPosition: -10
    MinPositionLimit: -10
    MotorId: 13
    MovevelocityTrnsTimeout: -1
    MoveTransTimemargin: 150
    OfftransTimeout: 10000
    OntransTimeout: 10000
    Resettingtime: 3000
    Slewthreshold: 0
    SoftwareLimitEnable: true
    SpeedLimit: 31
    StoptransTimeout: 3000
    Waitafterreset: 0
    Systemsource: 1100.BAL
  Balancing3:
    BalancePosition: 200
    DefaultAcceleration: 1500
    DefaultJerk: 3000
    DefaultSpeed: 25
    FirstSidePosition: 0
    IdString: 90. X-
    LastSidePosition: 1080
    MaxCriticalSpeed: 32
    MaxPosition: 1085
    MaxPositionLimit: 1085
    MaxSpeed: 31
    MinPosition: -3
    MinPositionLimit: -3
    MotorId: 16
    MovevelocityTrnsTimeout: -1
    MoveTransTimemargin: 150
    OfftransTimeout: 10000
    OntransTimeout: 10000
    Resettingtime: 3000
    Slewthreshold: 0
    SoftwareLimitEnable: true
    SpeedLimit: 31
    StoptransTimeout: 3000
    Waitafterreset: 0
    Systemsource: 1100.BAL
  Balancing4:
    BalancePosition: 100
    DefaultAcceleration: 1500
    DefaultJerk: 3000
    DefaultSpeed: 25
    FirstSidePosition: 0
    IdString: 90. X+
    LastSidePosition: 1080
    MaxCriticalSpeed: 32
    MaxPosition: 1085
    MaxPositionLimit: 1085
    MaxSpeed: 31
    MinPosition: -3
    MinPositionLimit: -3
    MotorId: 14
    MovevelocityTrnsTimeout: -1
    MoveTransTimemargin: 150
    OfftransTimeout: 10000
    OntransTimeout: 10000
    Resettingtime: 3000
    Slewthreshold: 0
    SoftwareLimitEnable: true
    SpeedLimit: 31
    StoptransTimeout: 3000
    Waitafterreset: 0
    Systemsource: 1100.BAL
BoschSystem:
  Address: 192.168.209.3
  Changetoparking: false
  ParkingMotorId1: false
  ParkingMotorId10: false
  ParkingMotorId11: false
  ParkingMotorId12: false
  ParkingMotorId13: false
  ParkingMotorId14: false
  ParkingMotorId15: false
  ParkingMotorId16: false
  ParkingMotorId17: false
  ParkingMotorId18: false
  ParkingMotorId19: false
  ParkingMotorId2: false
  ParkingMotorId20: false
  ParkingMotorId21: false
  ParkingMotorId22: false
  ParkingMotorId3: false
  ParkingMotorId4: false
  ParkingMotorId5: false
  ParkingMotorId6: false
  ParkingMotorId7: false
  ParkingMotorId8: false
  ParkingMotorId9: false
  Password: administrator
  User: administrator
CW:
  ACW:
    AzimuthDeviation: 5
    AzimuthDeviationOverride: true
    CriticalAzimuthDeviation: 8
    CriticalSpeedLimit: 15
    DefaultAcceleration: 15
    DefaultJerk: 70
    DefaultSpeed: 12
    ForceDriveSelection: false
    ForcedDrive: 1
    IdString1: Motor 1
    IdString2: Motor 2
    MaxAcceleration: 11
    MaxJerk: 48
    MaxPosition: 280
    MaxSoftwareLimit: 281
    MaxSpeed: 14
    MinPosition: -280
    MinSoftwareLimit: -281
    Motor1Id: 1
    Motor2Id: 2
    MotorTimerFilePath: 'c:\Configuration{\ACWData.ini'
    MoveTransTimemargin: 140
    OfftransTimeout: 5000
    OntransTimeout: 10000
    PositionOffset: 0
    Resettingtime: 2000
    RotationSense: 1
    SoftwareLimitEnable: true
    SpeedLimit: 12
    StoptransTimeout: 4000
    Systemsource: 300.ACW
    TrackingAcceleration: 10
    TrackingJerk: 48
    TrackingQueueSize: 3000
    TrackingSpeed: 10
    TrackingTimeOffset: 0
    TrackingWaitForDataBeforeError: 1300
    TrackingWaitForFirstTrackSetpointError: 500
    TrackingWaitTimeForCheckSetpoint: 5000
    TrackingWaitTimeIfNoDataInQueue: 500
    Waitafterreset: 100
  CCW:
    CameraDeviation: 0.9
    CameraDeviationOverride: true
    CriticalAzimuthDeviation: 1
    CriticalSpeedLimit: 6
    DefaultAcceleration: 1
    DefaultJerk: 6
    DefaultSpeed: 3.5
    ForceDriveSelection: false
    ForcedDrive: 1
    IdString1: Motor 1
    IdString2: Motor 2
    Inpositionmargin: 0.2
    NegativeLimitSwitchEnable: true
    PositiveLimitSwitchEnable: true
    MaxAcceleration: 1
    MaxJerk: 7
    MaxPosition: 90
    MaxSoftwareLimit: 91
    MaxSpeed: 5.6
    MinPosition: -90
    MinSoftwareLimit: -91
    Motor1Id: 3
    Motor2Id: 4
    MotorTimerFilePath: 'c:\Configuration\CCWData.ini'
    MoveTransTimemargin: 140
    OfftransTimeout: 5000
    OntransTimeout: 10000
    PositionOffset: 0
    Resettingtime: 2000
    RotationSense: 1
    NegativeSoftwareLimitEnable: true
    PositiveSoftwareLimitEnable: true
    SpeedLimit: 6
    StoptransTimeout: 4000
    Systemsource: 1000.CCW
    TrackingAcceleration: 1
    TrackingJerk: 6
    TrackingQueueSize: 3000
    TrackingSpeed: 3.5
    TrackingTimeOffset: 0
    TrackingWaitForDataBeforeError: 230
    TrackingWaitForFirstTrackSetpointError: 500
    TrackingWaitTimeForCheckSetpoint: 5000
    TrackingWaitTimeIfNoDataInQueue: 500
    Waitafterreset: 100
DeployablePlatform:
  DeployablePlatform1:
    ErrorTimeoutMargin: 1.5
    IdString: DP X+
    MaxPlatformPosition1: 2345
    MaxPlatformPosition2: 2415
    MinPlatformPosition1: -5
    MinPlatformPosition2: -5
    MotorIdPlatform1: 17
    MotorIdPlatform2: 18
    PlatformAcceleration: 98
    PlatformDeceleration: 98
    PlatformJerk: 100
    PlatformMaxCriticalVelocity: 110
    PlatformMaxVelocity: 105
    PlatformVelocity: 98
    Timeoutlockextension: 20000
    Timeoutmovemargin: 150
    Timeoutpoweroff: 3000
    Timeoutpoweron: 3000
    Timeoutreset: 3000
    Timeoutstop: 3000
    Enableextensionlockguard: true
    MaxElevationAngle: 10
    MinElevationAngle: -10
    Systemsource: 1200.DP
  DeployablePlatform2:
    ErrorTimeoutMargin: 1.5
    IdString: DP X-
    MaxPlatformPosition1: 2345
    MaxPlatformPosition2: 2415
    MinPlatformPosition1: -5
    MinPlatformPosition2: -5
    MotorIdPlatform1: 21
    MotorIdPlatform2: 22
    PlatformAcceleration: 98
    PlatformDeceleration: 98
    PlatformJerk: 100
    PlatformMaxCriticalVelocity: 110
    PlatformMaxVelocity: 105
    PlatformVelocity: 98
    Timeoutlockextension: 20000
    Timeoutmovemargin: 150
    Timeoutpoweroff: 3000
    Timeoutpoweron: 3000
    Timeoutreset: 3000
    Timeoutstop: 3000
    Enableextensionlockguard: true
    MaxElevationAngle: 10
    MinElevationAngle: -10
    Systemsource: 1200.DP
EncoderSystem:
  Head1:
    AzimuthCriticalActiveHeads: 2
    'AzimuthHeadsReferenceThreshold[i.u]': 10
    AzimuthTapeLineCount: 1243770
    AzimuthTelescopeOffset: 0
    CheckUpdTimeout: 4000
    CmdTimeoutClearErrors: 5000
    CmdTimeoutClearHeadsErrors: 2000
    CmdTimeoutPowerOff: 6000
    CmdTimeoutPowerOn: 9000
    CmdTimeoutReboot: 110000
    CmdTimeoutReferenceOff: 4000
    CmdTimeoutReferenceOn: 10000
    CmdTimeoutRelativeOffset: 2000
    EibConfigFilePath: /c/Configuration/EIB/multi_ext.txt
    ElevationCriticalActiveHeads: 2
    'ElevationHeadsReferenceThreshold[i.u]': 10
    ElevationTapeLineCount: 0
    ElevationTelescopeOffset: 0
    FpgaClockRate: 40
    LoggingIp: 192.168.211.10
    SyncTriggerOffset: 365
    SyncTriggerOnTime: 80
    UdpReadingTimeout: 10
    WaitAfterReset: 60
    AzimuthAxis: true
    EibInputName: 'SLOT01:AXIS01'
    HeadName: TMA-AZ-ENC-AZM-0004
    NsvLinkid: 4
    PositionGain: 0
    PositionOffset: 932828
  Head2:
    AzimuthCriticalActiveHeads: 2
    'AzimuthHeadsReferenceThreshold[i.u]': 10
    AzimuthTapeLineCount: 1243770
    AzimuthTelescopeOffset: 0
    CheckUpdTimeout: 4000
    CmdTimeoutClearErrors: 5000
    CmdTimeoutClearHeadsErrors: 2000
    CmdTimeoutPowerOff: 6000
    CmdTimeoutPowerOn: 9000
    CmdTimeoutReboot: 110000
    CmdTimeoutReferenceOff: 4000
    CmdTimeoutReferenceOn: 10000
    CmdTimeoutRelativeOffset: 2000
    EibConfigFilePath: /c/Configuration/EIB/multi_ext.txt
    ElevationCriticalActiveHeads: 2
    'ElevationHeadsReferenceThreshold[i.u]': 10
    ElevationTapeLineCount: 0
    ElevationTelescopeOffset: 0
    FpgaClockRate: 40
    LoggingIp: 192.168.211.10
    SyncTriggerOffset: 365
    SyncTriggerOnTime: 80
    UdpReadingTimeout: 10
    WaitAfterReset: 60
    AzimuthAxis: false
    EibInputName: 'SLOT01:AXIS02'
    HeadName: TMA-AZ-ENC-ELV-0003
    NsvLinkid: 3
    PositionGain: 0
    PositionOffset: 0
  Head3:
    AzimuthCriticalActiveHeads: 2
    'AzimuthHeadsReferenceThreshold[i.u]': 10
    AzimuthTapeLineCount: 1243770
    AzimuthTelescopeOffset: 0
    CheckUpdTimeout: 4000
    CmdTimeoutClearErrors: 5000
    CmdTimeoutClearHeadsErrors: 2000
    CmdTimeoutPowerOff: 6000
    CmdTimeoutPowerOn: 9000
    CmdTimeoutReboot: 110000
    CmdTimeoutReferenceOff: 4000
    CmdTimeoutReferenceOn: 10000
    CmdTimeoutRelativeOffset: 2000
    EibConfigFilePath: /c/Configuration/EIB/multi_ext.txt
    ElevationCriticalActiveHeads: 2
    'ElevationHeadsReferenceThreshold[i.u]': 10
    ElevationTapeLineCount: 0
    ElevationTelescopeOffset: 0
    FpgaClockRate: 40
    LoggingIp: 192.168.211.10
    SyncTriggerOffset: 365
    SyncTriggerOnTime: 80
    UdpReadingTimeout: 10
    WaitAfterReset: 60
    AzimuthAxis: true
    EibInputName: 'SLOT02:AXIS01'
    HeadName: TMA-AZ-ENC-AZM-0003
    NsvLinkid: 3
    PositionGain: 0
    PositionOffset: 621885
  Head4:
    AzimuthCriticalActiveHeads: 2
    'AzimuthHeadsReferenceThreshold[i.u]': 10
    AzimuthTapeLineCount: 1243770
    AzimuthTelescopeOffset: 0
    CheckUpdTimeout: 4000
    CmdTimeoutClearErrors: 5000
    CmdTimeoutClearHeadsErrors: 2000
    CmdTimeoutPowerOff: 6000
    CmdTimeoutPowerOn: 9000
    CmdTimeoutReboot: 110000
    CmdTimeoutReferenceOff: 4000
    CmdTimeoutReferenceOn: 10000
    CmdTimeoutRelativeOffset: 2000
    EibConfigFilePath: /c/Configuration/EIB/multi_ext.txt
    ElevationCriticalActiveHeads: 2
    'ElevationHeadsReferenceThreshold[i.u]': 10
    ElevationTapeLineCount: 0
    ElevationTelescopeOffset: 0
    FpgaClockRate: 40
    LoggingIp: 192.168.211.10
    SyncTriggerOffset: 365
    SyncTriggerOnTime: 80
    UdpReadingTimeout: 10
    WaitAfterReset: 60
    AzimuthAxis: false
    EibInputName: 'SLOT02:AXIS02'
    HeadName: TMA-AZ-ENC-ELV-0001
    NsvLinkid: 1
    PositionGain: -0.0
    PositionOffset: 0
  Head5:
    AzimuthCriticalActiveHeads: 2
    'AzimuthHeadsReferenceThreshold[i.u]': 10
    AzimuthTapeLineCount: 1243770
    AzimuthTelescopeOffset: 0
    CheckUpdTimeout: 4000
    CmdTimeoutClearErrors: 5000
    CmdTimeoutClearHeadsErrors: 2000
    CmdTimeoutPowerOff: 6000
    CmdTimeoutPowerOn: 9000
    CmdTimeoutReboot: 110000
    CmdTimeoutReferenceOff: 4000
    CmdTimeoutReferenceOn: 10000
    CmdTimeoutRelativeOffset: 2000
    EibConfigFilePath: /c/Configuration/EIB/multi_ext.txt
    ElevationCriticalActiveHeads: 2
    'ElevationHeadsReferenceThreshold[i.u]': 10
    ElevationTapeLineCount: 0
    ElevationTelescopeOffset: 0
    FpgaClockRate: 40
    LoggingIp: 192.168.211.10
    SyncTriggerOffset: 365
    SyncTriggerOnTime: 80
    UdpReadingTimeout: 10
    WaitAfterReset: 60
    AzimuthAxis: true
    EibInputName: 'SLOT03:AXIS01'
    HeadName: TMA-AZ-ENC-AZM-0001
    NsvLinkid: 1
    PositionGain: 0
    PositionOffset: 0
  Head6:
    AzimuthCriticalActiveHeads: 2
    'AzimuthHeadsReferenceThreshold[i.u]': 10
    AzimuthTapeLineCount: 1243770
    AzimuthTelescopeOffset: 0
    CheckUpdTimeout: 4000
    CmdTimeoutClearErrors: 5000
    CmdTimeoutClearHeadsErrors: 2000
    CmdTimeoutPowerOff: 6000
    CmdTimeoutPowerOn: 9000
    CmdTimeoutReboot: 110000
    CmdTimeoutReferenceOff: 4000
    CmdTimeoutReferenceOn: 10000
    CmdTimeoutRelativeOffset: 2000
    EibConfigFilePath: /c/Configuration/EIB/multi_ext.txt
    ElevationCriticalActiveHeads: 2
    'ElevationHeadsReferenceThreshold[i.u]': 10
    ElevationTapeLineCount: 0
    ElevationTelescopeOffset: 0
    FpgaClockRate: 40
    LoggingIp: 192.168.211.10
    SyncTriggerOffset: 365
    SyncTriggerOnTime: 80
    UdpReadingTimeout: 10
    WaitAfterReset: 60
    AzimuthAxis: false
    EibInputName: 'SLOT03:AXIS02'
    HeadName: TMA-AZ-ENC-ELV-0004
    NsvLinkid: 4
    PositionGain: 0
    PositionOffset: 21628
  Head7:
    AzimuthCriticalActiveHeads: 2
    'AzimuthHeadsReferenceThreshold[i.u]': 10
    AzimuthTapeLineCount: 1243770
    AzimuthTelescopeOffset: 0
    CheckUpdTimeout: 4000
    CmdTimeoutClearErrors: 5000
    CmdTimeoutClearHeadsErrors: 2000
    CmdTimeoutPowerOff: 6000
    CmdTimeoutPowerOn: 9000
    CmdTimeoutReboot: 110000
    CmdTimeoutReferenceOff: 4000
    CmdTimeoutReferenceOn: 10000
    CmdTimeoutRelativeOffset: 2000
    EibConfigFilePath: /c/Configuration/EIB/multi_ext.txt
    ElevationCriticalActiveHeads: 2
    'ElevationHeadsReferenceThreshold[i.u]': 10
    ElevationTapeLineCount: 0
    ElevationTelescopeOffset: 0
    FpgaClockRate: 40
    LoggingIp: 192.168.211.10
    SyncTriggerOffset: 365
    SyncTriggerOnTime: 80
    UdpReadingTimeout: 10
    WaitAfterReset: 60
    AzimuthAxis: true
    EibInputName: 'SLOT04:AXIS01'
    HeadName: TMA-AZ-ENC-AZM-0002
    NsvLinkid: 2
    PositionGain: 0
    PositionOffset: 310942
  Head8:
    AzimuthCriticalActiveHeads: 2
    'AzimuthHeadsReferenceThreshold[i.u]': 10
    AzimuthTapeLineCount: 1243770
    AzimuthTelescopeOffset: 0
    CheckUpdTimeout: 4000
    CmdTimeoutClearErrors: 5000
    CmdTimeoutClearHeadsErrors: 2000
    CmdTimeoutPowerOff: 6000
    CmdTimeoutPowerOn: 9000
    CmdTimeoutReboot: 110000
    CmdTimeoutReferenceOff: 4000
    CmdTimeoutReferenceOn: 10000
    CmdTimeoutRelativeOffset: 2000
    EibConfigFilePath: /c/Configuration/EIB/multi_ext.txt
    ElevationCriticalActiveHeads: 2
    'ElevationHeadsReferenceThreshold[i.u]': 10
    ElevationTapeLineCount: 0
    ElevationTelescopeOffset: 0
    FpgaClockRate: 40
    LoggingIp: 192.168.211.10
    SyncTriggerOffset: 365
    SyncTriggerOnTime: 80
    UdpReadingTimeout: 10
    WaitAfterReset: 60
    AzimuthAxis: false
    EibInputName: 'SLOT04:AXIS02'
    HeadName: TMA-AZ-ENC-ELV-0002
    NsvLinkid: 2
    PositionGain: -0.0
    PositionOffset: -21628
LockingPins:
  LockingPin1:
    Elevationposninetyminus: 89
    Elevationposninetyplus: 91
    Elevationposzerominus: -1
    Elevationposzeroplus: 1
    Systemsource: 1400.LP
    Defaultacceleration: 1000
    Defaultjerk: 4000
    Defaultspeed: 2
    Freeposition: 0
    IdString: LP X+
    Lockposition: 80
    MaxCriticalSpeed: 7.6
    MaxPosition: 82
    MaxSpeed: 7.5
    MinPosition: -3
    Motorid: 19
    MovevelocityTrnsTimeout: -1
    MoveTransTimemargin: 140
    OfftransTimeout: 10000
    OntransTimeout: 10000
    ResettransTimeout: 5000
    SpeedLimit: 7
    StoptransTimeout: 500
    Testposition: 40
  LockingPin2:
    Elevationposninetyminus: 89
    Elevationposninetyplus: 91
    Elevationposzerominus: -1
    Elevationposzeroplus: 1
    Systemsource: 1400.LP
    Defaultacceleration: 1000
    Defaultjerk: 4000
    Defaultspeed: 2
    Freeposition: 0
    IdString: LP X-
    Lockposition: 80
    MaxCriticalSpeed: 7.6
    MaxPosition: 82
    MaxSpeed: 7.5
    MinPosition: -3
    Motorid: 20
    MovevelocityTrnsTimeout: -1
    MoveTransTimemargin: 140
    OfftransTimeout: 10000
    OntransTimeout: 10000
    ResettransTimeout: 5000
    SpeedLimit: 7
    StoptransTimeout: 500
    Testposition: 40
MainAxis:
  Azimuth:
    AllowRelativeMovements: true
    CommandRelativeMovements: false
    ControlAccelerationFeedforwardGain: 0
    ControlDampingActive: true
    ControlDampingXDirectionGain: 0
    ControlDampingYDirectionGain: 0
    ControlInertia: 8670000
    ControlIntegralDisabled: false
    ControlKp(speedErrorGain): 30
    ControlKv(positionErrorGain): 7
    ControlMaximunTorqueVariation: 6500
    ControlSpeedFeedforwardGain: 0
    ControlTi(integralTime): 0.1
    EuiDefaultAcceleration: 3
    EuiDefaultJerk: 14
    EuiDefaultVelocity: 0.5
    EuiMaxAcceleration: 10
    EuiMaxJerk: 48
    EuiMaxVelocity: 10.1
    HhdDefaultAcceleration: 0.5
    HhdDefaultJerk: 1
    HhdDefaultVelocity: 0.5
    HhdMaxAcceleration: 1
    HhdMaxJerk: 28
    HhdMaxVelocity: 1
    HomingAcceleration: 0.1
    HomingJerk: 1
    HomingSpeed: 0.1
    HomingStartInPositiveDirection: true
    InPositionMargin: 0.01
    LimitsActiveDrives: 16
    LimitsAzcwCriticalDeviationValue: 5
    LimitsAzcwDeviationEnable: false
    LimitsAzcwMaximumDeviationValue: 4
    LimitsFollowingErrorEnable: true
    LimitsFollowingErrorValue: 5
    LimitsMaxPositionEnable: true
    LimitsMaxPositionValue: 260
    LimitsMinPositionEnable: true
    LimitsMinPositionValue: -260
    LimitsNegativeAdjustableSoftwareLimitEnable: true
    LimitsNegativeAdjustableSoftwareLimitValue: -265
    LimitsNegativeLimitSwitchEnable: true
    LimitsNegativeOperationalLimitSwitchEnable: false
    LimitsNegativePositiveSoftwareLimitEnable: true
    LimitsNegativeSoftwareLimitEnable: true
    LimitsNegativeSoftwareLimitValue: -265
    LimitsOverspeedEnable: true
    LimitsOverspeedValue: 8.5
    LimitsOverspeedWarningValue: 8
    LimitsPositiveAdjustableSoftwareLimitEnable: true
    LimitsPositiveAdjustableSoftwareLimitValue: 265
    LimitsPositiveLimitSwitchEnable: true
    LimitsPositiveOperationalLimitSwitchEnable: false
    LimitsPositiveSoftwareLimitEnable: true
    LimitsPositiveSoftwareLimitValue: 265
    LimitsSoftmotionNegativeSoftwareLimitEnable: true
    LimitsSoftmotionNegativeSoftwareLimitValue: -275
    LimitsSoftmotionPositiveSoftwareLimitEnable: true
    LimitsSoftmotionPositiveSoftwareLimitValue: 275
    OverrideDefaultAcceleration: 0.2
    OverrideDefaultJerk: 1
    OverrideDefaultVelocity: 0.1
    OverrideMaxAcceleration: 0.5
    OverrideMaxJerk: 28
    OverrideMaxVelocity: 0.1
    SoftmotionErrorStopAcceleration: 7
    SoftmotionErrorStopJerk: 48
    SoftmotionInPositionMargin: 0
    SoftmotionMaxAcceleration: 10.5
    SoftmotionMaxJerk: 50
    SoftmotionMaxSpeed: 10.5
    SoftmotionMaximumTorquePerDrive: 118000
    SoftmotionMinimunDrivesForNoFault: 8
    SoftmotionNoReceivedDataCounterAlarmValue: 5000
    SoftmotionNoReceivedDataCounterWarningValue: 3001
    SoftmotionNumberOfElectricTurns: 700
    SoftmotionSpeedFromPosition: true
    SoftmotionTrackingMarginAcceleration: 1
    SoftmotionTrackingMarginJerk: 9
    SoftmotionTrackingMarginSpeed: 1
    SoftmotionTrackingMaxAcceleration: 7
    SoftmotionTrackingMaxJerk: 48
    SoftmotionTrackingMaxSpeed: 7
    StabilizationTime: 1000
    SubsystemPath: 100.Azimuth
    TcsDefaultAcceleration: 1
    TcsDefaultJerk: 14
    TcsDefaultVelocity: 2
    TcsMaxAcceleration: 7
    TcsMaxJerk: 28
    TcsMaxVelocity: 7
    TimeoutForAckCw: 1000
    TimeoutForAckEib: 1500
    TimeoutForAxisDisable: 200
    TimeoutForAxisEnable: 1000
    TimeoutForBrakeEngage: 20000
    TimeoutForBrakeRelease: 60000
    TimeoutForCableWrapPowerOff: 1000
    TimeoutForCableWrapPowerOn: 1000
    TimeoutForEnabletracking: 1000
    TimeoutForFaultStop: 130
    TimeoutForHoming: 40000
    TimeoutForPowerOff: 300000
    TimeoutForPowerOn: 300000
    TimeoutForReset: 30000
    TimeoutForResetAxis: 2000
    TimeoutForStop: 150
    TimeoutForStopInHoming: 300
    TimeoutMoveMargin: 150
    TrakingConsecutiveTrackingCommandTimeout: 1000
    TrakingFirstTrackingCommandTimeout: 5000
    TrakingTimeOffset: 0
  Elevation:
    AllowRelativeMovements: true
    CommandRelativeMovements: false
    ControlAccelerationFeedforwardGain: 0
    ControlDampingActive: true
    ControlDampingXDirectionGain: 0
    ControlDampingYDirectionGain: 0
    ControlInertia: 8670000
    ControlIntegralDisabled: false
    ControlKp(speedErrorGain): 30
    ControlKv(positionErrorGain): 7
    ControlMaximunTorqueVariation: 6500
    ControlSpeedFeedforwardGain: 0
    ControlTi(integralTime): 0.1
    EuiDefaultAcceleration: 3
    EuiDefaultJerk: 14
    EuiDefaultVelocity: 0.5
    EuiMaxAcceleration: 5
    EuiMaxJerk: 24
    EuiMaxVelocity: 5
    HhdDefaultAcceleration: 0.5
    HhdDefaultJerk: 1
    HhdDefaultVelocity: 0.5
    HhdMaxAcceleration: 1
    HhdMaxJerk: 28
    HhdMaxVelocity: 1
    HomingAcceleration: 0.5
    HomingJerk: 5
    HomingSpeed: 0.1
    HomingStartInPositiveDirection: true
    InPositionMargin: 0.01
    LimitsActiveDrives: 12
    LimitsFollowingErrorEnable: false
    LimitsFollowingErrorValue: 5
    LimitsMaxPositionEnable: true
    LimitsMaxPositionValue: 86.4
    LimitsMinPositionEnable: true
    LimitsMinPositionValue: 15.1
    LimitsNegativeAdjustableSoftwareLimitEnable: true
    LimitsNegativeAdjustableSoftwareLimitValue: 15.1
    LimitsNegativeLimitSwitchEnable: true
    LimitsNegativeOperationalLimitSwitchEnable: true
    LimitsNegativeSoftwareLimitEnable: true
    LimitsNegativeSoftwareLimitValue: -0.1
    LimitsOverspeedEnable: true
    LimitsOverspeedValue: 5.2
    LimitsOverspeedWarningValue: 5.1
    LimitsPositiveAdjustableSoftwareLimitEnable: true
    LimitsPositiveAdjustableSoftwareLimitValue: 86.4
    LimitsPositiveLimitSwitchEnable: true
    LimitsPositiveOperationalLimitSwitchEnable: true
    LimitsPositiveSoftwareLimitEnable: true
    LimitsPositiveSoftwareLimitValue: 90.1
    LimitsSoftmotionNegativeSoftwareLimitEnable: true
    LimitsSoftmotionNegativeSoftwareLimitValue: -3
    LimitsSoftmotionPositiveSoftwareLimitEnable: true
    LimitsSoftmotionPositiveSoftwareLimitValue: 93
    OverrideDefaultAcceleration: 0.2
    OverrideDefaultJerk: 1
    OverrideDefaultVelocity: 0.1
    OverrideMaxAcceleration: 0.5
    OverrideMaxJerk: 28
    OverrideMaxVelocity: 0.1
    SoftmotionErrorStopAcceleration: 3.5
    SoftmotionErrorStopJerk: 24
    SoftmotionInPositionMargin: 0
    SoftmotionMaxAcceleration: 5.5
    SoftmotionMaxJerk: 25
    SoftmotionMaxSpeed: 5.5
    SoftmotionMaximumTorquePerDrive: 24000
    SoftmotionMinimunDrivesForNoFault: 5
    SoftmotionNoReceivedDataCounterAlarmValue: 5000
    SoftmotionNoReceivedDataCounterWarningValue: 3001
    SoftmotionNumberOfElectricTurns: 390
    SoftmotionSpeedFromPosition: true
    SoftmotionTrackingMarginAcceleration: 1
    SoftmotionTrackingMarginJerk: 9
    SoftmotionTrackingMarginSpeed: 1
    SoftmotionTrackingMaxAcceleration: 3.5
    SoftmotionTrackingMaxJerk: 24
    SoftmotionTrackingMaxSpeed: 5
    StabilizationTime: 1000
    SubsystemPath: 400.Elevation
    TcsDefaultAcceleration: 1
    TcsDefaultJerk: 14
    TcsDefaultVelocity: 2
    TcsMaxAcceleration: 7
    TcsMaxJerk: 28
    TcsMaxVelocity: 7
    TimeoutForAckCw: 1000
    TimeoutForAckEib: 1500
    TimeoutForAxisDisable: 200
    TimeoutForAxisEnable: 1000
    TimeoutForBrakeEngage: 20000
    TimeoutForBrakeRelease: 60000
    TimeoutForCableWrapPowerOff: 1000
    TimeoutForCableWrapPowerOn: 1000
    TimeoutForEnabletracking: 1000
    TimeoutForFaultStop: 130
    TimeoutForHoming: 40000
    TimeoutForPowerOff: 300000
    TimeoutForPowerOn: 300000
    TimeoutForReset: 30000
    TimeoutForResetAxis: 2000
    TimeoutForStop: 150
    TimeoutForStopInHoming: 300
    TimeoutMoveMargin: 150
    TrakingConsecutiveTrackingCommandTimeout: 1000
    TrakingFirstTrackingCommandTimeout: 5000
    TrakingTimeOffset: 0
MainAxisSoftMotion:
  Azimuth: {}
  Elevation: {}
MainCabinetTemperature:
  MainCabinet:
    CriticalHighCabinetTemperature: 45
    CriticalLowCabinetTemperature: 4
    Exitsetpoint: 20
    HighCabinetTemperature: 43
    HighSurfaceTemperature: 5
    IdString: ''
    LowCabinetTemperature: 5
    LowSurfaceTemperature: -5
    LowerLimitHysteresisRelative: 2
    MaximumSetpointTemperature: 35
    MinimumSetpointTemperature: 5
    Systemsource: 1300.MainCabinet
    UpperLimitHysteresisRelative: 2
MainPowerSupply:
  MainPowerSupply:
    Systemsource: 600.MPS
    CwPowerOff: 0
    CwPowerOn: 0
    CwReset: 0
    Maxcurrent: 75
    Maxcurrentwarning: 70
    Minvoltage: 500
    Minvoltagewarning: 550
    PoweringTimeOut: 150000
    ResetingTime: 1000
    WaitPowerOffInFault: 5000
MirrorCover:
  MC1:
    Minelevationposition: 15
    Systemsource: 900.MC
    WaittransTimeout: 60000
    CriticalMaxMotorVelocity: 25
    DeployedPosition: 134
    ErrorId: '900'
    IdString: MC X+
    Idstringstocheck: 'MC Y-,MC Y+,MCL Y-,MCL X+,MCL Y+,MCL X-'
    Inrangemargin: 8
    Isontop: false
    JogSpeed: 10
    MaxAnglePosition: 135
    MaxSpeed: 25
    MinAnglePosition: 1
    MotorAcceleration: 60
    MotorDeceleration: 60
    MotorId: 5
    MotorJerk: 120
    MotorMaxVelocity: 22
    MotorVelocity: 20
    MovevelocityTrnsTimeout: -1
    MoveTransTimemargin: 180
    OfftransTimeout: 3000
    OntransTimeout: 5000
    Openorder: false
    Resettingtime: 3000
    RetractedPosition: 2
    StoptransTimeout: 3000
    Waitafterreset: 0
  MC2:
    Minelevationposition: 15
    Systemsource: 900.MC
    WaittransTimeout: 60000
    CriticalMaxMotorVelocity: 25
    DeployedPosition: 134
    ErrorId: '900'
    IdString: MC Y+
    Idstringstocheck: 'MC X+,MC X-,MCL Y-,MCL X+,MCL Y+,MCL X-'
    Inrangemargin: 8
    Isontop: true
    JogSpeed: 10
    MaxAnglePosition: 135
    MaxSpeed: 25
    MinAnglePosition: 1
    MotorAcceleration: 60
    MotorDeceleration: 60
    MotorId: 7
    MotorJerk: 120
    MotorMaxVelocity: 22
    MotorVelocity: 20
    MovevelocityTrnsTimeout: -1
    MoveTransTimemargin: 180
    OfftransTimeout: 3000
    OntransTimeout: 5000
    Openorder: true
    Resettingtime: 3000
    RetractedPosition: 2
    StoptransTimeout: 3000
    Waitafterreset: 0
  MC3:
    Minelevationposition: 15
    Systemsource: 900.MC
    WaittransTimeout: 60000
    CriticalMaxMotorVelocity: 25
    DeployedPosition: 134
    ErrorId: '900'
    IdString: MC X-
    Idstringstocheck: 'MC Y+,MC Y-,MCL Y-,MCL X+,MCL Y+,MCL X-'
    Inrangemargin: 8
    Isontop: false
    JogSpeed: 10
    MaxAnglePosition: 135
    MaxSpeed: 25
    MinAnglePosition: 1
    MotorAcceleration: 60
    MotorDeceleration: 60
    MotorId: 9
    MotorJerk: 120
    MotorMaxVelocity: 22
    MotorVelocity: 20
    MovevelocityTrnsTimeout: -1
    MoveTransTimemargin: 180
    OfftransTimeout: 3000
    OntransTimeout: 5000
    Openorder: false
    Resettingtime: 3000
    RetractedPosition: 2
    StoptransTimeout: 3000
    Waitafterreset: 0
  MC4:
    Minelevationposition: 15
    Systemsource: 900.MC
    WaittransTimeout: 60000
    CriticalMaxMotorVelocity: 25
    DeployedPosition: 134
    ErrorId: '900'
    IdString: MC Y-
    Idstringstocheck: 'MC X+,MC X-,MCL Y-,MCL X+,MCL Y+,MCL X-'
    Inrangemargin: 8
    Isontop: true
    JogSpeed: 10
    MaxAnglePosition: 135
    MaxSpeed: 25
    MinAnglePosition: 1
    MotorAcceleration: 60
    MotorDeceleration: 60
    MotorId: 11
    MotorJerk: 120
    MotorMaxVelocity: 22
    MotorVelocity: 20
    MovevelocityTrnsTimeout: -1
    MoveTransTimemargin: 180
    OfftransTimeout: 3000
    OntransTimeout: 5000
    Openorder: true
    Resettingtime: 3000
    RetractedPosition: 2
    StoptransTimeout: 3000
    Waitafterreset: 0
MirrorCoverLocks:
  MCL1:
    Systemsource: 1500.MC
    CriticalMaxMotorVelocity: 50
    ErrorId: '1500'
    IdString: MCL X+
    Idstringstocheck: 'MC X+,MC Y+,MC X-,MC Y-'
    Inrangemargin: 8
    JogSpeed: 20
    LockedPosition: 5
    MaxAnglePosition: 173
    MaxSpeed: 50
    MinAnglePosition: 3
    MotorAcceleration: 180
    MotorDeceleration: 180
    MotorId: 6
    MotorJerk: 360
    MotorMaxVelocity: 48
    MotorVelocity: 45
    MovevelocityTrnsTimeout: -1
    MoveTransTimemargin: 150
    OfftransTimeout: 3000
    OntransTimeout: 3000
    Resettingtime: 3000
    StoptransTimeout: 3000
    UnlockedPosition: 170
    Waitafterreset: 0
  MCL2:
    Systemsource: 1500.MC
    CriticalMaxMotorVelocity: 50
    ErrorId: '1500'
    IdString: MCL Y+
    Idstringstocheck: 'MC X+,MC Y+,MC X-,MC Y-'
    Inrangemargin: 8
    JogSpeed: 20
    LockedPosition: 5
    MaxAnglePosition: 173
    MaxSpeed: 50
    MinAnglePosition: 3
    MotorAcceleration: 180
    MotorDeceleration: 180
    MotorId: 8
    MotorJerk: 360
    MotorMaxVelocity: 48
    MotorVelocity: 45
    MovevelocityTrnsTimeout: -1
    MoveTransTimemargin: 150
    OfftransTimeout: 3000
    OntransTimeout: 3000
    Resettingtime: 3000
    StoptransTimeout: 3000
    UnlockedPosition: 170
    Waitafterreset: 0
  MCL3:
    Systemsource: 1500.MC
    CriticalMaxMotorVelocity: 50
    ErrorId: '1500'
    IdString: MCL X-
    Idstringstocheck: 'MC X+,MC Y+,MC X-,MC Y-'
    Inrangemargin: 8
    JogSpeed: 20
    LockedPosition: 5
    MaxAnglePosition: 173
    MaxSpeed: 50
    MinAnglePosition: 3
    MotorAcceleration: 180
    MotorDeceleration: 180
    MotorId: 10
    MotorJerk: 360
    MotorMaxVelocity: 48
    MotorVelocity: 45
    MovevelocityTrnsTimeout: -1
    MoveTransTimemargin: 150
    OfftransTimeout: 3000
    OntransTimeout: 3000
    Resettingtime: 3000
    StoptransTimeout: 3000
    UnlockedPosition: 170
    Waitafterreset: 0
  MCL4:
    Systemsource: 1500.MC
    CriticalMaxMotorVelocity: 50
    ErrorId: '1500'
    IdString: MCL Y-
    Idstringstocheck: 'MC X+,MC Y+,MC X-,MC Y-'
    Inrangemargin: 8
    JogSpeed: 20
    LockedPosition: 5
    MaxAnglePosition: 173
    MaxSpeed: 50
    MinAnglePosition: 3
    MotorAcceleration: 180
    MotorDeceleration: 180
    MotorId: 12
    MotorJerk: 360
    MotorMaxVelocity: 48
    MotorVelocity: 45
    MovevelocityTrnsTimeout: -1
    MoveTransTimemargin: 150
    OfftransTimeout: 3000
    OntransTimeout: 3000
    Resettingtime: 3000
    StoptransTimeout: 3000
    UnlockedPosition: 170
    Waitafterreset: 0
ModbusTemperatureControllers:
  TMA_AX_DZ_CBT_0001:
    Systemsource: 2600.ModbusTemperature
    Tasksamplingtime: 1000
    Commandtimeout: 10000
    IdString: TMA_AX_DZ_CBT_0001
    Modbusserverconfigfilepath: >-
      /c/Configuration/ModbusTemperatureControllers/TMA_AX_DZ_CBT_0001_config.ini
    Modbusservermappingfilepath: >-
      /c/Configuration/ModbusTemperatureControllers/TMA_AX_DZ_CBT_0001_mapping.txt
    Resetvalue: 0
    Sendresetvalue: false
    Temperatureconversionread: 0.1
    Temperatureconversionwrite: 10
    Temperaturedeviationalarm: 5
    Temperaturedeviationreference: -1
    Temperaturedeviationwarning: 3
  TMA_AZ_PD_CBT_0001:
    Systemsource: 2600.ModbusTemperature
    Tasksamplingtime: 1000
    Commandtimeout: 10000
    IdString: TMA_AZ_PD_CBT_0001
    Modbusserverconfigfilepath: >-
      /c/Configuration/ModbusTemperatureControllers/TMA_AZ_PD_CBT_0001_config.ini
    Modbusservermappingfilepath: >-
      /c/Configuration/ModbusTemperatureControllers/TMA_AZ_PD_TRM_0001_mapping.txt
    Resetvalue: 0
    Sendresetvalue: false
    Temperatureconversionread: 0.1
    Temperatureconversionwrite: 10
    Temperaturedeviationalarm: 5
    Temperaturedeviationreference: -1
    Temperaturedeviationwarning: 3
  TMA_AZ_PD_TRM_0001:
    Systemsource: 2600.ModbusTemperature
    Tasksamplingtime: 1000
    Commandtimeout: 10000
    IdString: TMA_AZ_PD_TRM_0001
    Modbusserverconfigfilepath: >-
      /c/Configuration/ModbusTemperatureControllers/TMA_AZ_PD_TRM_0001_config.ini
    Modbusservermappingfilepath: >-
      /c/Configuration/ModbusTemperatureControllers/TMA_AZ_PD_CBT_0001_mapping.txt
    Resetvalue: 0
    Sendresetvalue: false
    Temperatureconversionread: 0.1
    Temperatureconversionwrite: 10
    Temperaturedeviationalarm: 5
    Temperaturedeviationreference: -1
    Temperaturedeviationwarning: 3
  TMA_EL_PD_CBT_0001:
    Systemsource: 2600.ModbusTemperature
    Tasksamplingtime: 1000
    Commandtimeout: 10000
    IdString: TMA_EL_PD_CBT_0001
    Modbusserverconfigfilepath: >-
      /c/Configuration/ModbusTemperatureControllers/TMA_EL_PD_CBT_0001_config.ini
    Modbusservermappingfilepath: >-
      /c/Configuration/ModbusTemperatureControllers/TMA_EL_PD_CBT_0001_mapping.txt
    Resetvalue: 0
    Sendresetvalue: false
    Temperatureconversionread: 0.1
    Temperatureconversionwrite: 10
    Temperaturedeviationalarm: 5
    Temperaturedeviationreference: -1
    Temperaturedeviationwarning: 3
  TMA_EL_PD_CBT_0002:
    Systemsource: 2600.ModbusTemperature
    Tasksamplingtime: 1000
    Commandtimeout: 10000
    IdString: TMA_EL_PD_CBT_0002
    Modbusserverconfigfilepath: >-
      /c/Configuration/ModbusTemperatureControllers/TMA_EL_PD_CBT_0002_config.ini
    Modbusservermappingfilepath: >-
      /c/Configuration/ModbusTemperatureControllers/TMA_EL_PD_CBT_0002_mapping.txt
    Resetvalue: 0
    Sendresetvalue: false
    Temperatureconversionread: 0.1
    Temperatureconversionwrite: 10
    Temperaturedeviationalarm: 5
    Temperaturedeviationreference: -1
    Temperaturedeviationwarning: 3
MotorThermalControl:
  AzimuthGroup1:
    Ambienttemperatureoffset: -1
    AutotuneControllerType: 2
    AutotuneStepAmplitude: 40
    AutotuneTimeConstantFator: 0.1
    ControlAlpha: 0
    ControlKp: 176.43
    ControlTd: 0.47
    ControlTi: 2.15
    IdString: AZDTgroup1
    PlantDelayTime: 176.53
    PlantProcessGain: 0
    PlantTimeConstant: 40.97
    Systemsource: 1600.AZDT
    Temperaturedeviationalarm: 5
    Temperaturedeviationreference: -1
    Temperaturedeviationwarning: 3
    TimeoutPoweron: 500000
    Valvedisabledaperture: 40
    Valveinpositiontimeout: 100
    Valveoutputhigh: 100
    Valveoutputlow: 0
    Valvevalidband: 5
  AzimuthGroup2:
    Ambienttemperatureoffset: -1
    AutotuneControllerType: 2
    AutotuneStepAmplitude: 40
    AutotuneTimeConstantFator: 0.1
    ControlAlpha: 0
    ControlKp: 179.44
    ControlTd: 0.47
    ControlTi: 2.15
    IdString: AZDTgroup2
    PlantDelayTime: 176.53
    PlantProcessGain: 0
    PlantTimeConstant: 40.97
    Systemsource: 1600.AZDT
    Temperaturedeviationalarm: 5
    Temperaturedeviationreference: -1
    Temperaturedeviationwarning: 3
    TimeoutPoweron: 500000
    Valvedisabledaperture: 40
    Valveinpositiontimeout: 100
    Valveoutputhigh: 100
    Valveoutputlow: 0
    Valvevalidband: 5
  AzimuthGroup3:
    Ambienttemperatureoffset: -1
    AutotuneControllerType: 2
    AutotuneStepAmplitude: 40
    AutotuneTimeConstantFator: 0.1
    ControlAlpha: 0
    ControlKp: 179.44
    ControlTd: 0.47
    ControlTi: 2.15
    IdString: AZDTgroup3
    PlantDelayTime: 176.53
    PlantProcessGain: 0
    PlantTimeConstant: 40.97
    Systemsource: 1600.AZDT
    Temperaturedeviationalarm: 5
    Temperaturedeviationreference: -1
    Temperaturedeviationwarning: 3
    TimeoutPoweron: 500000
    Valvedisabledaperture: 40
    Valveinpositiontimeout: 100
    Valveoutputhigh: 100
    Valveoutputlow: 0
    Valvevalidband: 5
  AzimuthGroup4:
    Ambienttemperatureoffset: -1
    AutotuneControllerType: 2
    AutotuneStepAmplitude: 40
    AutotuneTimeConstantFator: 0.1
    ControlAlpha: 0
    ControlKp: 179.44
    ControlTd: 0.47
    ControlTi: 2.15
    IdString: AZDTgroup4
    PlantDelayTime: 176.53
    PlantProcessGain: 0
    PlantTimeConstant: 40.97
    Systemsource: 1600.AZDT
    Temperaturedeviationalarm: 5
    Temperaturedeviationreference: -1
    Temperaturedeviationwarning: 3
    TimeoutPoweron: 500000
    Valvedisabledaperture: 40
    Valveinpositiontimeout: 100
    Valveoutputhigh: 100
    Valveoutputlow: 0
    Valvevalidband: 5
  ElevationGroup1:
    Ambienttemperatureoffset: -1
    AutotuneControllerType: 2
    AutotuneStepAmplitude: 1
    AutotuneTimeConstantFator: 10
    ControlAlpha: 0
    ControlKp: 1
    ControlTd: 0
    ControlTi: 0
    IdString: ELDTgroup1
    PlantDelayTime: 1
    PlantProcessGain: 1
    PlantTimeConstant: 1
    Systemsource: 1700.ELDT
    Temperaturedeviationalarm: 5
    Temperaturedeviationreference: -1
    Temperaturedeviationwarning: 3
    TimeoutPoweron: 50000
    Valvedisabledaperture: 40
    Valveinpositiontimeout: 100
    Valveoutputhigh: 100
    Valveoutputlow: 0
    Valvevalidband: 5
  ElevationGroup2:
    Ambienttemperatureoffset: -1
    AutotuneControllerType: 2
    AutotuneStepAmplitude: 1
    AutotuneTimeConstantFator: 10
    ControlAlpha: 0
    ControlKp: 1
    ControlTd: 0
    ControlTi: 0
    IdString: ELDTgroup2
    PlantDelayTime: 1
    PlantProcessGain: 1
    PlantTimeConstant: 1
    Systemsource: 1700.ELDT
    Temperaturedeviationalarm: 5
    Temperaturedeviationreference: -1
    Temperaturedeviationwarning: 3
    TimeoutPoweron: 50000
    Valvedisabledaperture: 40
    Valveinpositiontimeout: 100
    Valveoutputhigh: 100
    Valveoutputlow: 0
    Valvevalidband: 5
  Cabinet0101:
    Ambienttemperatureoffset: -1
    AutotuneControllerType: 2
    AutotuneStepAmplitude: 40
    AutotuneTimeConstantFator: 0.1
    ControlAlpha: 0
    ControlKp: 179.44
    ControlTd: 0.47
    ControlTi: 2.15
    IdString: Cabinet0101
    PlantDelayTime: 176.53
    PlantProcessGain: 0
    PlantTimeConstant: 40.97
    Systemsource: 1900.Cabinet0101
    Temperaturedeviationalarm: 5
    Temperaturedeviationreference: -1
    Temperaturedeviationwarning: 3
    TimeoutPoweron: 500000
    Valvedisabledaperture: 40
    Valveinpositiontimeout: 100
    Valveoutputhigh: 100
    Valveoutputlow: 0
    Valvevalidband: 5
OSS:
  OSS:
    Cooldownbearingstimeout: 6
    Cooldowntanktimeout: 6
    IdString: ''
    Operationtransitiontimeout: 7
    Poweroffcoolingtimeout: 6
    Poweroffmainpumptimeout: 6
    Poweroffoilcirculationtimeout: 6
    Poweroncoolingtimeout: 6
    Poweronmainpumptimeout: 6
    Poweronoilcirculationtimeout: 6
    Resettingtime: 3000
    Systemsource: 800.OSS
    Watchdogmaximumtime: 10
Safety:
  FaultIterationsSafetyNook: 3
  MonitoringRateMs: 100
  Overridecausestimeout: 1000
  Resetcausesnumberofchecks: 6
  ResetcausestimebeforecheckingS: 1
  ResetcausestimebetweencheckingS: 0.5
  Resetcausestimeout: 5200
  RtToSafetyClockMs: 500
  SafetyClockMs: 3000
TMAMainRT:
  Aftercomanderlaunchwait: 200
  Beforeazmotorthermallaunchwait: 6500
  Beforeballaunchwait: 4500
  Beforeboschlaunchwait: 3500
  Beforeboschpowersupplylaunchwait: 3000
  Beforecabinet0101launchwait: 7500
  Beforecabinetlaunchwait: 8000
  Beforeccwlaunchwait: 10000
  Beforedplaunchwait: 5500
  Beforeelmotorthermallaunchwait: 7000
  Beforeiotelemetrypublicationlaunchwait: 9500
  Beforelplaunchwait: 4000
  Beforemainaxislaunchwait: 20
  Beforemclaunchwait: 4500
  Beforemcllaunchwait: 5000
  Beforemodbustempcontrollerslaunchwait: 9000
  Beforempslaunchwait: 6000
  Beforeosslaunchwait: 8500
  Beforesafetylaunchwait: 0
TopEndChiller:
  LocalremoteTimeout: 10000
  PowerOn/offTimeout: 10000
  ResetPulseTime: 1000
TrackingTrajectory:
  Averagesamplelength: 100
  AzimuthSlewThreshold: 0
  CommandQueueSize: 12000
  ElevationSlewThreshold: 0
  FinalMotionBehaviour: 0
  Maxallowederror: 0
  NumberOfAxes: 2
  ReflexxessDllPath: 'C:\ReflexxesFuncs.dll'
  SynchronizationBehaviour: 3
  TimeSetpointOffset: -0.0
  WaitForDataBeforeError: 1000
  WaitingTimeForSetpointTime: 50
  WaitingTimeIfNoData: 5000
"""
)
