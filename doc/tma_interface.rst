.. py:currentmodule:: lsst.ts.mtmount

.. _lsst.ts.mtmount-tma_interface:

Interface with the Telescope Mount Assembly
===========================================

The low-level controller, written by Tekniker, is called the Telescope Mount Assembly (TMA).
Communication with the CSC is via two TCP/IP ports:

* One port is used for commands, replies and events (reports of changes to discrete state).
* The other port is used for telemetry; this a read-only connection.

Telemetry is output on a separate port because we were afraid the CSC would be overwhelmed
if it had to deal with telemetry as well as commands, replies, and status.
This allows the CSC to launch a background process to handle telemetry.

Acronyms
--------

Acronyms used in Tekniker's code and (sparingly) in this package:

* ACW: Azimuth Cable Wrap.
* AZ: Azimuth axis.
* EL: Elevation axis.
* BAL: Balance motors.
* CCW: Camera Cable Wrap
* CW: Cable Wrap (azimuth or camera)
* DP: Deployable Platform.
* EIB: encoder interface box (EIB is a Heidenhain term).
* EUI: Engineering User Interface.
* GIS: Global Interlock System, also called the "safety system".
* HHD: Hand-Held Device.
* HMI: Human-machine interface this refers to both the EUI and HHD.
* LP: Locking Pin.
* MC: Mirror Covers.
* MCL: Mirror Cover Locks which lock the mirror covers when retracted.
* MPS: Main Power Supply.
* OSS: Oil Supply System.
* TCS: MTMount CSC ("Telescope Control System").
* TEC: Top-End Chiller.
* TF: Transfer Function.
* TMA: Telescope mount assembly; the low-level controller.

Components
----------

* EUI (LabVIEW). Sends commands to the PXI and reads command acknowledgements, using TCP/IP through the Operation Manager.
  Gets state from the PXI using LabVIEW Network Shared Variables.
* HHD (LabVIEW). Same communication scheme as the EUI.
* CSC/TCS (Python): the code in this ts_mtmount package.
* Operation Manager (C++). Connects to the CSC/TCS, EUI, and HHD using TCP/IP, and coordinates communication between them and the low-level controller.
* PXI (LabVIEW): there are two PXI computers running LabVIEW that together implement the low-level controller.
  These PXIs divide responsibilities approximately as follows:
  
  * One controls the azimuth and altitude axes.
  * The other implements the remaining low level control tasks, including communication with the Operation Manager and maintaining the LabVIEW Network Shared Variables for the EUI.

TCP/IP Protocol
---------------

The TCP/IP communication protocol is as follows:

* Commands, replies and events go over one socket, with the server in the operation manager.
* Telemetry is output on another socket, with the server in the EUI computer.
* Data is sent as ASCII, with all messages terminated with `\r\n`.
  Telemetry and events are formatted as a dict in json format.
  Commands use a simple custom format with `\n`-separated fields;
  these fields include a list of zero or more command-specific parameter values.
  Commands do not include parameter names; commands and replies do.

There are TCP/IP connections between the following components and the Operations Manager:

* EUI
* HHD
* TCS/CSC
* PXI

Commands
^^^^^^^^

Commands have the following fields, in order:

* sequence_id (int): an incrementing value supplied by the commander (e.g. EUI or CSC) used to identify the command in replies.
* command_code (int): the command code
* source (int): who initiated the command
* timestamp (str): timestamp in ISO format (UTC)
* additional parameters, if any. Their number and meaning depends on the ``command_code``.

Replies
^^^^^^^

All replies are in json format and have the following fields:

* id (int): a ReplyId enum value.
* timestamp (float): the time the message created or sent, as TAI unix.
* parameters (dict): additional parameters which vary depending on the id.

The vendor's documentation for replies ("events") is
https://gitlab.tekniker.es/publico/3151-lsst/pxicontroller_documentation/-/blob/develop/02%20CommandsAndEventsManagement/03%20Events.md

Some supplemental information:

* axisMotionState: the position field is the target position for point to point moves.

Here are some sample replies:

* CMD_ACKNOWLEDGED: {"id":1,"timestamp":3696497925.408238,"parameters":{"sequenceId":1500,"timeout":1.500000}}\r\n
* CMD_REJECTED: {"id":2,"timestamp":3696498401.849609,"parameters":{"sequenceId":1500,"explanation":"Rejected explanation goes here."}}\r\n
* CMD_SUCCEEDED: {"id":3,"timestamp":3696498414.471823,"parameters":{"sequenceId":1500}}\r\n
* CMD_FAILED: {"id":4,"timestamp":3696498419.879615,"parameters":{"sequenceId":1500,"explanation":"Failed explanation goes here."}}\r\n
* CMD_SUPERSEDED: {"id":5,"timestamp":3696498425.299656,"parameters":{"sequenceId":1500,"supersedingSequenceId":1499, "supersedingCommander":2,"supersedingCommandCode":1201}}\r\n
* WARNING: {"id":10,"timestamp":3696569120.755037,"parameters":{"name":"This is the warning name.","subsystemId":1400,"subsystemInstance":"LP","active":false,"code":1402,"description":"This is the warning description."}}\r\n
* ALARM: {"id":11,"timestamp":3696569097.115004,"parameters":{"name":"This is the alarm name.","subsystemId":1400,"subsystemInstance":"LP","active":false,"latched":false,"code":1402,"description":"This is the alarm description."}}\r\n
* IN_POSITION: {"id":200,"timestamp":3696569018.128953,"parameters":{"axis":0,"inPosition":true}}\r\n

CMD_x replies have the following parameters:

* sequenceId (int): the incrementing value specified by the client, used to identify replies for a given command.
* CMD_ACKNOWLEDGED: ``timeout`` (double), expected command duration (seconds).
  Add 2 seconds to this value if you wish to use this for a timeout timer.
  -1 means "no known timeout" (wait forever).
* CMD_REJECTED and CMD_FAILED: ``explanation`` (str): text explaining why the command was rejected.
* CMD_SUPERSEDED: ``supersedingSequenceId`` (int), ``supersedingCommander`` (int), ``supersedingCommandCode`` (int):
  information about the superseding command, where ``supersedingCommander`` is a `SourceId` (e.g. HHD).

WARNING and ALARM replies have the following parameters:

* ALARM: latched (bool): has the alarm condition been seen?
  When the alarm condition is first seen this field is set to true;
  it remains true until the alarm is reset (which can only happen if the alarm condition is no longer active).
* active (bool): is the alarm condition present?
* code (int): code number of event.
  The code numbers consist of a ``subsystemId`` plus a condition-specific value.
* subsystemId (int): ID of subsystem, a `Source`
* subsystemInstance (str): subsytem component.
  Here are three examples provided by Alberto: "Azimuth", "Trajectory generator", "MyTopVI/MyNextVI/MyNextNextVI".
* timestamp (float): time of message, TAI unix seconds
* description (str): description of the problem.

IN_POSITION replies indicate if the Azimuth or Elevation axes are in position.
Parameters:

* axis (int): 0 for Azimuth, 1 for Elevation
* inPosition (bool): in position?

Axis Limits and Motion Parameters
---------------------------------

This section describes the limits and some of the motion parameters for the main axes (azimuth and elevation) and the camera cable wrap.
(The azimuth cable wrap is completely controlled by the low-level controller, so is not described here.)

The low-level controller reports the parameters described here as part of the voluminous DETAILED_SETTINGS_APPLIED event.
All of the reported settings are contained in named configuration files in the low-level controller.
You may change settings by loading a different file (the preferred way to do it), or by editing settings in the EUI (only do this for engineering work).

*Warning*: the low-level controller rounds reported values to 2 decimal places, so be careful.
The CSC compensates for this by shrinking the command limits a bit, when commanding the camera cable wrap to follow the rotator.

Limits Overview
^^^^^^^^^^^^^^^

The main axes and camera cable wrap have the following types of limits:

* Command limits. Point-to-point and tracking commands are rejected if they have values outside these limits.
* L1 (software) limits.
  If the limit is enabled and the axis moves outside this limit (or near it for the main axes) the axis is halted.
  You can move the axis in the other direction to back out of the limit.
  There are subtle differences between the way L1 limits are enforced for the main axes vs. the camera cable wrap:

  * Main axes: the L1 limits are applied by monitoring the actual position and velocity and stopping motion early, such that the axes should never overshoot these limits.
  * Camera cable wrap: the L1 limits are triggered when the actual position crosses the limit.
    Thus the camera cable wrap will overshoot the L1 limits.

* L2 (direction inhibit) limit switches.
  If an L2 switch is hit and the switch is enabled the axis is halted.
  You can move the axis in the other direction to back out of the limit.
* L3 (safety) limit switches.
  The GIS halts the axis and cuts power to the motors.
  The only way to recover is to temporarily disable the safety system and slowly move away from the switch.

The low-level controller does not require the command limits to be within L1 (software) limits, but that is the normal way to configure them.

The low-level controller does not apply position limits to the demand generated by the trajectory generator.
Thus it is possible to command the low-level controller to go beyond its L1 limits.
When this happens, the L1 limit enforcer halts the axis.

Main Axes
^^^^^^^^^

In addition to the usual L1 (software) and L2 (direction inhibit) switches,
the main axes (azimuth and elevation) also have an extra set of limit switches which are used for normal operation, but disabled for parking the telescope.
These extra limit switches are:

* "Adjustable" L1 limit switches.
* "Operational" L2 limit switches. Only elevation has these switches, but azimuth reports Limits[Negative|Positive]OperationalLimitSwitchEnable=False, as if it had such switches, but they are never enabled.

Here is the meaning for some of the main axis parameters of the ``DETAILED_SETTINGS_APPLIED`` event.
Note that any limit with ``Tcs`` in its name only applies to the MTMount CSC (aka TCS);
there will be similarly-named limits for the EUI and HHD:

* Limits[Min|Max]PositionEnable: are the min/max command position limits enforced?
* Limits[Min|MaxPositionValue: the min/max command position limits.
* Thus the actual min/max command position limits are given by::

    if Limits[Min|Max]PositionEnable:
        Limits[Min|Max]PositionValue
    else:
        no min/max command limit

* TcsMaxVelocity: command velocity limit.
* Limits[Negative|Positive]AdjustableSoftwareLimitEnable: are the adjustable L1 (software) limits enabled?
* Limits[Negative|Positive]SoftwareLimitEnabled: are the non-adjustable L1 (software) limits enabled?
* Thus the actual min/max L1 (software) limits are given by::

        if Limits[Negative|Positive]AdjustableSoftwareLimitEnable:
            Limits[Negative|Positive]AdjustableSoftwareLimitValue
        else if Limits[Negative|Positive]SoftwareLimitEnabled:
            Limits[Negative|Positive]SoftwareLimitValue
        else: 
            no min/max L1 software limit.
* Limits[Negative|Positive]LimitSwitchEnable (sic): is the min/max normal L2 switch enabled?
* Limits[Negative|Positive]OperationalLimitSwitchEnable (sic): is the min/max operational L2 switch enabled?
* TcsDefaultVelocity|Acceleration|Jerk: the max velocity, acceleration and jerk for point-to-point moves for the CSC.
  Technically these are defaults, which are only used if the specified value is 0;
  however, the CSC always specifies 0 for these parameters.
* SoftmotionTrackingMax[Speed|Acceleration|Jerk]: the max velocity, acceleration and jerk used by the trajectory planner for slewing and tracking.
* Override[Max|Default][Velocity|Acceleration|Jerk]: these values are used when you temporarily override an L3 (safety) limit.
  Overriding the L3 limit switches is done via the Global Interlock System (GIS) and not via loading a settings file;
  that is why there is no "Override" flag for L3 limit switches in the detailed settings.
  The safety system also has speed limits for elevation and azimuth which kill power to the motor. Changing those limits requires recompiling the safety system.

Camera Cable Wrap
^^^^^^^^^^^^^^^^^

The camera cable wrap (CCW) has simpler limits than the main axes.
For example it has no "adjustable" L1 or "operational" L2 limits,
and the limits are the same for the EUI, HHD and TCS.
Also many parameter names are different than for the main axes.
Here are some of the most important parameters:

* [Min|Max]Position: command position limits.
* MaxSpeed: command velocity limits.
* Default[Speed|Acceleration|Jerk]: the max velocity, acceleration and jerk for point-to-point moves.
  Technically these are defaults, which are only used if the specified value is 0;
  however, the CSC always specifies 0 for these parameters.
* Tracking[Speed|Acceleration|Jerk]: the max velocity, acceleration and jerk used by the trajectory planner for slewing and tracking.
* [Min|Max]SoftwareLimit: the software position limits.
* [Negative|Positive]SoftwareLimitEnable: are the software position limits enabled?
* [Negative|Positive]LimitSwitchEnable: are the direction inhibit limit switches enabled?
