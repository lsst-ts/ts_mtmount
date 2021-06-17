.. py:currentmodule:: lsst.ts.MTMount

.. _lsst.ts.MTMount-tma_interface:

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

* AZ: Azimuth axis.
* EL: Elevation axis.
* BAL: Balance motors.
* CW: Cable Wrap.
* DP: Deployable Platform.
* EIB: encoder interface box (EIB is a Heidenhain term).
* EUI: Engineering User Interface.
* HHD: Hand-Held Device.
* HMI: Human-machine interface this refers to both the EUI and HHD.
* LP: Locking Pin.
* MC: Mirror Covers.
* MCL: Mirror Cover Locks which lock the mirror covers when retracted.
* MPS: Main Power Supply.
* OSS: Oil Supply System.
* TEC: Top-End Chiller.
* TF: Transfer Function.
* TMA: Telescope mount assembly; the low-level controller.

Components
----------

* EUI (LabVIEW). Sends commands to the PXI and reads command acknowledgements, using TCP/IP through the Operation Manager.
  Gets state from the PXI using LabVIEW Network Shared Variables.
* HHD (LabVIEW). Same communication scheme as the EUI.
* CSC (Python): the code in this ts_MTMount package.
* Operation Manager (C++). Connects to the CSC, EUI, and HHD (using TCP/IP), and coordinates communication between them and the low-level controller.
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
* PXI

In addition we will ask Tekniker to provide a new port for the CSC.

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
https://gitlab.tekniker.es/aut/projects/3151-LSST/LabVIEWCode/documentation/pxicontroller_documentation/-/blob/develop/02%20CommandsAndEventsManagement/03%20Events.md

Some supplemental information:

* motionState: the position field is the target position for point to point moves.

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
  -1 means "no known timeout (wait forever).
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
