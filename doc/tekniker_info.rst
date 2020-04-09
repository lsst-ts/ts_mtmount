.. py:currentmodule:: lsst.ts.MTMount

.. _lsst.ts.MTMount-tekniker_info:

Interactions with Tekniker's software
=====================================

Acronyms
--------

Acronyms used in Tekniker's code and (sparingly) in this package:

* AZ: Azimuth axis
* EL: Elevation axis
* BAL: Balance motors
* CW: Cable Wrap
* DP: Deployable Platform
* EIB: encoder interface box: EIB is a Heidenhain term.
* EUI: Engineering User Interface
* HHD: Hand-Held Device
* HMI: Human-machine interface: both the EUI and HHD
* LP: Locking Pin
* MC: Mirror Covers
* MCL: Mirror Cover Locks which lock the mirror covers when retracted.
* MPS: Main Power Supply
* OSS: Oil Supply System
* TEC: Top-End Chiller
* TF: Transfer Function

Components
----------

* EUI (LabVIEW). Sends commands to the PXI and reads command acknowledgements, using TCP/IP through the Operation Manager. Gets state from the PXI using LabVIEW Network Shared Variables.
* HHD (LabVIEW). Same communication scheme as the EUI.
* TCS (C++). Tekniker's CSC code; written as part of the Operation Manager. Reads SAL MTMount Commands and writes SAL MTMount Events. Obsolete.
* CSC (Python): the code in this ts_MTMount package. This replaces Tekniker's TCS code. Initially it will communicate with the Operation Manager using the HHD port, but we plan to ask Tekniker to provide us a new TCP/IP port dedicated to the CSC.
* Operation Manager (C++). Talks to the EUI, HHD and PXI via TCP/IP. Coordinates who can talk to the PXI.
* PXI (LabVIEW): there are two low level PXI computers running LabVIEW. One controls the azimuth and altitude axes and the other does the remaining low level control, including communication with the Operation Manager using TCP/IP and writing SAL MTMount Telemetry topics.

TCP/IP Protocol
---------------

The TCP/IP communication protocol is as follows:

* Each end connects via two sockets: one client and one server.
* Each socket is only used for data in one direction, e.g. commands are written to one socket and command acknowledgements are read from the other.
* Data is sent as ASCII with `\n` as a field separator and `\r\n` for "end of text".
  The fields depend on the type of message.

There are TCP/IP connections between the following components and the Operations Manager:

* EUI
* HHD
* PXI

In addition we will ask Tekniker to provide a new port for the CSC.

Commands
^^^^^^^^

Commands have the following fields:

* sequence_id (int): an incrementing value supplied by the commander (e.g. EUI or CSC) used to identify the command in replies.
* command_code (int): the command code
* source (int): who initiated the command
* timestamp (str): timestamp in ISO format (UTC)
* additional parameters, if any. Their number and meaning depends on the ``command_code``.

Replies
^^^^^^^

Ack, NoAck and Done command replies have the following fields:

* reply_code (int): 0=Ack, 1=NoAck, 2=Done.
* sequence_id (int): the value specified in the command.
* source (int): who initiated the command
* timestamp (str): timestamp in ISO format (UTC)
* One additional parameter, if relevant:

  * Ack: command timeout value as an integer in milliseconds.
    Add 2 seconds to this value if you wish to use this for a timeout timer.
  * NoAck: an explanation of why the command was rejected.
  * Done: no additional parameter

Warning and Error events have the following fields:

* reply_code (int): 3=Error, 4=Warning.
* on (int) (*only present for Errors*): 1 = error latched.
  When the error condition is first active this field is set to 1 and remains one until the error is reset.
* active (int): 0 = condition is not present, 1 = condition is present.
* code (int): code number of event.
  The code numbers consist of a `SubsystemId` plus a condition-specific value.
* subsystem (str): subsystem with the problem, in the format f"{subsystem_id}. {component}", where:

    * ``subsystem_id`` (int): subsystem ID: a `SubsystemId` value.
    * ``component`` (str): the component of the subsystem.
      Here are three examples provided by Alberto: "Azimuth", "Trajectory generator", "MyTopVI/MyNextVI/MyNextNextVI".
* timestamp (str): timestamp in ISO format (UTC).
* description (str): description of the problem.
  Note: Tekniker's code includes all remaining message text in this field, but Tekniker assures me that the string will never include `\n`.

OnStateInfo replies report the state of the TCS. They are the only replies initiated by the Operation Manager instead of the PXI:

* reply_code (int): 5=OnStateInfo.
* timestamp (str): timestamp in ISO format (UTC).
* description (str): primary and secondary state, concatenated. For example "PublishOnlyWaitingForCommand".
  I do not know all possible values, but we don't plan to use the TCS so it doesn't matter.

InPositionReply replies indicate if the Azimuth or Elevation axes are in position.
Tekniker is still working on the details.
The current format (which is not adequate) is as follows:

* reply_code (int): 6=InPositionReply.
* timestamp (str): timestamp in ISO format (UTC).
* in_position (bool): in position?
