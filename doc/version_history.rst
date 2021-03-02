.. py:currentmodule:: lsst.ts.MTMount

.. _lsst.ts.MTMount.version_history:

###############
Version History
###############

ccw_only 2
==========

Changes:

* `MTMountCsc`: if the enable command fails, give up control.
* Add config parameter camera_cable_wrap_interval.
* `CommandFutures`: make setack safe even if ack is done.
* `AckReply`: make the ``timeout_ms`` field required and fix the code that uses it
  (the code was treating timeout_ms as 0.01 sec, not 0.001 sec).


v0.13.0
=======

Changes:

* Overhaul camera cable wrap control.
  This requires ts_xml 7.2:

    * Rename command ``disableCameraCableWrapTracking`` to ``disableCameraCableWrapFollowing``
    * Rename command ``enableCameraCableWrapTracking`` to ``enableCameraCableWrapFollowing``.
      Make that command wait until camera cable wrap tracking is enabled and fail if it cannot be.
    * Output new event ``cameraCableWrapFollowing``.
    * Simplify the ``cameraCableWrap`` telemetry schema;
      the set position, set velocity and actual accleration cannot be set because the information is not available.
    * Simplify the algorithm for following the camera rotator.
      With recent improvements from Tekniker we can now directly use the rotator demand position and velocity as the camera cable wrap target
      (or actual rotator position and velocity, if actual position is too different from demand position).
    * Limit the camera cable wrap target velocity if the rotator demand velocity is larger than the cable wrap supports.
    * Correctly handle lack of telemetry messages from the camera rotator.
      Stop the camera cable wrap while waiting for rotator telemetry to resume.
    * Add configuration parameter ``max_rotator_position_error``.

* `MTMountCsc`: reset e-stops as part of going to enabled state.

v0.12.1
=======

Changes:

* Fixed setup.py and conda/meta.yaml so the conda build works again.

v0.12.0
=======

Changes:

* Add missing ``description`` field to `replies.WarningReply` and `replies.ErrorReply`.
* Fix the enable tracking low-level commands:

    * Only the command for camera cable wrap has a parameter: on=0/1.
      Specify 0 to pause tracking: while paused the axis halts and tracking commands are ignored.
      Specify 1 to enable tracking or resume paused tracking.
      The use case is to reduce vibration during an exposure.
      Note that `MTMountCsc` does not yet support pausing cable wrap tracking during an exposure.
    * Exit tracking mode using the appropriate stop command, rather than enable tracking with on=0.
* Improve logging when a low-level command fails by not printing a traceback.
* `MTMountCommander`: improve output of the ``cameraCableWrap`` telemetry topic;
  it was constantly output in v0.11.0 because of the ``nan`` values for some fields.
* `TmaCommander`: improve error handling in the tracking sequences.
  Output more information and pause briefly before halting the axis.


v0.11.0
=======

Changes:

* Update to use MTMount instead of NewMTMount IDL files.
  This requires ts_xml 7.1.
* Update to read telemetry from a TCP/IP socket in the low-level controller.
* Update TMA commander:

    * Move the code to a new TmaCommander class.
    * Rename the bin script to ``bin/command_tma.py``.
    * Add two camera cable wrap tracking sequences.
* Fix an error in `CommandFuture` that allowed it to try to set a done Future to a new state.
* Improve the way `MtMountCsc` enables and disables the low-level controller, as follows:

    * Leave the state at DISABLED if any command to enable the low-level systems fail, rather than going to a FAULT state.
      This leaves the telemetry client running.
    * Run all disable commands, even if one fails.
* Work around a bug in the AskForCommand low-level command by pausing briefly after issuing it.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for MTMount and MTRotator from ts_xml 7.1

v0.10.0
=======

Changes:

* Rename ``bin/zrun_mtmount_commander.py`` to ``bin/command_mtmount.py`` to match naming in other packages.
* Change the ``--log-level`` command-line argument to ``--loglevel`` for ``bin/run_mock_tma.py`` and ``bin/tma_commander.py``, to match the command-line argument for running CSCs.
* In simulation mode start the mock controller process just before connecting to the low-level controller, and terminate it just after disconnecting.
  This slows down the `start` command but allows recovery if something goes wrong with the mock controller.
* Improve error handling if a TCP/IP server cannot be constructed.
  This fixes a source of silent errors and a failure mode where ``run_mock_tma.py`` could not be terminated.
* Log more information in `Communicator` connection monitoring.
* Removed ``Commander`` enum; use ``Source`` instead with the `AskForCommand` command.
* Use ``pre-commit`` instead of a custom git pre-commit hook.
  See ``README.rst`` for instructions.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for NewMTMount, MTMount, and MTRotator from ts_xml 7

v0.9.0
======

Changes:

* Update the `MTMountCsc` to send the ``ASK_FOR_COMMAND`` low-level command when going to ``ENABLED`` state.
  Only send device initialization and shutdown commands if the CSC has command.
* Add more commands to the TMA commander.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for NewMTMount, MTMount, and MTRotator from ts_xml 7

v0.8.1
======

Changes:

* Update Jenkinsfile.conda to use the shared library.
* Pin the versions of ts_idl and ts_salobj in conda/meta.yaml.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for NewMTMount, MTMount, and MTRotator from ts_xml 7

v0.8.0
======

Changes:

* Update to use and require ts_xml 7.

    * Use MTRotator's ``rotation`` telemetry topic instead of Rotator's ``Application`` telemetry topic
      (in the camera cable wrap following code).
    * Improve use of MTMount telemetry in the same code.
      Adjust the camera cable wrap position to match the camera rotator timestamp,
      and use what are likely better fields for that position.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for NewMTMount, MTMount, and MTRotator from ts_xml 7

v0.7.4
======

Changes:

* Add run_mock_tma.py script to setup.py.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.7.3
======

Changes:

* Fix a bug in the close method of the mock controller.
  It would try to close the communicator even if was still None.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.7.2
======

Changes:

* Fix a bug that prevents the CSC from starting the mock TMA controller.
* Added missing ``enable`` constructor argument to `MTMountCommander`.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.7.1
======

Changes:

* Fix the requirements information in the version history for v0.6.0, v0.6.1, and v0.7.0.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.7.0
======

Changes:

* This release requires ts_salobj 6.
* Simplified the simulation mode support, using ts_salobj 6-specific features.
* Added class attribute ``version`` to `MTMountCsc`.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.6.1
======

Changes:

* Fix bin/run_mtmount.py so that it works with ts_salobj 6 (and 5).
* Add a unit test of bin/run_mtmount.py.

Requires:

* ts_salobj 5.15 or 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 1 (with salobj 5) or 2 (with salobj 6)
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.6.0
======

Changes:

* In simulation mode have the `MTMountCSC` run the mock controller in a subprocess,
  in order to give the CSC a better chance of keeping up with tracking commands.
  This eliminates the `MTMountCSC.mock_controller` attribute.
* Add `MTMountCsc` constructor argument ``run_mock_controller``
  to control whether the CSC runs the mock controller in simulation mode
  (if false then you must run the mock controller yourself).
  This supports unit tests that need access to the mock controller --
  access that is difficult if the CSC runs the mock controller in a subuprocess.

Requires:

* ts_salobj 5.15
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 1
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.5.0
======

Changes:

* Send camera cable wrap tracking commands in advance, by a configurable duration.
* Make the CSC enable camera cable wrap tracking when first enabled.

Requires:

* ts_salobj 5.15
* ts_simactuators 2
* ts_hexrotcomm
* ts_idl
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.4.0
======

Changes:

* Update CCW-Rotator synchronization algorithm to account for the current position of the CCW when computing the CCW demand.

Requires:

* ts_salobj 5.15
* ts_simactuators 2
* ts_hexrotcomm
* ts_idl
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.3.0
======

Changes:

* Update the motion limits for the simulator with more realistic values.

Requires:

* ts_salobj 5.15
* ts_simactuators 2
* ts_hexrotcomm
* ts_idl
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.2.0
======

Changes:

* Updated for ts_simactuators 2
* Changed ``Limits.scale`` to `Limits.scaled`.
  It now returns a scaled copy instead of modifying the instance in place.
* Added minimal camera cable wrap telemetry to the mock controller.
* Added this version history.

Requires:

* ts_salobj 5.15
* ts_simactuators 2
* ts_hexrotcomm
* ts_idl
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.1.0
======

Initial release

Requires:

* ts_salobj 5.11
* ts_simactuators 1
* ts_hexrotcomm
* ts_idl
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8
