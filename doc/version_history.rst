.. py:currentmodule:: lsst.ts.mtmount

.. _lsst.ts.mtmount.version_history:

###############
Version History
###############

v0.23.4
-------

* `MTMountCsc`:

    * Reject most commands while devices are being disabled (e.g. while going to fault).
    * Lock tracking-related commands separately from other commands, so that commands such as disableCameraCableWrapFollowing will not interfere with tracking.

* `MTMountCommander`: remove a bit of residual code needed for salobj &lt; 7.3.

Requires:

* ts_salobj 7.3
* ts_simactuators 2
* ts_tcpip 0.3.7
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 15

v0.25.3
-------

* `MTMountCsc`: fix the disable_devices method to properly send the disable commands.
* ``Jenkinsfile``: update for a newer version of the shared library.

Requires:

* ts_salobj 7.3
* ts_simactuators 2
* ts_tcpip 0.3.7
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 15

v0.25.2
-------

* `MTMountCsc`: Log warnings for excessive delays in the event loop.
* ``Jenkinsfile``: use new shared library.

Requires:

* ts_salobj 7.3
* ts_simactuators 2
* ts_tcpip 0.3.7
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 15

v0.25.1
-------

* `MTMountCsc`:

  * When disabling devices eliminate a source of cascading error messages by giving up after the first error
    (and then trying to yield control and stopping the low level heartbeat command).
  * Make retry_command more robust by giving the TMA time to fail a command with "no ack seen in 500ms".
    This avoids prematurely retrying a command.

Requires:

* ts_salobj 7.3
* ts_simactuators 2
* ts_tcpip 0.3.7
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 15

v0.25.0
-------

* `MTMountCsc`:

    * When going to fault, try to stay connected while giving up control.
      This will improve output if an axis goes to fault, and generally give more useful output while in fault state.
    * Retry low-level commands that stop motion and turn off subsystems.
    * Fix a bug that prevented leaving enabled state if something went wrong while disabling (DM-37972).
    * Go to fault if a ``trackTarget`` command times out.
    * Go to fault if the CSC loses command (e.g. if the EUI or HHD takes command).
    * Do not try to disable TMA components if the CSC is no longer the commander.
      This should reduce noise when going to disabled or fault.
    * Reduce latency in camera cable wrap following the rotator.
    * Improve logging in camera cable wrap following code.
    * Remove ts_xml 14 backwards compatibility code (DM-37114).
    * Remove Python 3.8 backwards compatibility code.

* `MTMountCommander`:

    * Hide more unwanted data, to avoid overwhelming the display, including:

      * Never publish oilSupplySystem telemetry.
      * Only publish cameraCableWrapTarget and clockOffset if values change significantly.
      * Only publish the most important fields of logMessage: level, name, message, and (if not empty) traceback.

    * Use an async callback for telemetry handling.
      This requires ts_salobj 7.3.

* `monitor_mtmount_telemetry`: make this command-line utility much more flexible.
  You can now specify which topics to monitor, how long to monitor them, and how many messages to skip (per topic).

* Simplify the code for low-level commands (cleanup that has been pending since the low-level controller started issuing replies as yaml):

    * Change commands to set ``sequence_id`` and ``timestamp`` fields to zero, instead of guessing correct values.
      `MTMountCsc` is now setting those fields, as are unit tests that generate low-level commands.
    * Rename ``BaseMessage`` to ``BaseCommand``, since we no longer use it for replies.
    * Delete the ``Command`` class and use ``BaseCommand`` instead.
    * Delete the ``TimestampFieldInfo`` class; use ``FloatFieldInfo`` instead.

* pre-commit: update black to 23.1.0, isort to 5.12.0, mypy to 1.0.0, and pre-commit-hooks to v4.4.0.
* ``Jenkinsfile``: do not run as root.

Requires:

* ts_salobj 7.3
* ts_simactuators 2
* ts_tcpip 0.3.7
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 15

v0.24.0
-------

* `MTMountCsc`:

    * Publish the ``azimuthHomed`` and ``elevationHomed`` events.
      This requires ts_xml 14.
    * When disconnecting give control to nobody instead of to the EUI (now that the TMA has been updated to make this practical).
    * Handle the OIL_SUPPLY_SYSTEM_STATE low-level event.
    * Fix a resource leak in the command cache: commands that were done when acknowledged were not removed.
    * Improve robustness of the trackTarget command by waiting for the communication lock to be available before measuring whether the command is too late.

* `TelemetryClient`:

    * Publish all telemetry topics defined in ts_xml 14.
      Do this in a way that handles the rename of topic "oSS" to "oilSupplySystem" in ts_xml 15.
    * Publish the clockOffset event if using ts_xml 15.

* `mock.Controller`:

    * Pubish AXIS_HOMED events for azimuth and elevation.
    * Add missing camera cable wrap telemetry fields.
    * Fix a bug that could cause output data to be written in a separate message from its terminator.

* `TMATelemetryConfigParser`: write the data needed for RAW_TELEMETRY_MAP.
* `MTMountCommander`: show most telemetry (not ``encoder`` or ``oSS/oilSupplySystem``).
* test_csc: test_tracking was not working as designed, and was ignoring some errors.
* ``conda/meta.yaml``: remove redundant ``entry_points`` section.

Requires:

* ts_salobj 7.1
* ts_simactuators 2
* ts_tcpip 0.3.7
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 14

v0.23.2
-------

* `MTMountCSC`: improve error reporting when a low-level command fails.

Requires:

* ts_salobj 7.1
* ts_simactuators 2
* ts_tcpip 0.3.7
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 13

v0.23.1
-------

* `MTMountCSC`:

    * Fail pending low-level commands on disconnect.
    * Ignore trackTarget commands if the tracking advance time is too small, but log a warning.

Requires:

* ts_salobj 7.1
* ts_simactuators 2
* ts_tcpip 0.3.7
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 13

v0.23.0
-------

* Publish the ``connected`` and ``telemetryConnected`` events.
  This requires ts_xml 13.
* `MTMountCSC`:

    * Enable the oil supply system as part of enabling subsystems, now that the TMA can control it.
    * Only issue the low-level heartbeat command when the CSC has control of the TMA.
    * Change the ``startTracking``, ``stop``, and ``stopTracking`` commands to report an in-progress ACK.
      This means a normal timeout should be sufficient for these commands.
    * Give control of the TMA to the EUI when going to fault state, to avoid turning off the oil supply system and main power supply (which are slow to turn back on).
    * Go to fault if telemetry stops arriving from the TMA.
    * Go to fault if the azimuth, elevation, or camera cable wrap axis faults, with new error code ``CscErrorCode.AXIS_FAULT``.
    * Log commands sent and command replies received at level 15 (halfway between info and debug).
    * Add support for the GetActualSettings command and use it to get camera cable wrap motion constraints.
    * Make subsystem enable and disable more reliable by waiting briefly between commands.
    * Remove the ``wait_done`` argument from the ``send_command`` method; always wait.
    * Improved timestamps in commands sent to the TMA.
    * Fix a bug in the ``monitor_telemetry_client`` method.
    * Fix a bug in the camera cable wrap following code.
    * Fix a bug in handling the TMA limits event for some systems.

* `MTMountCommander`: improve uniformity and advance time of tracking commands sent by the ramp command.
* `TelemetryClient`:

    * Simplify the code by assuming that the field names reported by the TMA match those in SAL.
      This works because we can specify the field names reported by the TMA.
    * Fail if no telemetry received from the TMA for long enough.

* Add bin/run_tma_telemetry_config_parser to generate MTMount_Telemetry.xml from the TMA telemetry config file.
* Sort imports with isort, and enforce with pre-commit.
* ``test_mock_devices.py``: fix deprecation warnings caused by no running event loop when constructing mock devices.

Requires:

* ts_salobj 7.1
* ts_simactuators 2
* ts_tcpip 0.3.7
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 13

v0.22.2
-------

* Update entrypoints in pyproject.toml
* In conda recipe, add entry points and replace py.test with pytest, running in verbose mode.
* Update run_mtmount_telemetry_client entrypoint and bin script.
* Update run_mtmount entrypoint and bin script.
* Update run_mock_tma entrypoint and bin script.
* Update monitor_mtmount_telemetry entrypoint and bin script.
* Update command_tma entrypoint and bin script.
* Update command_mtmount entrypoint and bin script.

Requires:

* ts_salobj 7.1
* ts_simactuators 2
* ts_tcpip 0.3.7
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 12

v0.22.1
-------

* Rename bin scripts to remove ".py" extension.
* pyproject.toml: add missing entries to ``[project.scripts]``.
* Jenkinsfile: work around a new git permission issue.
* Suppress N818 flake8 warning.

Requires:

* ts_salobj 7.1
* ts_simactuators 2
* ts_tcpip 0.3.7
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 12

v0.22.0
-------

* `MTMountCsc`:

  * Call ``super().start()`` at the beginning of the start method.
    This requires ts_salobj 7.1.
  * Report camera cable wrap actual torque (percentage).
    This requires ts_xml 12.
  * Make going to fault more robust when the connection to the low-level controller is lost.
  * Reset the oil supply system alarms when resetting other alarms.
  * Turn on the oil supply system before the main axes power supply, instead of after.
  * Build using pyproject.toml.

* `TelemetryClient`:

    * Make the controller write-only.
      This requires ts_salobj 7.1.
    * Stop publishing actual acceleration.
      It is not available for the azimuth and acceleration axes, and is probably too noisy to be useful for camera cable wrap.

* `mock.Controller`: eliminate the code that detects if the telemetry client drops the connection.
  This requires ts_tcpip 0.4.
* ``setup.cfg``: set asyncio_mode = auto.
* Modified a unit test to work with the kafka version of ts_salobj.
* git ignore .hypothesis.
* Modernize ``Jenkinsfile``.

Requires:

* ts_salobj 7.1
* ts_simactuators 2
* ts_tcpip 0.3.7
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 12

v0.21.1
-------

* `MTMountCsc`: stop writing the appliedSettingsMatchStart event.
* `CONFIG_SCHEMA`: delete default values.

Requires:

* ts_salobj 7
* ts_simactuators 2
* ts_tcpip 0.1
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 11

v0.21.0
-------

* Update for ts_salobj v7, which is required.
  This also requires ts_xml 11.
* Added bin/monitor_mtmount_telemetry.py.
* `mock.AxisDevice`: fix another instance of incorrect text in an out-of-range error message.

Requires:

* ts_salobj 7
* ts_simactuators 2
* ts_tcpip 0.1
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 11

v0.20.1
-------

* Expand the elevation limits back to 0, 90, to match LTS-103.
* `mock.AxisDevice`: fix the "out of range" error message.
  It was printing the minimum value as the upper limit, instead of the maximum value.

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_tcpip 0.1
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 10.1

v0.20.0
-------

* Publish new events based on DETAILED_SETTINGS_APPLIED event from the low-level controller.
* Limit the camera cable wrap commanded position to be within acceptable limits,
  using data from the DETAILED_SETTINGS_APPLIED event from the low-level controller.
* Renamed ``LimitsDict`` to `mock.CmdLimitsDict` and update the values to match the current values.
* `mock.AxisDevice`: add ``cmd_limits`` attribute and enforce those limits for point-to-point moves and tracking commands.
* `mock.BaseDevice`: add ``__repr__`` method.
* Modernize the unit tests to use bare assert and a few pytest functions.
* Increase the tracking timeout interval in the mock axis controllers to 5 seconds (from 1 second), to match the real low-level controller.
* Document some of the data in the DETAILED_SETTINGS_APPLIED event from the low-level controller in tma_interface.rst.

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_tcpip 0.1
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 10.1

v0.19.1
-------

* Use ts_utils.
* Fix tests/test_csc.py; two tests were failing because they did not provide regular rotation telemetry.
* Fix a typo in bin/command_mtmount.py.

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_tcpip 0.1
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 10.0

v0.19.0
-------

* Add support for all but one of the new low-level controller events.
  The one missing event is DETAILED_SETTINGS_APPLIED;
  its documentation is incomplete and we need to decide which of the many fields to publish.
  This version requires ts_xml 10.0 and ts_idl 3.2.
* Lock the low-level TCP/IP stream for a few more commands,
  to reduce the chance of sending a command that will be rejected.
* Rename the package from ts_MTMount to ts_mtmount,
  and the Python namespace from lsst.ts.MTMount to lsst.ts.mtmount.

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_tcpip 0.1
* ts_idl 3.2
* IDL files for MTMount and MTRotator from ts_xml 10.0

0.18.1
-------

* Make camera cable wrap (CCW) following more robust by not locking the low-level TCP/IP stream while commands run
  (except in limited cases, such as initializing subsystems and shutting them back down).
  This fixes DM-30990: moveToTarget causes CCW following to fail.

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_tcpip 0.1
* ts_idl 3.1
* IDL files for MTMount and MTRotator from ts_xml 7.2

v0.18.0
-------

* Update to use ts_tcpip instead of ts_hexrotcomm.
* Test black formatting with pytest, instead of tests/test_black.py.

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_tcpip 0.1
* ts_idl 2
* IDL files for MTMount and MTRotator from ts_xml 7.2

v0.17.1
-------

* Format the code with black 20.8b1.

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for MTMount and MTRotator from ts_xml 7.2

v0.17.0
-------

* Fix two bugs that prevented the CSC from outputting telemetry after going to standby and back to disabled state:

    * `MTMountCsc`: the CSC was not reliably shutting down the telemetry client.
    * `mock.Controller`: the mock simulator was not reliably stopping and restarting the telemetry loop.
      This was due a bug in `lsst.ts.hexrotcomm.OneClientServer` (fixed in v0.17.0),
      but I added simple workaround in the mock controller for that kind of error.
* `mock.AxisDevice`: implement realistic handling of late tracking commands.
* `MtMountCsc`: improve handling of several commands:

    * moveToTarget: output the ``target`` event and return an IN_PROGRESS ack with a realistic timeout.
    * open/closeMirrorCovers: return an IN_PROGRESS ack with an upper limit timeout.

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for MTMount and MTRotator from ts_xml 7.2

v0.16.0
-------

* `MTMountCsc`: improve camera cable wrap following startup and shutdown,
  including more reliably stopping the axis.
* `MTMountCsc`: bug fix: it was using the wrong telemetry port in normal mode (not simulating).
* Update unit tests to use `unittest.IsolatedAsyncioTestCase` instead of the abandoned ``asynctest`` package.
* Update code to use the ``LINE_TERMINATOR`` constant.
* Modernize the documentation: add a User Guide section to the main documentation page
  and move the developer information to a separate Developer Guide.
* Modernize doc/conf.py for documenteer 0.6.

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for MTMount and MTRotator from ts_xml 7.2

v0.15.0
-------

* `MTMountCsc` (and, where relevant, `mock.Controller`) updates:

    * Support new command acknowledgement events: ``superseded`` and ``failed``.
    * Support new event format: json-encoded dict.
    * Disable devices and give up control if the ``enable`` command fails.
    * The ``stop`` command now stops mirror cover and mirror cover lock motion,
      in addition to the main axes and camera cable wrap.

* `Command`: update for command timestamps changing from UTC ISO to TAI unix seconds.
* Add configuration parameter ``camera_cable_wrap_interval``.
* Store the CSC configuration schema in code.
  This requires ts_salobj 6.3.
* Store the telemetry map in code instead of a separate yaml file.

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for MTMount and MTRotator from ts_xml 7.2

v0.14.0
-------

* Use a single socket for commands and replies.
* `mock.Controller` related changes: 
    * Replaced ``command_port`` and ``telemetry_port`` constructor argument with ``random_ports``
    * Removed the ``reconnect`` argument.
    * Updated the command-line arguments of ``run_mock_tma.py`` to match.

* `MTMountCsc` updated for the changes in `mock.Controller`.
* `MTMountCommander` updated to use `lsst.ts.simactuators.RampGenerator`, for a more accurate ramp.
* `mock`: add ``INITIAL_POSITION`` dict and use it to set the initial position of the mock axis actuators.
  Change the initial elevation to 80 degrees.
* Modernize ``doc/conf.py`` for documenteer 0.6.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for MTMount and MTRotator from ts_xml 7.2

v0.13.0
-------

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

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for MTMount and MTRotator from ts_xml 7.2

v0.12.1
-------

* Fixed setup.py and conda/meta.yaml so the conda build works again.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for MTMount and MTRotator from ts_xml 7.1

v0.12.0
-------

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

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for MTMount and MTRotator from ts_xml 7.1

v0.11.0
-------

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
-------

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
------

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
------

* Update Jenkinsfile.conda to use the shared library.
* Pin the versions of ts_idl and ts_salobj in conda/meta.yaml.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for NewMTMount, MTMount, and MTRotator from ts_xml 7

v0.8.0
------

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
------

* Add run_mock_tma.py script to setup.py.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.7.3
------

* Fix a bug in the close method of the mock controller.
  It would try to close the communicator even if was still None.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.7.2
------

* Fix a bug that prevents the CSC from starting the mock TMA controller.
* Added missing ``enable`` constructor argument to `MTMountCommander`.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.7.1
------

* Fix the requirements information in the version history for v0.6.0, v0.6.1, and v0.7.0.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 2
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.7.0
------

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
------

* Fix bin/run_mtmount.py so that it works with ts_salobj 6 (and 5).
* Add a unit test of bin/run_mtmount.py.

Requires:

* ts_salobj 5.15 or 6
* ts_simactuators 2
* ts_hexrotcomm 0.9
* ts_idl 1 (with salobj 5) or 2 (with salobj 6)
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.6.0
------

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
------

* Send camera cable wrap tracking commands in advance, by a configurable duration.
* Make the CSC enable camera cable wrap tracking when first enabled.

Requires:

* ts_salobj 5.15
* ts_simactuators 2
* ts_hexrotcomm
* ts_idl
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.4.0
------

* Update CCW-Rotator synchronization algorithm to account for the current position of the CCW when computing the CCW demand.

Requires:

* ts_salobj 5.15
* ts_simactuators 2
* ts_hexrotcomm
* ts_idl
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.3.0
------

* Update the motion limits for the simulator with more realistic values.

Requires:

* ts_salobj 5.15
* ts_simactuators 2
* ts_hexrotcomm
* ts_idl
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8

v0.2.0
------

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
------

Initial release

Requires:

* ts_salobj 5.11
* ts_simactuators 1
* ts_hexrotcomm
* ts_idl
* IDL files for NewMTMount, MTMount, and Rotator from ts_xml 4.8
