.. py:currentmodule:: lsst.ts.MTMount

.. _lsst.ts.MTMount.version_history:

###############
Version History
###############

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
