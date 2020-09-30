.. py:currentmodule:: lsst.ts.MTMount

.. _lsst.ts.MTMount.version_history:

###############
Version History
###############

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

v0.5.0
======

Changes:

* Send camera cable wrap tracking commands in advance, by a configurable duration.
* Make the CSC enable camera cable wrap tracking when first enabled.

Requires:

* ts_salobj 5.15
* ts_simactuators 2
* ts_idl
* IDL files for NewMTMount and MTMount from ts_xml 4.8

v0.4.0
======

Changes:

* Update CCW-Rotator synchronization algorithm to account for the current position of the CCW when computing the CCW demand.

Requires:

* ts_salobj 5.15
* ts_simactuators 2
* ts_idl
* IDL files for NewMTMount and MTMount from ts_xml 4.8

v0.3.0
======

Changes:

* Update the motion limits for the simulator with more realistic values.

Requires:

* ts_salobj 5.15
* ts_simactuators 2
* ts_idl
* IDL files for NewMTMount and MTMount from ts_xml 4.8

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
* ts_idl
* IDL files for NewMTMount and MTMount from ts_xml 4.8

v0.1.0
======

Initial release

Requires:

* ts_salobj 5.11
* ts_simactuators 1
* ts_idl
* IDL files for NewMTMount and MTMount from ts_xml 4.8
