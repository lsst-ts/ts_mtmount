.. py:currentmodule:: lsst.ts.MTMount

.. _lsst.ts.MTMount.version_history:

###############
Version History
###############

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
