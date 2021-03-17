.. py:currentmodule:: lsst.ts.MTMount

.. _lsst.ts.MTMount.developer_guide:

###############
Developer Guide
###############

The MTMount CSC is implemented using `ts_salobj <https://github.com/lsst-ts/ts_salobj>`_.

The CSC controls the main telescope mount using a TCP/IP connection to a low-level controller provided by Tekniker.
It also reads telemetry from another TCP/IP connection using a background process (to avoid being overloaded).

See :ref:`lsst.ts.MTMount-tma_interface` for information about the low-level controller.

.. _lsst.ts.MTMount.api:

API
===

The primary classes are:

* `MTMountCsc`: control for the mount.
* `TelemetryClient`: read telemetry and translated it to SAL/DDS.

.. automodapi:: lsst.ts.MTMount
    :no-main-docstr:

.. _lsst.ts.MTMount.build:

Build and Test
==============

This is a pure python package. There is nothing to build except the documentation.

.. code-block:: bash

    make_idl_files.py MTMount
    setup -r .
    pytest -v  # to run tests
    package-docs clean; package-docs build  # to build the documentation

Tekniker Info
=============

.. toctree::
    tma_interface
    :maxdepth: 2

.. _lsst.ts.MTMount.contributing:

Contributing
============

``lsst.ts.MTMount`` is developed at https://github.com/lsst-ts/ts_MTMount.
Bug reports and feature requests use `Jira with labels=ts_MTMount <https://jira.lsstcorp.org/issues/?jql=project%20%3D%20DM%20AND%20labels%20%20%3D%20ts_MTMount>`_.
