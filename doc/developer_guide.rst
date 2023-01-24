.. py:currentmodule:: lsst.ts.mtmount

.. _lsst.ts.mtmount.developer_guide:

###############
Developer Guide
###############

The mtmount CSC is implemented using `ts_salobj <https://github.com/lsst-ts/ts_salobj>`_.

The CSC controls the main telescope mount using a TCP/IP connection to a low-level controller provided by Tekniker.
It also reads telemetry from another TCP/IP connection using a background process (to avoid being overloaded).

See :ref:`lsst.ts.mtmount-tma_interface` for information about the low-level controller.

.. _lsst.ts.mtmount.api:

API
===

The primary classes are:

* `MTMountCsc`: control for the mount.
* `TelemetryClient`: read telemetry and translated it to SAL/DDS.

.. automodapi:: lsst.ts.mtmount
    :no-main-docstr:

.. _lsst.ts.mtmount.build:

Build and Test
==============

This is a pure python package. There is nothing to build except the documentation.

.. code-block:: bash

    make_idl_files.py MTMount
    setup -r .
    pytest -v  # to run tests
    package-docs clean; package-docs build  # to build the documentation

Interface with the Telescope Mount Assembly
===========================================

.. toctree::
    tma_interface
    :maxdepth: 2

.. _lsst.ts.mtmount.contributing:

Contributing
============

``lsst.ts.mtmount`` is developed at https://github.com/lsst-ts/ts_mtmount.
Bug reports and feature requests use `Jira with labels=ts_mtmount <https://jira.lsstcorp.org/issues/?jql=project%3DDM%20AND%20labels%3Dts_mtmount>`_.
