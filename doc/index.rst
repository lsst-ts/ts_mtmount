.. py:currentmodule:: lsst.ts.mtmount

###############
lsst.ts.mtmount
###############

.. image:: https://img.shields.io/badge/SAL\ Interface-gray.svg
    :target: https://ts-xml.lsst.io/sal_interfaces/mtmount.html
.. image:: https://img.shields.io/badge/GitHub-gray.svg
    :target: https://github.com/lsst-ts/ts_mtmount
.. image:: https://img.shields.io/badge/Jira-gray.svg
    :target: https://jira.lsstcorp.org/issues/?jql=labels+%3D+ts_mtmount

The MTMount CSC controls the Simonyi Survey Telescope main axes and related components at Vera C. Rubin Observatory.

The MTMount CSC communicates with the low level Telescope Mount Assembly (TMA) controller, written by Tekniker.
The MTMount CSC also monitors MTRotator in order to command the camera cable wrap (CCW) to follow the rotator.

.. .. _lsst.ts.mtmount-user_guide:

User Guide
==========

Start the MTMount CSC as follows:

.. prompt:: bash

    run_mtmount.py

Stop the CSC by sending it to the OFFLINE state.

See MTMount `SAL communication interface <https://ts-xml.lsst.io/sal_interfaces/mtmount.html>`_ for commands, events and telemetry.

When MTMount is started (sent from STANDBY to DISABLED state) it launches a background process
to read telemetry from the low-level controller and publish it as SAL/DDS.

To control the MTMount CSC from the command line (e.g. for engineering), you may use the CSC commander.
Please only do this if you are sure it will not interfere with operations:

.. prompt:: bash

    command_mtmount.py --host ccw-mgmt.cp.lsst.org

.. _lsst.ts.mtmount-pyapi:

Developer Guide
===============

.. toctree::
    developer_guide
    :maxdepth: 1

Version History
===============

.. toctree::
    version_history
    :maxdepth: 1
