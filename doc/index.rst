.. py:currentmodule:: lsst.ts.mtmount

###############
lsst.ts.mtmount
###############

.. image:: https://img.shields.io/badge/SAL\ Interface-gray.svg
    :target: https://ts-xml.lsst.io/sal_interfaces/MTMount.html
.. image:: https://img.shields.io/badge/GitHub-gray.svg
    :target: https://github.com/lsst-ts/ts_mtmount
.. image:: https://img.shields.io/badge/Jira-gray.svg
    :target: https://jira.lsstcorp.org/issues/?jql=labels%3Dts_mtmount

The MTMount CSC controls the Simonyi Survey Telescope main axes and related components at Vera C. Rubin Observatory.

The MTMount CSC also communicates with the low level Telescope Mount Assembly (TMA) controller, written by Tekniker.
Also the MTMount CSC monitors the MTRotator CSC in order to command the camera cable wrap (CCW) to follow the rotator.

.. .. _lsst.ts.mtmount-user_guide:

User Guide
==========

Start the MTMount CSC as follows:

.. prompt:: bash

    run_mtmount.py

Stop the CSC by sending it to the OFFLINE state.

See MTMount `SAL communication interface <https://ts-xml.lsst.io/sal_interfaces/MTMount.html>`_ for commands, events and telemetry.

When the MTMount CSC is started (sent from STANDBY to DISABLED state) it launches a background process
to read telemetry from the low-level controller and publish it as SAL/DDS.

To control the MTMount CSC from the command line (e.g. for engineering), you may use the CSC commander.
Please only do this if you are sure it will not interfere with operations:

.. prompt:: bash

    command_mtmount.py --host ccw-mgmt.cp.lsst.org

To generate the **MTMount_Telemetry.xml** file automatically for `ts_xml <https://github.com/lsst-ts/ts_xml>`_ to use, you can use the telemetry configuration parser script, passing the `TelemetryTopicsConfiguration <https://github.com/lsst-ts/ts_tma_labview_hmi-computers/blob/develop/Configuration/TelemetryTopicsConfiguration.ini>`_ file:

.. prompt:: bash

    run_tma_telemetry_config_parser TelemetryTopicsConfiguration.ini --output <path-to-xml>

You may want to put the **TelemetryTopicsConfiguration.ini** and **MTMount_Telemetry.xml** in ``tests/data/tma_telemetry_config`` directory.
Note the **TelemetryTopicsConfiguration.ini** is renamed to be **good_TelemetryTopicsConfiguration.ini**.

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
