##########
ts_mtmount
##########

``ts_mtmount`` is a CSC that controls the Simonyi Survey Telescope main axes and related components.

`Documentation <https://ts-mtmount.lsst.io>`_

This version disables control of the camera cable wrap, oil supply subsystem, and top end chiller.
It is intended for 2022-06 commissioning tests with an incomplete low-level controller.
Do not merge this to the develop or main branches.

The package is compatible with the `eups <https://github.com/RobertLuptonTheGood/eups>`_ package management system and ``scons`` build system.
Assuming you have the basic Vera C. Rubin LSST DM stack installed you can do the following, from within the package directory:

* ``setup -r .`` to setup the package and dependencies.
* ``scons`` to build the package and run unit tests.
* ``scons install declare`` to install the package and declare it to eups.
* ``package-docs build`` to build the documentation.
  This requires ``documenteer``; see `building single package docs <https://developer.lsst.io/stack/building-single-package-docs.html>`_ for installation instructions.

This code uses ``pre-commit`` to maintain ``black`` formatting and ``flake8`` compliance.
To enable this, run the following command once::

    pre-commit install
