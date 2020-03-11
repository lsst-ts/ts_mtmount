##########
ts_MTMount
##########

``ts_MTMount`` is a CSC that controls the Simonyi Survey Telescope main axes and related components.

`Documentation <https://ts-mtmount.lsst.io>`_

The package is compatible with the `eups <https://github.com/RobertLuptonTheGood/eups>`_ package management system and ``scons`` build system.
Assuming you have the basic Vera C. Rubin LSST DM stack installed you can do the following, from within the package directory:

* ``setup -r .`` to setup the package and dependencies.
* ``scons`` to build the package and run unit tests.
* ``scons install declare`` to install the package and declare it to eups.
* ``package-docs build`` to build the documentation.
  This requires ``documenteer``; see `building single package docs <https://developer.lsst.io/stack/building-single-package-docs.html>`_ for installation instructions.

This code is automatically formatted by ``black`` using a git pre-commit hook.
To enable this:

* Install the ``black`` Python package.
* Run ``git config core.hooksPath .githooks`` once in this repository.
