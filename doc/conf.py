"""Sphinx configuration file for an LSST stack package.

This configuration only affects single-package Sphinx documentation builds.
"""

from documenteer.sphinxconfig.stackconf import build_package_configs
import lsst.ts.MTMount


_g = globals()
_g.update(
    build_package_configs(
        project_name="ts_MTMount", version=lsst.ts.MTMount.version.__version__
    )
)
