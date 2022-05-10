"""Sphinx configuration file for an LSST stack package.

This configuration only affects single-package Sphinx documentation builds.
"""

from documenteer.conf.pipelinespkg import *  # type: ignore # noqa
import lsst.ts.mtmount  # noqa

project = "ts_mtmount"
html_theme_options["logotext"] = project  # type: ignore # noqa
html_title = project
html_short_title = project
doxylink = {}  # Avoid warning: Could not find tag file _doxygen/doxygen.tag

intersphinx_mapping["ts_xml"] = ("https://ts-xml.lsst.io", None)  # type: ignore # noqa
intersphinx_mapping["ts_salobj"] = ("https://ts-salobj.lsst.io", None)  # type: ignore # noqa
intersphinx_mapping["ts_utils"] = ("https://ts-utils.lsst.io", None)  # type: ignore # noqa
intersphinx_mapping["ts_tcpip"] = ("https://ts-tcpip.lsst.io", None)  # type: ignore # noqa
