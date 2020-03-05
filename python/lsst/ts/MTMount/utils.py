# This file is part of ts_MTMount.
#
# Developed for the LSST Data Management System.
# This product includes software developed by the LSST Project
# (https://www.lsst.org).
# See the COPYRIGHT file at the top-level directory of this distribution
# for details of code ownership.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

__all__ = ["get_tai_time", "get_utc_time", "wrap_parameter_doc"]

import textwrap

import astropy.time


def get_tai_time():
    """Get current TAI as an astropy.time.Time.
    """
    return astropy.time.Time(astropy.time.Time.now(), scale="tai")


def get_utc_time():
    """Get current UTC as an astropy.time.Time.
    """
    return astropy.time.Time.now()


_ParamWrapper = textwrap.TextWrapper(
    width=79, initial_indent="    ", subsequent_indent="    "
)


def wrap_parameter_doc(text):
    """Wrap a parameter description appropriately for a doc string.

    Parameters
    ----------
    doc : `str`
       Documentation to wrap; typically a `FieldInfo.doc`.

    Returns
    -------
    wrapped_doc : `str`
        The doc wrapped to 79 characters with 4 spaces of indentation.
    """
    return _ParamWrapper.fill(text)
