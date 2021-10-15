# This file is part of ts_mtmount.
#
# Developed for Rubin Observatory Telescope and Site Systems.
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

__all__ = ["truncate_value", "wrap_parameter_doc"]

import textwrap

# Maximum documentation string length (chars)
MAX_DOC_LENGTH = 79


_ParamWrapper = textwrap.TextWrapper(
    width=MAX_DOC_LENGTH, initial_indent="    ", subsequent_indent="    "
)


def truncate_value(value, min_value, max_value, descr):
    """Truncate a value, if necessary.

    Returns
    -------
    A tuple:
    - truncated_value: one of value, min_value, or max_value, as appropriate.
    - message in format f"{descr} from {value:0.2f} to {truncated_value:0.2f}"
      or "" if the value is not truncated.

    Raises
    ------
    ValueError
        If min_value >= max_value.
    """
    if min_value >= max_value:
        raise ValueError(
            f"Invalid {descr} limits: min_value={min_value} >= {max_value}=max_value"
        )
    if value < min_value:
        return (min_value, f"{descr} from {value:0.2f} to {min_value:0.2f}")
    elif value > max_value:
        return (max_value, f"{descr} from {value:0.2f} to {max_value:0.2f}")
    return value, ""


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
