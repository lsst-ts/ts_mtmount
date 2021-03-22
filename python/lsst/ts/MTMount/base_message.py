# This file is part of ts_MTMount.
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

__all__ = ["BaseMessage"]

import enum

import astropy.time
from . import constants


class BaseMessage:
    """BaseMessage data.

    Parameters
    ----------
    kwargs : `dict`
        Keyword arguments. Names and types are specified by `FieldInfo`
        instances in class variable ``field_infos``.
        Fields with a default value may be omitted.
    extra_data : `List` [`str`]
        This argument is required if class variable ``has_extra_data`` is true,
        and prohibited otherwise.

    Notes
    -----
    Set the following class variables:

    field_infos : `List` [`BaseFieldInfo`]
        Information about the fields for this message.
        The number and meaning is specific to the message type.
    has_extra_data : `bool`
        Can this message have additional data?
        If True then:

        * The constructor takes argument ``extra_data``:
          a tuple or list of 0 or more strings.
        * This data is available as attribute ``extra_data``:
          a tuple of the same strings.
        * Warning: behavior is undefined if the constructor receives
          a tuple or list with non-string elements.
        The default is False, since few messages have extra data.
    """

    has_extra_data = False

    def __init__(self, **kwargs):
        for finfo in self.field_infos:
            field_name = finfo.name
            value = kwargs.get(field_name)
            if value is None:
                value = finfo.default
                if value is None:
                    raise ValueError(f"{finfo.name} is a required argument")
            else:
                finfo.assert_value_ok(value)
            setattr(self, field_name, value)
        if self.has_extra_data:
            self.extra_data = tuple(kwargs.get("extra_data", ()))

    @classmethod
    def from_str_fields(cls, fields):
        """Construct a BaseMessage from a list of string fields.

        Parameters
        ----------
        fields : `List` [`str`]
            Data fields, as strings:

            * There must be at least one field per ``field_infos`` entry.
            * If ``has_extra_data`` is treue then additional fields are
              provided as the ``extra_data`` argument,
              else additional fields result in an error.

        Raises
        ------
        ValueError
            If the wrong number of fields is presented or the data cannot
            be parsed as the kind of message indicated by its command_code
            field (for commands) or reply_code field (for replies).
        """
        num_field_infos = len(cls.field_infos)
        if cls.has_extra_data:
            if len(fields) < num_field_infos:
                raise ValueError(
                    f"{cls.__name__} requires at least {num_field_infos} fields, "
                    f"but got {len(fields)}: {fields}"
                )
        else:
            if len(fields) != num_field_infos:
                raise ValueError(
                    f"{type(cls).__name__} requires exactly {num_field_infos} fields, "
                    f"but got {len(fields)}: {fields}"
                )
        kwargs = {
            finfo.name: finfo.value_from_str(fields[i])
            for i, finfo in enumerate(cls.field_infos)
        }
        if cls.has_extra_data:
            kwargs["extra_data"] = tuple(fields[num_field_infos:])
        return cls(**kwargs)

    def encode(self):
        """Return the data encoded as a bytes string,
        including the standard terminator.
        """
        data_str = "\n".join(self.str_fields())
        return data_str.encode() + constants.LINE_TERMINATOR

    def str_fields(self):
        """Return the data as a list of string fields.
        """
        str_list = [
            field.str_from_value(getattr(self, field.name))
            for field in self.field_infos
        ]
        if self.has_extra_data:
            str_list += list(self.extra_data)
        return str_list

    def _get_formatted_value(self, name):
        value = getattr(self, name)
        if isinstance(value, enum.Enum):
            return repr(value)
        elif isinstance(value, astropy.time.Time):
            return value.isot
        return str(value)

    def __eq__(self, other):
        return self.str_fields() == other.str_fields()

    def __repr__(self):
        arglist = [
            f"{finfo.name}={self._get_formatted_value(finfo.name)}"
            for finfo in self.field_infos
        ]
        argstr = ", ".join(arglist)
        return f"{type(self).__name__}({argstr})"
