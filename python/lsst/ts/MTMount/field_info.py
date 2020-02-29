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

__all__ = [
    "BaseFieldInfo",
    "BoolFieldInfo",
    "EnumFieldInfo",
    "FixedEnumFieldInfo",
    "FloatFieldInfo",
    "IntFieldInfo",
    "StrFieldInfo",
    "TimeFieldInfo",
    "CommandCodeFieldInfo",
    "ReplyCodeFieldInfo",
    "SourceFieldInfo",
    "TimestampFieldInfo",
]

import abc
import enum

import astropy.time

from . import enums


class BaseFieldInfo(metaclass=abc.ABCMeta):
    """Information about one field of a message.

    Parameters
    ----------
    name : `str`
        Name of field. Must be a valid Python identifier.
    doc : `str`
        Description of the field.
    default : ``any`` or `None`
        Default value (using the native type of this FieldInfo).
        If `None` then this field must be specified
        when constructing a command or reply.
    """

    def __init__(self, name, doc, default=None):
        self.name = name
        self.doc = doc
        self._default = default
        if default is not None:
            self.assert_value_ok(default)

    @property
    def default(self):
        return self._default

    @abc.abstractmethod
    def assert_value_ok(self, value):
        """Raise ValueError if the specified value is not of the correct type.
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def value_from_str(self, strval):
        """Convert a string to a value of the correct type.

        Parameters
        ----------
        strval : `str`
            String representation of value.

        Returns
        -------
        value : ``any``
            The parsed value.

        Raises
        ------
        ValueError
            If the value cannot be parsed.
        """
        raise NotImplementedError()

    def str_from_value(self, value):
        """Return a string representation of the value.
        """
        return str(value)


class BoolFieldInfo(BaseFieldInfo):
    """A bool field with str representation "0"/"1".
    """

    def assert_value_ok(self, value):
        if not isinstance(value, bool):
            raise ValueError(f"value={value!r} is not a bool")

    def value_from_str(self, strval):
        return bool(int(strval))

    def str_from_value(self, value):
        return "1" if value else "0"


class EnumFieldInfo(BaseFieldInfo):
    """An enum field.

    Parameters
    ----------
    name : `str`
        Name of field. Must be a valid Python identifier.
    doc : `str`
        Description of the field.
    EnumClass : ``enum``
        Class of enum; any of the enum classes from the enum package is fine.
    default : ``EnumClass`` or `None`
        Default value.
        If `None` then this field must be specified
        when constructing a command or reply.
    """

    def __init__(self, name, doc, EnumClass, default=None):
        self.EnumClass = EnumClass
        super().__init__(name=name, doc=doc, default=default)

    def assert_value_ok(self, value):
        self.EnumClass(value)

    def value_from_str(self, strval):
        intval = int(strval)
        return self.EnumClass(intval)

    def str_from_value(self, value):
        return str(value.value)


class FixedEnumFieldInfo(EnumFieldInfo):
    """Information for an enum field that must be a given enum value.

    Parameters
    ----------
    name : `str`
        Name of field. Must be a valid Python identifier.
    default : `enum.Enum`
        The required enum value for this field.
    """

    def __init__(self, name, default):
        if not isinstance(default, enum.Enum):
            raise ValueError(f"default={default} is not an enum instance.")
        super().__init__(
            name=name,
            doc=f"{name} field with a fixed value: {default!r}",
            EnumClass=type(default),
            default=default,
        )

    def assert_value_ok(self, value):
        if value != self.default:
            raise ValueError(f"value={value!r} must be {self.default!r}")

    def value_from_str(self, strval):
        value = super().value_from_str(strval)
        self.assert_value_ok(value)
        return value


class FloatFieldInfo(BaseFieldInfo):
    """A float field.
    """

    def assert_value_ok(self, value):
        if not (isinstance(value, float) or isinstance(value, int)):
            raise ValueError(f"value={value!r} is not a float or int")

    def value_from_str(self, strval):
        return float(strval)

    def str_from_value(self, value):
        return str(float(value))


class IntFieldInfo(BaseFieldInfo):
    """An int field.
    """

    def assert_value_ok(self, value):
        if not isinstance(value, int):
            raise ValueError(f"value={value!r} is not an int")

    def value_from_str(self, strval):
        return int(strval)


class StrFieldInfo(BaseFieldInfo):
    """A str field.
    """

    def assert_value_ok(self, value):
        if not isinstance(value, str):
            raise ValueError(f"value={value!r} is not a str")

    def value_from_str(self, strval):
        return strval


class TimeFieldInfo(BaseFieldInfo):
    """Date and time field.

    The str representation is ISO-8601, with "T" between the date and time.
    For example: "2020-02-27T14:48:27.469".

    Parameters
    ----------
    name : `str`
        Name of field. Must be a valid Python identifier.
    doc : `str` (optional)
        Description of the field.
    scale : `str`
        One of "tai" or "utc". Support for other values supported by
        `astropy.time` can easily be added, if needed. But Tekniker's
        code only uses UTC and TAI.
    default : `True` or `None`
        If `True` then the current date is used as a default.
        If `None` then the date must be specified.
    """

    def __init__(self, name, doc, scale, default=None):
        if scale not in ("tai", "utc"):
            raise ValueError(f"Unsupported scale {scale}")
        if default not in (True, None):
            raise ValueError(f"default={default} must be True or None")
        self.scale = scale
        # Don't pass default=True to the parent class,
        # because it is not a valid time.
        super().__init__(name=name, doc=doc, default=None)
        self._default = default

    @property
    def default(self):
        if self._default is None:
            return None
        return astropy.time.Time(astropy.time.Time.now(), scale=self.scale)

    def assert_value_ok(self, value):
        if not isinstance(value, astropy.time.Time):
            raise ValueError(f"value={value!r} is not an astropy.time.Time")

    def value_from_str(self, strval):
        return astropy.time.Time(strval, scale=self.scale)

    def str_from_value(self, value):
        return value.isot


# Convenience versions of the fields above


class CommandCodeFieldInfo(FixedEnumFieldInfo):
    """Information for the ``command_code`` field of command.

    Parameters
    ----------
    default : `CommandCode`
        The command code for this command.
    """

    def __init__(self, default):
        enums.CommandCode(default)  # Check value.
        super().__init__(name="command_code", default=default)


class ReplyCodeFieldInfo(FixedEnumFieldInfo):
    """Information for the ``command_code`` field of reply.

    Parameters
    ----------
    default : `ReplyCode`
        The reply code for this reply.
    """

    def __init__(self, default):
        enums.ReplyCode(default)  # Check value.
        super().__init__(name="reply_code", default=default)


class SourceFieldInfo(EnumFieldInfo):
    """Information for the ``source`` field.

    Parameters
    ----------
    what : `str`
        Kind of message, e.g. "command" or "warning".
        Used as part of the doc string.
    """

    def __init__(self, what):
        super().__init__(
            name="source",
            doc=f"Source of the {what}; a `Source`",
            EnumClass=enums.Source,
            # TODO: change this when we have a value for the CSC
            default=enums.Source.HHD,
        )


class TimestampFieldInfo(TimeFieldInfo):
    """Information for the ``timestamp`` field.
    """

    def __init__(self):
        super().__init__(
            name="timestamp",
            doc="Time at which the message was sent.",
            scale="utc",
            default=True,
        )
