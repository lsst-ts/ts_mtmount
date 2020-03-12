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
    "get_random_value",
    "make_random_message",
    "make_random_message_with_defaults",
]

import datetime
import random
import string

import astropy.time

from . import field_info


def get_random_date(start=datetime.date(2000, 1, 1), end=datetime.date(2020, 12, 31)):
    """Return a random date in the specified range.

    The default range is narrow in order to avoid these warnings:
    ErfaWarning: ERFA function "dtf2d" yielded 1 of "dubious year (Note 6)

    Parameters
    ----------
    start : `datetime.date`
        Start date.
    end : `datetime.date`
        End date.

    Returns
    -------
    date : `datetime.date`
        Random date between ``start`` and ``end``.
    """
    return start + (end - start) * random.random()


def get_random_value(finfo):
    """Return a randomly chosen value for the specified field info type.

    Parameters
    ----------
    finfo : `BaseFieldInfo`
        Field info.

    Returns
    -------
    value : ``any``
        Randomly chosen value of the correct type.
    """
    if isinstance(finfo, field_info.FixedEnumFieldInfo):
        # Only one value is valid; return it.
        return finfo.default
    elif isinstance(finfo, field_info.BoolFieldInfo):
        # Only one value is valid; return it.
        return random.choice([False, True])
    elif isinstance(finfo, field_info.EnumFieldInfo):
        return random.choice(list(finfo.dtype))
    elif isinstance(finfo, field_info.IntFieldInfo):
        return random.randint(-10000, 10000)
    elif isinstance(finfo, field_info.FloatFieldInfo):
        return random.uniform(-9e99, 9e99)
    elif isinstance(finfo, field_info.TimeFieldInfo):
        scale = random.choice(["utc", "tai"])
        return astropy.time.Time(get_random_date().isoformat(), scale=scale)
    elif isinstance(finfo, field_info.StrFieldInfo):
        nchar = random.randint(1, 100)
        return "".join(random.sample(string.printable, nchar))
    raise ValueError(f"Unrecognized field type {finfo!r}")


def make_random_message(message_type):
    """Given a Message class, construct a message with random data
    for all fields.

    All fields are specified.

    Parameters
    ----------
    message_type : `BaseMessage`
        Message type

    Returns
    -------
    message : ``message_type``
        Message constructed with random data.
    """
    kwargs = {finfo.name: get_random_value(finfo) for finfo in message_type.field_infos}
    return message_type(**kwargs)


def make_random_message_with_defaults(message_type):
    """Given a Message class, construct a message with default values for
    fields that have them, and random values for fields that do not.

    Parameters
    ----------
    message_type : `BaseMessage`
        Message type

    Returns
    -------
    message : ``message_type``
        Message constructed with random data.
    """
    kwargs = {
        finfo.name: get_random_value(finfo)
        for finfo in message_type.field_infos
        if finfo.default is None
    }
    return message_type(**kwargs)
