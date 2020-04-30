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
    "Reply",
    "AckReply",
    "NoAckReply",
    "DoneReply",
    "WarningReply",
    "ErrorReply",
    "OnStateInfoReply",
    "InPositionReply",
    "TelemetryReply",
    "TelemetryOSS5Reply",
    "Replies",
    "ReplyDict",
    "parse_reply",
]

from . import base_message
from . import enums
from . import field_info
from . import utils


class Reply(base_message.BaseMessage):
    """Base class for replies.

    This primarily exists to allow better documentation of APIs
    and to allow testing for "is a reply".
    """

    pass


def make_reply_doc(cls):
    """Make and attach a doc string to a reply class.
    """
    short_class_name = cls.__name__
    if short_class_name.endswith("Reply"):
        short_class_name = short_class_name[:-5]
    param_strings = []
    for finfo in cls.field_infos:
        param_doc = utils.wrap_parameter_doc(finfo.doc)
        is_optional = finfo.default is not None
        optional_str = " (optional)" if is_optional else ""
        param_strings.append(
            f"{finfo.name} : `{finfo.dtype.__name__}{optional_str}`\n{param_doc}"
        )
    param_block = "\n".join(param_strings)
    cls.__doc__ = f"""{short_class_name} command.

Parameters
----------
{param_block}
"""


def make_reply_field_infos(reply_code, field_infos):
    """Make a list of FieldInfo for the specified reply.

    Parameters
    ----------
    reply_code : `ReplyCode`
        Reply code.
    field_infos : `List` [`FieldInfo`]
        Information for fields after the ``reply_code`` field.
    """
    for finfo in field_infos:
        if not isinstance(finfo, field_info.BaseFieldInfo):
            raise ValueError(
                f"field_infos={field_infos} is not a sequence of field_info.BaseFieldInfo"
            )
    return (field_info.ReplyCodeFieldInfo(reply_code),) + field_infos


# All command responses (ACK, NOACK and DONE replies)
# have the following fields, and possibly before one additional field.
_ResponseFieldInfos = (
    field_info.IntFieldInfo(
        name="sequence_id", doc="sequence_id of command being acknowledged."
    ),
    field_info.SourceFieldInfo(what="command being replied to"),
    field_info.TimestampFieldInfo(),
)


class AckReply(Reply):
    field_infos = make_reply_field_infos(
        enums.ReplyCode.ACK,
        _ResponseFieldInfos
        + (
            field_info.IntFieldInfo(
                name="timeout_ms", doc="Expected duration of command (msec)."
            ),
        ),
    )


class NoAckReply(Reply):
    field_infos = make_reply_field_infos(
        enums.ReplyCode.NOACK,
        _ResponseFieldInfos
        + (
            field_info.StrFieldInfo(
                name="explanation", doc="Explanation of the problem."
            ),
        ),
    )


class DoneReply(Reply):
    field_infos = make_reply_field_infos(enums.ReplyCode.DONE, _ResponseFieldInfos)


class WarningReply(Reply):
    field_infos = make_reply_field_infos(
        enums.ReplyCode.WARNING,
        (
            field_info.BoolFieldInfo(name="active", doc="Is the condition present?"),
            field_info.IntFieldInfo(
                name="code",
                doc="Warning/error code number; A SubsystemId plus a condition-specific code.",
            ),
            field_info.SubsystemFieldInfo(what="warning"),
            field_info.TimestampFieldInfo(),
        ),
    )
    has_extra_data = True


class ErrorReply(Reply):
    field_infos = make_reply_field_infos(
        enums.ReplyCode.ERROR,
        (
            field_info.BoolFieldInfo(name="on", doc="Is the error latched?"),
            field_info.BoolFieldInfo(
                name="active", doc="Is the condition still present?",
            ),
            field_info.IntFieldInfo(
                name="code",
                doc="Warning/error code number; A SubsystemId plus a condition-specific code.",
            ),
            field_info.SubsystemFieldInfo(what="error"),
            field_info.TimestampFieldInfo(),
        ),
    )
    has_extra_data = True


class OnStateInfoReply(Reply):
    field_infos = make_reply_field_infos(
        enums.ReplyCode.ON_STATE_INFO,
        (
            field_info.TimestampFieldInfo(),
            field_info.StrFieldInfo(name="description", doc="?"),
        ),
    )


class InPositionReply(Reply):
    field_infos = make_reply_field_infos(
        enums.ReplyCode.IN_POSITION,
        (
            field_info.TimestampFieldInfo(),
            field_info.IntFieldInfo(name="what", doc="0 for azimuth, 1 for elevation"),
            field_info.BoolFieldInfo(
                name="in_position", doc="True if in position, False if not"
            ),
        ),
    )


class TelemetryReply(Reply):
    """Telemetry information

    Parameters
    ----------
    reply_code : `ReplyCode`
        Reply code. Must be TELEMETRY.
    timestamp : `astropy.time.Time` or `None`
        Timestamp. If `None` use the current time.
    topic : `int`
        The subsystem: 0 for azimuth, 1 for elevation.
    data : `str`
        The data, encoded as comman-separated values
    """

    field_infos = make_reply_field_infos(
        enums.ReplyCode.TELEMETRY,
        (
            field_info.TimestampFieldInfo(),
            field_info.StrFieldInfo(name="name", doc="telemetry topic name"),
            field_info.StrFieldInfo(name="data", doc="telemetry data, encoded as json"),
        ),
    )


class TelemetryOSS5Reply(Reply):
    """OSS5 telemetry data"""

    field_infos = make_reply_field_infos(
        enums.ReplyCode.TELEMETRY_OSS5,
        (
            field_info.TimestampFieldInfo(),
            field_info.FloatFieldInfo(name="CFM_5011", doc="data"),
            field_info.FloatFieldInfo(name="CFM_5012", doc="data"),
            field_info.FloatFieldInfo(name="CFM_5013", doc="data"),
            field_info.FloatFieldInfo(name="CFM_5041", doc="data"),
            field_info.FloatFieldInfo(name="CFM_5042", doc="data"),
            field_info.FloatFieldInfo(name="CFM_5043", doc="data"),
            field_info.FloatFieldInfo(name="CPM_5011", doc="data"),
            field_info.FloatFieldInfo(name="CPM_5012", doc="data"),
            field_info.FloatFieldInfo(name="CPM_5013", doc="data"),
            field_info.FloatFieldInfo(name="CPM_5014", doc="data"),
            field_info.FloatFieldInfo(name="CPM_5015", doc="data"),
            field_info.FloatFieldInfo(name="CPM_5016", doc="data"),
            field_info.FloatFieldInfo(name="CPM_5041", doc="data"),
            field_info.FloatFieldInfo(name="CPM_5042", doc="data"),
            field_info.FloatFieldInfo(name="CPM_5043", doc="data"),
            field_info.FloatFieldInfo(name="CPM_5044", doc="data"),
            field_info.FloatFieldInfo(name="CPM_5045", doc="data"),
            field_info.FloatFieldInfo(name="CPM_5046", doc="data"),
            field_info.FloatFieldInfo(name="CTM_5011", doc="data"),
            field_info.FloatFieldInfo(name="CTM_5041", doc="data"),
            field_info.FloatFieldInfo(name="EL_XPos_M1_left_Bearing_1", doc="data"),
            field_info.FloatFieldInfo(name="EL_XPos_M1_left_Bearing_2", doc="data"),
            field_info.FloatFieldInfo(name="EL_XPos_M1_left_Bearing_3", doc="data"),
            field_info.FloatFieldInfo(
                name="EL_XPos_M1_left_Bearing_calculated", doc="data"
            ),
            field_info.FloatFieldInfo(name="EL_XPos_M1_right_Bearing_1", doc="data"),
            field_info.FloatFieldInfo(name="EL_XPos_M1_right_Bearing_2", doc="data"),
            field_info.FloatFieldInfo(name="EL_XPos_M1_right_Bearing_3", doc="data"),
            field_info.FloatFieldInfo(
                name="EL_XPos_M1_right_Bearing_calculated", doc="data"
            ),
            field_info.FloatFieldInfo(name="EL_XPos_M2_left_Bearing_1", doc="data"),
            field_info.FloatFieldInfo(name="EL_XPos_M2_left_Bearing_2", doc="data"),
            field_info.FloatFieldInfo(name="EL_XPos_M2_left_Bearing_3", doc="data"),
            field_info.FloatFieldInfo(
                name="EL_XPos_M2_left_Bearing_calculated", doc="data"
            ),
            field_info.FloatFieldInfo(name="EL_XPos_M2_right_Bearing_1", doc="data"),
            field_info.FloatFieldInfo(name="EL_XPos_M2_right_Bearing_2", doc="data"),
            field_info.FloatFieldInfo(name="EL_XPos_M2_right_Bearing_3", doc="data"),
            field_info.FloatFieldInfo(
                name="El_XPos_M2_right_Bearing_calculated", doc="data"
            ),
            field_info.FloatFieldInfo(name="EL_XNeg_M1_left_Bearing_1", doc="data"),
            field_info.FloatFieldInfo(name="EL_XNeg_M1_left_Bearing_2", doc="data"),
            field_info.FloatFieldInfo(name="EL_XNeg_M1_left_Bearing_3", doc="data"),
            field_info.FloatFieldInfo(
                name="EL_XNeg_M1_left_Bearing_calculated", doc="data"
            ),
            field_info.FloatFieldInfo(name="EL_XNeg_M1_right_Bearing_1", doc="data"),
            field_info.FloatFieldInfo(name="EL_XNeg_M1_right_Bearing_2", doc="data"),
            field_info.FloatFieldInfo(name="EL_XNeg_M1_right_Bearing_3", doc="data"),
            field_info.FloatFieldInfo(
                name="EL_XNeg_M1_right_Bearing_calculated", doc="data"
            ),
            field_info.FloatFieldInfo(name="EL_XNeg_M2_left_Bearing_1", doc="data"),
            field_info.FloatFieldInfo(name="EL_XNeg_M2_left_Bearing_2", doc="data"),
            field_info.FloatFieldInfo(name="EL_XNeg_M2_left_Bearing_3", doc="data"),
            field_info.FloatFieldInfo(
                name="EL_XNeg_M2_left_Bearing_calculated", doc="data"
            ),
            field_info.FloatFieldInfo(name="EL_XNeg_M2_right_Bearing_1", doc="data"),
            field_info.FloatFieldInfo(name="EL_XNeg_M2_right_Bearing_2", doc="data"),
            field_info.FloatFieldInfo(name="EL_XNeg_M2_right_Bearing_3", doc="data"),
            field_info.FloatFieldInfo(
                name="El_XNeg_M2_right_Bearing_calculated", doc="data"
            ),
        ),
    )


Replies = (
    AckReply,
    NoAckReply,
    DoneReply,
    WarningReply,
    ErrorReply,
    OnStateInfoReply,
    InPositionReply,
    TelemetryReply,
    TelemetryOSS5Reply,
)

for reply in Replies:
    make_reply_doc(reply)


def _make_reply_dict():
    """Make a dict of reply_code: ReplyClass from `Replies`.

    Check that no reply codes are duplicated.
    """
    reply_dict = {}
    for reply in Replies:
        reply_code = reply.field_infos[0].default
        if reply_code in reply_dict:
            raise RuntimeError(
                "The reply code appears twice: "
                f"once in {reply_dict[reply_code].__name__}; "
                f"and once in {reply.__name__}"
            )
        reply_dict[reply_code] = reply
    return reply_dict


# Dict of CommandCode: CommandClass
ReplyDict = _make_reply_dict()


def parse_reply(fields):
    """Parse a set of strings as a reply.

    Parameters
    ----------
    fields : `List` [`str`]
        Fields from a read message.
        The fields should not be terminated with ``\n``.

    Raises
    ------
    ValueError
        If the data cannot be parsed.
    """
    if len(fields) < 1:
        raise ValueError("No fields provided")
    reply_code = enums.ReplyCode(int(fields[0]))
    try:
        ReplyClass = ReplyDict[reply_code]
    except KeyError:
        raise RuntimeError(f"Invalid reply_code={reply_code}")
    return ReplyClass.from_str_fields(fields)
