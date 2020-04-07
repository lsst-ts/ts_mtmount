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

import enum
import unittest

import astropy.time

from lsst.ts import MTMount


class ExampleEnum(enum.Enum):
    one = 1
    forty_seven = 47
    negative_one = -1


class FieldInfoTestCase(unittest.TestCase):
    def setUp(self):
        self.name = "otho"
        self.doc = f"a doc string for the field named {self.name}"

    def check_name_doc(self, field_info, name=None, doc=None):
        """Check name and doc attributes of a FieldInfo.

        Parameters
        ----------
        field_info : `FieldInfo`
            Field info to check
        name : `str` or `None` (optional)
            Expected name, or `None` to use ``self.name``
        doc : `str` or `None` (optional)
            Expected doc, or `None` to use ``self.doc``
        """
        if name is None:
            name = self.name
        if doc is None:
            doc = self.doc
        self.assertEqual(field_info.name, name)
        self.assertEqual(field_info.doc, doc)

    def check_field_basics(self, field_info, str_value_dict, bad_values):
        """Check basic field methods.

        Parameters
        ----------
        field_info : `FieldInfo`
            Field info to check
        str_value_dict : `dict` [`str`, `any`]
            Dict of valid string value: desired converted value
        bad_values : `List` [`any`]
            List of invalid values.
        """
        for strval, expected_value in str_value_dict.items():
            with self.subTest(strval=strval, expected_value=expected_value):
                value = field_info.value_from_str(strval)
                self.assertEqual(value, expected_value)

                field_info.assert_value_ok(value)

                strval_round_trip = field_info.str_from_value(value)
                self.assertEqual(strval_round_trip, strval)

        for bad_value in bad_values:
            with self.subTest(bad_value=bad_value):
                with self.assertRaises(ValueError):
                    field_info.assert_value_ok(bad_value)

    def check_fixed_field_info(self, factory, dtype, expected_name):
        """Check a FixedFieldInfo class or a subclass.

        Parameters
        ----------
        factory : ``callable``
            Function that constructs a `FixedEnumFieldInfo` or subclass.
            It accepts one argument: default.
        dtype : `dtype`
            Enum class of the field.
        expected_name : `str`
            Expected name of field info.
        """
        for default in dtype:
            with self.subTest(default=default):
                field_info = factory(default=default)
                self.assertIs(field_info.dtype, dtype)
                self.assertEqual(field_info.name, expected_name)

                other_values = tuple(value for value in dtype if value != default)
                other_int_values = tuple(value.value for value in other_values)
                self.check_field_basics(
                    field_info=field_info,
                    # "value.value" gives the numeric value of the enum value
                    str_value_dict={str(default.value): default},
                    bad_values=(None, "foo") + other_values + other_int_values,
                )

    def test_bool_field_info(self):
        field_info = MTMount.field_info.BoolFieldInfo(name=self.name, doc=self.doc)
        self.check_name_doc(field_info)
        self.check_field_basics(
            field_info=field_info,
            str_value_dict={"0": False, "1": True},
            bad_values=(None, "1", 0, 1, "F"),
        )

    def test_enum_field_info(self):
        field_info = MTMount.field_info.EnumFieldInfo(
            name=self.name, doc=self.doc, dtype=ExampleEnum
        )
        self.check_name_doc(field_info)
        self.check_field_basics(
            field_info=field_info,
            # "value.value" gives the numeric value of the enum value
            str_value_dict={str(value.value): value for value in ExampleEnum},
            bad_values=(None, 0, 2, "foo"),
        )

    def test_fixed_enum_field_info(self):
        def factory(default):
            return MTMount.field_info.FixedEnumFieldInfo(
                name=self.name, default=default
            )

        self.check_fixed_field_info(
            factory=factory, dtype=ExampleEnum, expected_name=self.name
        )

    def test_float_field_info(self):
        field_info = MTMount.field_info.FloatFieldInfo(name=self.name, doc=self.doc)
        self.check_name_doc(field_info)
        str_value_dict = {
            str(float(value)): value for value in (0, 1, -1, 3.14, 9e99, 1e-5)
        }
        self.check_field_basics(
            field_info=field_info,
            str_value_dict=str_value_dict,
            bad_values=(None, "1.5", "foo"),
        )

    def test_int_field_info(self):
        field_info = MTMount.field_info.IntFieldInfo(name=self.name, doc=self.doc)
        self.check_name_doc(field_info)
        # Note: True and False are valid values (1 and 0, respectively).
        # Floats are also valid values (they get truncated).
        self.check_field_basics(
            field_info=field_info,
            str_value_dict={"0": 0, "-1": -1, "153": 153},
            bad_values=(None, "1.5", "foo"),
        )

    def test_str_field_info(self):
        field_info = MTMount.field_info.StrFieldInfo(name=self.name, doc=self.doc)
        self.check_name_doc(field_info)
        self.check_field_basics(
            field_info=field_info,
            str_value_dict={
                strval: strval for strval in ("0", "None", "Vera Rubin Observatory")
            },
            bad_values=(None, False, True, 1, 5.5),
        )

    def test_time_field_info(self):
        for scale in ("tai", "utc"):
            with self.subTest(scale=scale):
                field_info = MTMount.field_info.TimeFieldInfo(
                    name=self.name, doc=self.doc, scale=scale
                )
                self.check_name_doc(field_info)
                valid_date_str = "2020-04-06T22:33:57.335"
                valid_times = (
                    astropy.time.Time(valid_date_str, format="isot", scale=scale),
                    astropy.time.Time(2000, format="jyear", scale=scale),
                    astropy.time.Time(58884, format="mjd", scale=scale),
                )
                self.check_field_basics(
                    field_info=field_info,
                    str_value_dict={t.isot: t for t in valid_times},
                    bad_values=(None, False, True, 1, 5.5, valid_date_str),
                )

        for bad_scale in ("inches", None, 5):
            with self.subTest(bad_scale=bad_scale):
                with self.assertRaises(ValueError):
                    MTMount.field_info.TimeFieldInfo(
                        name=self.name, doc=self.doc, scale=bad_scale
                    )

    def test_command_code_field_info(self):
        self.check_fixed_field_info(
            MTMount.field_info.CommandCodeFieldInfo,
            dtype=MTMount.CommandCode,
            expected_name="command_code",
        )

    def test_reply_code_field_info(self):
        self.check_fixed_field_info(
            MTMount.field_info.ReplyCodeFieldInfo,
            dtype=MTMount.ReplyCode,
            expected_name="reply_code",
        )

    def test_source_field_info(self):
        for what in ("command", "warning"):
            field_info = MTMount.field_info.SourceFieldInfo(what=what)
            self.assertEqual(field_info.name, "source")
            self.assertIn(what, field_info.doc)
            self.assertIsInstance(field_info, MTMount.field_info.EnumFieldInfo)
            self.assertIs(field_info.dtype, MTMount.Source)

    def test_timestamp_field_info(self):
        field_info = MTMount.field_info.TimestampFieldInfo()
        self.assertIsInstance(field_info, MTMount.field_info.TimeFieldInfo)
        self.assertEqual(field_info.name, "timestamp")
        self.assertEqual(field_info.scale, "utc")


if __name__ == "__main__":
    unittest.main()
