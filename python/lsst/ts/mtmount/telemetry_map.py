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

__all__ = ["TELEMETRY_MAP"]

import abc
import re

import yaml

# Basic information to translate telemetry from the low-level controller
# to SAL telemetry and back.
#
# RAW_TELEMETRY_MAP is a dict of:
# ``topic_id: [sal_topic_name, field_translation_dict]``, where:
#
# * ``topic_id`` is the low-level controller topic ID (an integer).
#   These IDs must match the TopicID entries in the telemetry config file.
# * ``sal_topic_name`` is the SAL telemetry topic name.
# * ``field_translation_dict`` is a dict of
#   ``sal_field_name: [num_elements, llv_name_template]``,
#   information used to map between one SAL telemetry field and one or more
#   low-level controller fields, where:
#
#   * ``sal_field_name`` is the name of a SAL telemetry field.
#   * ``num_elements`` it the number of array elements of that SAL field.
#     1 indicates a scalar, and num_elements must be ≥ 1.
#   * ``llv_name_template`` is a template for the name of the low-level
#     controller field(s), with a format that depends on num_elements:
#
#     * if ``num_elements = 1``: ``llv_name_template`` is a low-level
#       telemetry field name.
#     * if ``num_elements > 1``: ``llv_name_template`` is a template string
#       of the form "field_name_prefix{0:d}", or similar, that when formatted
#       with a single integer argument whose value ranges from 1
#       through num_elements, produces a low-level telemetry field name.
#
# Note: in order to simplify the code, there is a 1:1 mapping between
# low-level controller topic IDs and SAL telemetry topics.
RAW_TELEMETRY_MAP = yaml.safe_load(
    """

6: # fields are in flux
- azimuth
- actualPosition: [1, angleActual]
  demandPosition: [1, angleSet]
  actualVelocity: [1, velocityActual]
  demandVelocity: [1, velocitySet]
  actualTorque: [1, torqueActual]
  timestamp: [1, timestamp]

5:
- azimuthDrives
- current: [16, "currentDrive{0:d}"]
  timestamp: [1, timestamp]

# 4:
# - azimuthDrivesThermal
# - surfaceTemperature: [16, "azimuthSurfaceTemperatureDrive{0:d}"]
#   temperatureSetPoint: [4, "(azimuthSurfaceTemperatureSetpointGroup){0:d}"]
#   # what is azimuthValveFeedbackGroup1? do we publish it?
#   timestamp: [1, timestamp]

15: # fields are in flux
- elevation
- actualPosition: [1, angleActual]
  demandPosition: [1, angleSet]
  actualVelocity: [1, velocityActual]
  demandVelocity: [1, velocitySet]
  actualTorque: [1, torqueActual]
  timestamp: [1, timestamp]

14:
- elevationDrives
- current: [12, "currentDrive{0:d}"]
  timestamp: [1, timestamp]

8:
- cameraCableWrap
  # Note: the low-level controller reports consolidated angle and speed
  # and drive-specific angles and speeds. At present MTMount
  # only reports the consolidated values.
- actualPosition: [1, angle]
  actualVelocity: [1, speed]
  actualTorquePercentage: [2, "torquePercentage{0:d}"]
  timestamp: [1, timestamp]
"""
)


class BaseTelemetryFieldFunctor(abc.ABC):
    """Functor to extract the value for one SAL field
    from low-level telemetry field(s) and vice-versa.

    Parameters
    ----------
    num_elements : `int`
        The number of elements in the low-level telemetry.
    llv_name_template : `str`
        Name template for low-level controller telemetry fields.
    """

    def __init__(self, num_elements, llv_name_template):
        self.num_elements = num_elements
        self.llv_name_template = llv_name_template

    @abc.abstractmethod
    def sal_value_from_llv_dict(self, data_dict):
        """Get the value for one SAL field from low-level telemetry data.

        Parameters
        ----------
        data_dict : `dict`
            Dict of llv_field_name: value. This must include all fields
            needed by this SAL field; other entries are ignored.
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def llv_dict_from_sal_value(self, value):
        """Get a dict of low-level telemetry for the fields associated
        with a single SAL field.

        Parameters
        ----------
        value
            The value of the SAL field.
        """
        raise NotImplementedError()


class ScalarTelemetryFieldFunctor(BaseTelemetryFieldFunctor):
    """Functor to extract the value for one SAL scalar field
    from a scalar low-level telemetry field.

    Parameters
    ----------
    num_elements : `int`
        The number of elements in the low-level telemetry; must be 1
    llv_name_template : `str`
        The name of the low-level controller telemetry field.
        Must not contain "{" or "}".

    Raises
    ------
    ValueError
        If num_elements != 1 or llv_name_template contains "{" or "}".
    """

    def __init__(self, num_elements, llv_name_template):
        if num_elements != 1:
            raise ValueError(f"{num_elements=} must be 1")
        if "{" in llv_name_template or "}" in llv_name_template:
            raise ValueError(f"{llv_name_template=} must not contain {{ or }}")
        super().__init__(num_elements=num_elements, llv_name_template=llv_name_template)

    def sal_value_from_llv_dict(self, data_dict):
        return data_dict[self.llv_name_template]

    def llv_dict_from_sal_value(self, value):
        return {self.llv_name_template: value}


class ArrayTelemetryFieldFunctor(BaseTelemetryFieldFunctor):
    """Functor to extract the value for one SAL array field
    from a sequence of low-level telemetry field.

    Parameters
    ----------
    num_elements : `int`
        The number of elements in the low-level telemetry; must be > 1
    llv_name_template : `str`
        Template for the name of the low-level controller telemetry field.
        Must contain one template section that takes an integer.

    Raises
    ------
    ValueError
        If num_elements <= 1 or llv_name_template.format(1) raises.
    """

    def __init__(self, num_elements, llv_name_template):
        if num_elements <= 1:
            raise ValueError(f"{num_elements=} must be > 1")
        try:
            llv_name_template.format(1)
        except Exception as e:
            raise ValueError(f"{llv_name_template=} is not a usable template: {e!r}")
        super().__init__(num_elements=num_elements, llv_name_template=llv_name_template)

    def sal_value_from_llv_dict(self, data_dict):
        return [
            data_dict[self.llv_name_template.format(i)]
            for i in range(1, self.num_elements + 1)
        ]

    def llv_dict_from_sal_value(self, value):
        if len(value) != self.num_elements:
            raise ValueError(
                f"{value=} must contain exactly {self.num_elements} elements"
            )
        return {
            self.llv_name_template.format(i + 1): item for i, item in enumerate(value)
        }


#: Dict of topic_id: [sal_topic_name, field_extraction_func_dict] where:
#:
#: * ``topic_id`` is the integer identifier of the low-level telemetry topic.
#: * ``sal_topic_name`` is the SAL telemetry topic name
#: * ``field_extraction_func_dict`` is a dict of
#:   ``sal_field_name``: ``field_extraction_func``, where:
#:
#:   * ``sal_field_name`` is the name of the SAL telemetry topic field
#:   * ``field_extraction_func`` is an instance of `BaseTelemetryFieldFunctor`
#:     which can convert between SAL and low-level data for that field.
TELEMETRY_MAP = dict()
for topic_id, topic_data in RAW_TELEMETRY_MAP.items():
    if len(topic_data) != 2:
        raise ValueError(
            f"cannot parse {topic_id=}: {topic_data=} must have two elements"
        )
    sal_topic_name = topic_data[0]
    field_translation_dict = topic_data[1]
    field_extraction_func_dict = dict()
    for (
        sal_field_name,
        num_elements_llv_name_template,
    ) in field_translation_dict.items():
        try:
            num_elements, llv_name_template = num_elements_llv_name_template
            # Make sure the template can handle 1
            llv_name_template.format(1)
            if num_elements == 1:

                field_extraction_func = ScalarTelemetryFieldFunctor(
                    num_elements=num_elements, llv_name_template=llv_name_template
                )

            else:
                num_elements_llv_name_template[1] = re.compile(f"^{llv_name_template}$")

                field_extraction_func = ArrayTelemetryFieldFunctor(
                    num_elements=num_elements, llv_name_template=llv_name_template
                )

            field_extraction_func_dict[sal_field_name] = field_extraction_func
        except Exception as e:
            raise ValueError(f"Cannot parse {topic_id=} {sal_field_name=}: {e!r}")
    TELEMETRY_MAP[topic_id] = [sal_topic_name, field_extraction_func_dict]
