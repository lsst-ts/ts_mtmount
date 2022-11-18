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

import yaml

# Basic information to translate telemetry from the low-level controller
# to SAL telemetry and back.
#
# RAW_TELEMETRY_MAP is a dict of:
# ``topic_id: [sal_topic_name, field_len_dict]``, where:
#
# * ``topic_id`` is the low-level controller topic ID (an integer).
#   These IDs must match the TopicID entries in the telemetry config file.
# * ``sal_topic_name`` is the SAL telemetry topic name.
# * ``field_len_dict`` is a dict of ``field_name: num_elements``
# * ``num_elements`` is the number of array elements of that SAL field
#   (1 for a scalar).
#
# Note: in order to simplify the code, there is a 1:1 mapping between:
#
# * Low-level controller topic IDs and SAL telemetry topics.
# * Low-level controller telemetry field names and SAL field names.
RAW_TELEMETRY_MAP = yaml.safe_load(
    """

6: # fields are in flux
- azimuth
- actualPosition: 1
  demandPosition: 1
  actualVelocity: 1
  demandVelocity: 1
  actualTorque: 1
  timestamp: 1

5:
- azimuthDrives
- current: 16
  timestamp: 1

# 4:
# - azimuthDrivesThermal
# - actualSurfaceTemperature: 16
#   demandTemperature: 4
#   actualValvePosition: 4
#   demandValvePosition: 4
#   timestamp: 1

15: # fields are in flux
- elevation
- actualPosition: 1
  demandPosition: 1
  actualVelocity: 1
  demandVelocity: 1
  actualTorque: 1
  timestamp: 1

14:
- elevationDrives
- current: 12
  timestamp: 1

# 13:
# - elevationDrivesThermal
# - actualSurfaceTemperature: 12
#   demandTemperature: 2
#   actualValvePosition: 2
#   demandValvePosition: 2
#   timestamp: 1

8:
- cameraCableWrap
  # Note: the low-level controller reports consolidated angle and speed
  # and drive-specific angles and speeds. At present MTMount
  # only reports the consolidated values.
- actualPosition: 1
  actualVelocity: 1
  actualTorquePercentage: 2
  timestamp: 1
"""
)


class BaseTelemetryFieldFunctor(abc.ABC):
    """Functor to extract the value for one SAL field
    from low-level telemetry field(s) and vice-versa.

    Parameters
    ----------
    num_elements : `int`
        The number of elements in the low-level telemetry.
    field_name : `str`
        Name template for low-level controller telemetry fields.
    """

    def __init__(self, num_elements, field_name):
        self.num_elements = num_elements
        self.field_name = field_name

    @abc.abstractmethod
    def sal_value_from_llv_dict(self, data_dict):
        """Get the value for one SAL field from low-level telemetry data.

        Parameters
        ----------
        data_dict : `dict`
            Dict of field_name: value. For an array, the dict
            must contain all fields that belong in that array.
            For a scalar, this must include just the one field.
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
    field_name : `str`
        The name of the low-level controller telemetry field.
        Must not contain "{" or "}".

    Raises
    ------
    ValueError
        If num_elements != 1.
    """

    def __init__(self, num_elements, field_name):
        if num_elements != 1:
            raise ValueError(f"{num_elements=} must be 1")
        super().__init__(num_elements=num_elements, field_name=field_name)

    def sal_value_from_llv_dict(self, data_dict):
        return data_dict[self.field_name]

    def llv_dict_from_sal_value(self, value):
        return {self.field_name: value}


class ArrayTelemetryFieldFunctor(BaseTelemetryFieldFunctor):
    """Functor to extract the value for one SAL array field
    from a sequence of low-level telemetry field.

    Parameters
    ----------
    num_elements : `int`
        The number of elements in the low-level telemetry; must be > 1
    field_name : `str`
        Template for the name of the low-level controller telemetry field.
        Must contain one template section that takes an integer.

    Raises
    ------
    ValueError
        If num_elements <= 1.
    """

    def __init__(self, num_elements, field_name):
        if num_elements <= 1:
            raise ValueError(f"{num_elements=} must be > 1")
        self.field_name_template = f"{field_name}{{0:d}}"
        super().__init__(num_elements=num_elements, field_name=field_name)

    def sal_value_from_llv_dict(self, data_dict):
        return [
            data_dict[self.field_name_template.format(i)]
            for i in range(1, self.num_elements + 1)
        ]

    def llv_dict_from_sal_value(self, value):
        if len(value) != self.num_elements:
            raise ValueError(
                f"{value=} must contain exactly {self.num_elements} elements"
            )
        return {
            self.field_name_template.format(i + 1): item for i, item in enumerate(value)
        }


#: Dict of topic_id: [sal_topic_name, field_extraction_func_dict] where:
#:
#: * ``topic_id`` is the integer identifier of the low-level telemetry topic.
#: * ``sal_topic_name`` is the SAL telemetry topic name
#: * ``field_extraction_func_dict`` is a dict of
#:   ``field_name``: ``field_extraction_func``, where:
#:
#:   * ``field_name`` is the name of the SAL telemetry topic field
#:   * ``field_extraction_func`` is an instance of `BaseTelemetryFieldFunctor`
#:     which can convert between SAL and low-level data for that field.
TELEMETRY_MAP = dict()
for topic_id, topic_data in RAW_TELEMETRY_MAP.items():
    if len(topic_data) != 2:
        raise ValueError(
            f"cannot parse {topic_id=}: {topic_data=} must have two elements"
        )
    sal_topic_name = topic_data[0]
    field_len_dict = topic_data[1]
    field_extraction_func_dict = dict()
    for (field_name, num_elements) in field_len_dict.items():
        try:
            if num_elements == 1:
                field_extraction_func = ScalarTelemetryFieldFunctor(
                    num_elements=num_elements, field_name=field_name
                )
            else:
                field_extraction_func = ArrayTelemetryFieldFunctor(
                    num_elements=num_elements, field_name=field_name
                )

            field_extraction_func_dict[field_name] = field_extraction_func
        except Exception as e:
            raise ValueError(f"Cannot parse {topic_id=} {field_name=}: {e!r}")
    TELEMETRY_MAP[topic_id] = [sal_topic_name, field_extraction_func_dict]
