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

__all__ = ["RAW_TELEMETRY_MAP", "TELEMETRY_MAP", "TelemetryTopicId"]

import abc
import enum

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
# The yaml data is automatically generated by command-line script
# ``run_tma_telemetry_config_parser``.
#
# Note: there is a 1:1 mapping between:
#
# * Low-level controller topic IDs and SAL telemetry topics.
# * Low-level controller telemetry field names and SAL field names.
#
# So if you want to change a field name, change it in the TMA's configuration
# configuration file, then run run_tma_telemetry_config_parser
# to write new MTMount_Telemetry.xml and new data for RAW_TELEMETRY_MAP.
RAW_TELEMETRY_MAP = yaml.safe_load(
    """

6:
- azimuth
- actualPosition: 1
  actualTorque: 1
  actualVelocity: 1
  demandPosition: 1
  demandVelocity: 1
  timestamp: 1

26:
- safetySystem
- timestamp: 1
  versionNumber: 1

15:
- elevation
- actualPosition: 1
  actualTorque: 1
  actualVelocity: 1
  demandPosition: 1
  demandVelocity: 1
  timestamp: 1

19:
- lockingPins
- actualPosition: 2
  actualTorquePercentage: 2
  demandPosition: 2
  timestamp: 1

11:
- deployablePlatforms
- actualPositionPlatform1Section: 2
  actualPositionPlatform2Section: 2
  actualTorquePercentagePlatform1Drive: 2
  actualTorquePercentagePlatform2Drive: 2
  timestamp: 1

2:
- cabinet0101Thermal
- actualTemperature: 1
  actualValvePosition: 1
  setpointTemperature: 1
  setpointValvePosition: 1
  timestamp: 1

3:
- azimuthCableWrap
- actualPositionDrive: 2
  actualVelocityDrive: 2
  timestamp: 1

8:
- cameraCableWrap
- actualPosition: 1
  actualTorquePercentage: 2
  actualVelocity: 1
  demandPosition: 1
  demandVelocity: 1
  timestamp: 1

7:
- balancing
- actualPosition: 4
  actualTorquePercentage: 4
  actualVelocity: 4
  timestamp: 1

5:
- azimuthDrives
- current: 16
  timestamp: 1

4:
- azimuthDrivesThermal
- actualTemperature: 16
  actualValvePositionAzimuth: 4
  setpointTemperature: 4
  setpointValvePositionAzimuth: 4
  timestamp: 1

14:
- elevationDrives
- current: 12
  timestamp: 1

13:
- elevationDrivesThermal
- actualTemperature10: 1
  actualTemperature12: 1
  actualTemperature2: 1
  actualTemperature4: 1
  actualTemperature6: 1
  actualTemperature8: 1
  actualValvePositionElevation: 2
  setpointTemperature: 2
  setpointValvePositionElevation: 2
  timestamp: 1

16:
- encoder
- azimuthEncoderAbsolutePosition: 4
  azimuthEncoderPosition: 4
  azimuthEncoderRelativePosition: 4
  elevationEncoderAbsolutePosition: 4
  elevationEncoderPosition: 4
  elevationEncoderRelativePosition: 4
  encoderHeadReadReferenceAZ: 4
  encoderHeadReadReferenceEL: 4
  timestamp: 1

24:
- mainCabinetThermal
- mainCabinetExternalTemperature: 1
  mainCabinetInternalTemperature: 2
  setpointTemperature: 1
  timestamp: 1

22:
- mirrorCoverLocks
- actualPosition: 4
  actualTorquePercentage: 4
  timestamp: 1

23:
- mirrorCover
- actualPosition: 4
  actualTorquePercentage: 4
  timestamp: 1

21:
- mainPowerSupply
- powerSupplyCurrent: 1
  powerSupplyVoltage: 1
  timestamp: 1

27:
- topEndChiller
- actualTemperatureAmbient: 1
  actualTemperatureArea: 5
  ambientRelativeHumiditySensor0501: 1
  ambientRelativeHumiditySensor0502: 1
  ambientRelativeHumiditySensor0504: 1
  ambientRelativeHumiditySensor0505: 1
  ambientTemperature: 1
  ambientTemperatureSensor0502: 1
  ambientTemperatureSensor0504: 1
  ambientTemperatureSensor0505: 1
  ductTemperatureSensor0506: 1
  ductTemperatureSensor0507: 1
  externalTemperatureElectricalCabinet: 4
  heatExchangerSupplyTemperature: 1
  internalTemperatureElectricalCabinet: 4
  temperatureSensor0501: 1
  threeWayValvePosition201: 1
  threeWayValvePosition202: 1
  timestamp: 1

1:
- auxiliaryCabinetsThermal
- actualTemperatureAzimuthDriveCabinet0001: 1
  actualTemperatureAzimuthPowerDistributionCabinet0001: 1
  actualTemperatureAzimuthPowerDistributionTransformer0001: 1
  actualTemperatureElevationPowerDistributionCabinet0001: 1
  actualTemperatureElevationPowerDistributionCabinet0002: 1
  setpointTemperatureAzimuthDZCabinet0001: 1
  setpointTemperatureAzimuthPowerDistributionCabinet0001: 1
  setpointTemperatureAzimuthPowerDistributionTransformer0001: 1
  setpointTemperatureElevationPowerDistributionCabinet0001: 1
  setpointTemperatureElevationPowerDistributionCabinet0002: 1
  timestamp: 1

25:
- oilSupplySystem
- ambientTemperature: 1
  computedOilFilmThicknessAzimuthBearing5004: 1
  computedOilFilmThicknessAzimuthBearing5014: 1
  computedOilFilmThicknessAzimuthBearing5024: 1
  computedOilFilmThicknessAzimuthBearing5034: 1
  computedOilFilmThicknessAzimuthBearing5044: 1
  computedOilFilmThicknessAzimuthBearing5054: 1
  computedOilFilmThicknessElevationBearing5001: 1
  computedOilFilmThicknessElevationBearing5011: 1
  computedOilFilmThicknessElevationBearing5021: 1
  computedOilFilmThicknessElevationBearing5031: 1
  oilChillerTemperature5012: 1
  oilChillerTemperature5013: 1
  oilFilmThicknessAzimuthBearing5001: 1
  oilFilmThicknessAzimuthBearing5002: 1
  oilFilmThicknessAzimuthBearing5003: 1
  oilFilmThicknessAzimuthBearing5004: 1
  oilFilmThicknessAzimuthBearing5005: 1
  oilFilmThicknessAzimuthBearing5006: 1
  oilFilmThicknessAzimuthBearing5011: 1
  oilFilmThicknessAzimuthBearing5012: 1
  oilFilmThicknessAzimuthBearing5013: 1
  oilFilmThicknessAzimuthBearing5014: 1
  oilFilmThicknessAzimuthBearing5015: 1
  oilFilmThicknessAzimuthBearing5016: 1
  oilFilmThicknessAzimuthBearing5017: 1
  oilFilmThicknessAzimuthBearing5018: 1
  oilFilmThicknessAzimuthBearing5019: 1
  oilFilmThicknessAzimuthBearing5021: 1
  oilFilmThicknessAzimuthBearing5022: 1
  oilFilmThicknessAzimuthBearing5023: 1
  oilFilmThicknessAzimuthBearing5024: 1
  oilFilmThicknessAzimuthBearing5025: 1
  oilFilmThicknessAzimuthBearing5026: 1
  oilFilmThicknessAzimuthBearing5031: 1
  oilFilmThicknessAzimuthBearing5032: 1
  oilFilmThicknessAzimuthBearing5033: 1
  oilFilmThicknessAzimuthBearing5034: 1
  oilFilmThicknessAzimuthBearing5035: 1
  oilFilmThicknessAzimuthBearing5036: 1
  oilFilmThicknessAzimuthBearing5041: 1
  oilFilmThicknessAzimuthBearing5042: 1
  oilFilmThicknessAzimuthBearing5043: 1
  oilFilmThicknessAzimuthBearing5044: 1
  oilFilmThicknessAzimuthBearing5045: 1
  oilFilmThicknessAzimuthBearing5046: 1
  oilFilmThicknessAzimuthBearing5047: 1
  oilFilmThicknessAzimuthBearing5048: 1
  oilFilmThicknessAzimuthBearing5049: 1
  oilFilmThicknessAzimuthBearing5051: 1
  oilFilmThicknessAzimuthBearing5052: 1
  oilFilmThicknessAzimuthBearing5053: 1
  oilFilmThicknessAzimuthBearing5054: 1
  oilFilmThicknessAzimuthBearing5055: 1
  oilFilmThicknessAzimuthBearing5056: 1
  oilFilmThicknessElevationBearing5001: 1
  oilFilmThicknessElevationBearing5002: 1
  oilFilmThicknessElevationBearing5003: 1
  oilFilmThicknessElevationBearing5004: 1
  oilFilmThicknessElevationBearing5005: 1
  oilFilmThicknessElevationBearing5006: 1
  oilFilmThicknessElevationBearing5011: 1
  oilFilmThicknessElevationBearing5012: 1
  oilFilmThicknessElevationBearing5013: 1
  oilFilmThicknessElevationBearing5014: 1
  oilFilmThicknessElevationBearing5015: 1
  oilFilmThicknessElevationBearing5016: 1
  oilFilmThicknessElevationBearing5021: 1
  oilFilmThicknessElevationBearing5022: 1
  oilFilmThicknessElevationBearing5023: 1
  oilFilmThicknessElevationBearing5024: 1
  oilFilmThicknessElevationBearing5025: 1
  oilFilmThicknessElevationBearing5026: 1
  oilFilmThicknessElevationBearing5031: 1
  oilFilmThicknessElevationBearing5032: 1
  oilFilmThicknessElevationBearing5033: 1
  oilFilmThicknessElevationBearing5034: 1
  oilFilmThicknessElevationBearing5035: 1
  oilFilmThicknessElevationBearing5036: 1
  oilFlowRateAzimuth5001: 1
  oilFlowRateAzimuth5002: 1
  oilFlowRateAzimuth5011: 1
  oilFlowRateAzimuth5012: 1
  oilFlowRateAzimuth5013: 1
  oilFlowRateAzimuth5021: 1
  oilFlowRateAzimuth5022: 1
  oilFlowRateAzimuth5031: 1
  oilFlowRateAzimuth5032: 1
  oilFlowRateAzimuth5041: 1
  oilFlowRateAzimuth5042: 1
  oilFlowRateAzimuth5043: 1
  oilFlowRateAzimuth5051: 1
  oilFlowRateAzimuth5052: 1
  oilFlowRateElevation5001: 1
  oilFlowRateElevation5002: 1
  oilFlowRateElevation5011: 1
  oilFlowRateElevation5012: 1
  oilFlowRateElevation5021: 1
  oilFlowRateElevation5022: 1
  oilFlowRateElevation5031: 1
  oilFlowRateElevation5032: 1
  oilLevelFacilities5001: 1
  oilPressureAzimuth5001: 1
  oilPressureAzimuth5002: 1
  oilPressureAzimuth5003: 1
  oilPressureAzimuth5011: 1
  oilPressureAzimuth5012: 1
  oilPressureAzimuth5013: 1
  oilPressureAzimuth5014: 1
  oilPressureAzimuth5015: 1
  oilPressureAzimuth5016: 1
  oilPressureAzimuth5021: 1
  oilPressureAzimuth5022: 1
  oilPressureAzimuth5023: 1
  oilPressureAzimuth5031: 1
  oilPressureAzimuth5032: 1
  oilPressureAzimuth5033: 1
  oilPressureAzimuth5034: 1
  oilPressureAzimuth5041: 1
  oilPressureAzimuth5042: 1
  oilPressureAzimuth5043: 1
  oilPressureAzimuth5044: 1
  oilPressureAzimuth5045: 1
  oilPressureAzimuth5046: 1
  oilPressureAzimuth5051: 1
  oilPressureAzimuth5052: 1
  oilPressureAzimuth5053: 1
  oilPressureAzimuth5054: 1
  oilPressureElevation5001: 1
  oilPressureElevation5002: 1
  oilPressureElevation5003: 1
  oilPressureElevation5011: 1
  oilPressureElevation5012: 1
  oilPressureElevation5013: 1
  oilPressureElevation5021: 1
  oilPressureElevation5022: 1
  oilPressureElevation5023: 1
  oilPressureElevation5031: 1
  oilPressureElevation5032: 1
  oilPressureElevation5033: 1
  oilPressureElevation5034: 1
  oilPressureFacilities5001: 1
  oilPressureFacilities5002: 1
  oilPressureFacilities5003: 1
  oilPressureFacilities5004: 1
  oilPressureFacilities5005: 1
  oilPressureFacilities5006: 1
  oilPressureFacilities5007: 1
  oilPressureFacilities5008: 1
  oilPressureFacilities5011: 1
  oilPressureFacilities5012: 1
  oilPressureFacilities5013: 1
  oilPressureFacilities5021: 1
  oilPressureFacilities5031: 1
  oilPressureFacilities5051: 1
  oilPressureFacilities5052: 1
  oilPressureFacilities5053: 1
  oilPressureFacilities5054: 1
  oilPressureFacilities5055: 1
  oilPressureFacilities5056: 1
  oilPressureFacilities5057: 1
  oilTemperatureAzimuth5001: 1
  oilTemperatureAzimuth5011: 1
  oilTemperatureAzimuth5021: 1
  oilTemperatureAzimuth5031: 1
  oilTemperatureAzimuth5041: 1
  oilTemperatureAzimuth5051: 1
  oilTemperatureElevation5001: 1
  oilTemperatureElevation5011: 1
  oilTemperatureElevation5021: 1
  oilTemperatureElevation5031: 1
  oilTemperatureElevation5101: 1
  oilTemperatureElevation5111: 1
  oilTemperatureFacilities5001: 1
  oilTemperatureFacilities5002: 1
  oilTemperatureFacilities5011: 1
  oilTemperatureFacilities5121: 1
  timestamp: 1
  valvePositionFacilities5201: 1

9:
- compressedAir
- airPressureAzimuth0001: 1
  airPressureAzimuth0002: 1
  airPressureElevation0001: 1
  airPressureElevation0002: 1
  airPressurePier0001: 1
  airTemperaturePier0001: 1
  timestamp: 1

10:
- cooling
- glycolPressureAzimuth0001: 1
  glycolPressureAzimuth0002: 1
  glycolPressureAzimuth0003: 1
  glycolPressureAzimuth0004: 1
  glycolPressureAzimuth0005: 1
  glycolPressureAzimuth0006: 1
  glycolPressureAzimuth0007: 1
  glycolPressureAzimuth0008: 1
  glycolPressureAzimuth0009: 1
  glycolPressureAzimuth0010: 1
  glycolPressureAzimuth0011: 1
  glycolPressureAzimuth0012: 1
  glycolPressureAzimuth0013: 1
  glycolPressureAzimuth0014: 1
  glycolPressureAzimuth0015: 1
  glycolPressureAzimuth0016: 1
  glycolPressureAzimuth0017: 1
  glycolPressureAzimuth0018: 1
  glycolPressureAzimuth0019: 1
  glycolPressureAzimuth0020: 1
  glycolPressureAzimuth0021: 1
  glycolPressureAzimuth0022: 1
  glycolPressureElevation0001: 1
  glycolPressureElevation0002: 1
  glycolPressurePier0101: 1
  glycolPressurePier0102: 1
  glycolTemperatureAzimuth0001: 1
  glycolTemperatureAzimuth0002: 1
  glycolTemperatureAzimuth0021: 1
  glycolTemperatureAzimuth0022: 1
  glycolTemperatureElevation0001: 1
  glycolTemperaturePier0101: 1
  glycolTemperaturePier0102: 1
  timestamp: 1

12:
- dynaleneCooling
- dynalenePressureAzimuth0001: 1
  dynalenePressureAzimuth0002: 1
  dynalenePressureElevation0001: 1
  dynalenePressureElevation0002: 1
  dynalenePressureElevation0003: 1
  dynalenePressureElevation0004: 1
  dynalenePressureElevation0005: 1
  dynalenePressureElevation0006: 1
  dynalenePressureElevation0007: 1
  dynalenePressurePier0101: 1
  dynalenePressurePier0102: 1
  dynaleneTemperatureAzimuth0001: 1
  dynaleneTemperatureAzimuth0002: 1
  dynaleneTemperaturePier0101: 1
  dynaleneTemperaturePier0102: 1
  timestamp: 1

17:
- generalPurposeGlycolWater
- glycolPressureAzimuth0001: 1
  glycolPressureAzimuth0002: 1
  glycolPressurePier0001: 1
  glycolPressurePier0002: 1
  glycolTemperaturePier0001: 1
  glycolTemperaturePier0002: 1
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
    for field_name, num_elements in field_len_dict.items():
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


#: Enum of sal_topic_name: topic_id where:
#:
#: * ``topic_id`` is the integer identifier of the low-level telemetry topic.
#: * ``sal_topic_name`` is the SAL telemetry topic name
TelemetryTopicId = enum.IntEnum(
    "TelemetryTopicId",
    {data_tuple[0]: topic_id for topic_id, data_tuple in TELEMETRY_MAP.items()},
)
