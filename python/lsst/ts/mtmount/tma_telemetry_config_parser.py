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

__all__ = ["TMATelemetryConfigParser", "run_tma_telemetry_config_parser"]

import argparse
import configparser
import dataclasses
import pathlib
import re
import sys

# Dict of TMA telemetry type (str): SAL type (str)
SAL_TYPES = dict(
    String="string",
    Int64="long long",
    INT32="int",
    DBL="double",
    Boolean="boolean",
)

# Regular expression to parse a field name as [name][optional numeric suffix]
NAME_INT_RE = re.compile(r"^(.*?)(\d*)$")

# Dict of TMA section name: SAL topic name,
# for names that the automatic translation isn't good enough.
TOPIC_NAME_TRANSLATION_DICT = {
    "AuxiliaryBoxes": "auxiliaryCabinetsThermal",
    "Cabinet0101": "cabinet0101Thermal",
    "MountControlMainCabinet": "mainCabinetThermal",
    "OSS": "oilSupplySystem",
}

# Dict of TMA unit: XML unit.
UNITS_TRANSLATION_DICT = {
    "iu": "unitless",  # encoder counts
}


@dataclasses.dataclass
class FieldInfo:
    """Information about one field of a topic.

    Abbreviated version of salobj (Kafka version) FieldInfo.
    Once we switch to Kafka we could use the salobj version.

    Parameters
    ----------
    name : `str`
        Field name.
    sal_type : `str`
        SAL data type.
    count : `int`
        For lists: the fixed list length.
    units : `str`
        Units, "unitless" if none.
    description : `str`
        Description (arbitrary text).
    """

    name: str
    sal_type: str
    count: int = 1
    units: str = "unitless"
    description: str = ""


# Dict of field name: FieldInfo
FieldsType = dict[str, FieldInfo]


@dataclasses.dataclass
class TelemetryTopicInfo:
    """Information about one SAL MTMount telemetry topic.

    Minor, but incomptatible, variant of salobj (Kafka version) TopicInfo.

    Parameters
    ----------
    topic_id : `int`
        TMA topic ID.
    sal_name : `str`
        SAL topic name, e.g. logevent_summaryState.
    fields : `dict` [`str`, `FieldInfo`]
        Dict of field name: field info.
    """

    topic_id: int
    sal_name: str
    fields: dict[str, FieldInfo]


# Dict of topic attr_name: TelemetryTopicInfo
TopicsType = dict[str, TelemetryTopicInfo]


class TMATelemetryConfigParser:
    """Create MTMount_Telemetry.xml from the TMA telemetry config file.

    Parameters
    ----------
    tma_config_path : `str` | `pathlib.Path`
        Path to TMA telemetry config file TelemetryTopicsConfiguration.ini

    Notes
    -----
    To use::

        processor = TMATelemetryConfigParser(tma_config_path=...)
        topics = TMATelemetryConfigParser.process_file()
        processor.write_xml(xml_telemetry_path=..., topics=topics)

    or call `run_tma_telemetry_config_parser` to do this from the command line.

    ** Format of the TMA telemetry configuration file **

    This file is used for feeding data to the EUI and for feeding
    telemetry data (but not events) to the CSC.

    Boolean and String items are discrete state and so are not published
    to the CSC as telemetry, but instead are sent as events (and not
    configured via this file).

    Ignore the word Array in the data type; it is for internal use.

    The following fields are specific to sending data to the CSC
    and we can set them as we like (with one noted exception):

    * ``TCP_Publish``: if ``"TRUE"`` send the data to the CSC;
      if "FALSE" do not. Note: for the few ``timestamp`` fields
      this may have to be left at "TRUE" for internal reasons.
      This parser ignores "timestamp" fields and adds the standard
      timestamp field to all telemetry topics.
    * ``TCP_PublishName``: the field name used for telemetry data sent
      to the CSC, and also for the corresponding SAL telemetry field.
    * ``Comments``: the data used for the ``Description`` section
      of each SAL telemetry fields.
    * ``Unit``: the value used for the ``Units`` section of each SAL
      telemetry field.

    Do not touch the ``url`` fields; these are for the EUI.

    Items may be re-ordered, but you must change the integer indices
    (at the end of each data type) to match the new order.
    This seems like a lot of work, so I did not rearrange any items.

    Sensor names for meters on the telescope or pier have these parts:

    * First part:

        * TMA or tMA: TMA
        * TEC: top-end chiller
        * TEA: top-end chiller assembly

    * Second part: location

        * FA: facilities
        * FS: a typo for FA
        * PI: pier
        * AZ, EL
        * AX: a typo for AZ

    * Third part: liquid or air:

        * GW: glycol water
        * CP: compressed air
        * DY: Dynalene
        * GP: general purpose glycol
        * HB: hydrostatic bearing
        * OC: oil chiller
        * OS: oil or oil supply

    * Fourth part:

        * AAA: valve state if bool, valve position (%) if double (AAA…PV)
        * CPD: valve open/closed (bool only)
        * CPM: pressure meter (bar)
        * CTM: temperature meter (deg_C)
        * CLM: oil film thickness meter (µm, except FA_OS_CLM, see below):

            * AZ/EL_HB_CLM are measured film thickness
                at hydrostatic bearings.
            * AZ/EL_OS_CLM are computed film thickness
                at hydrostatic bearings.
            * FA_OS_CLM is the amount of oil in the oil tank,
                measured as pressure in mmH2O (mm of water column).

        * CFM: flow meter (l/min)
        * MOT: pump motor on/off (bool only)

    Sensor names for cabinet sensors have these parts:

    * The first and second parts are the same as above.

    * Third part:

        * OS: oil system
        * PD: power distribution
        * CS: control system
        * DR: drive; may also be door switch?
        * DZ: typo for DR

    * Fourth part:

        * CBT: cabinet
        * TRM: electrical transformer

    Other notes:

    * There are a few explicit ``timestamp`` fields, though in most cases
      ``timestamp`` is provided without being listed. These explicit fields
      are needed by the TMA for internal use (including, at least in some
      cases, setting TCP_Publish = "TRUE"). So the config file parser ignores
      ``timestamp`` fields and adds the standard “timestamp” field to
      every topic.
    * Field described as ``Comments = "Reserved"`` are for internal use.
      Do not publish them.
    * In [LockingPins] and some other sections the original comment said
      “current %” but the TCP name said “torque” or vice-versa.
      These are all % torque.
    * HX = heat exchanger
    """

    def __init__(self, tma_config_path: pathlib.Path | str) -> None:
        tma_config_path = pathlib.Path(tma_config_path)
        if not tma_config_path.is_file:
            raise ValueError(f"No such file {tma_config_path}")
        self.config = self.create_config(tma_config_path)

    def create_config(
        self, tma_config_path: pathlib.Path | str
    ) -> configparser.ConfigParser:
        """Create a config parser that reads the specified file."""
        config = configparser.ConfigParser(interpolation=None)
        config.optionxform = str  # preserve key case
        config.read(tma_config_path)
        return config

    def make_basic_fields(
        self, section: configparser.SectionProxy
    ) -> tuple[int, FieldsType]:
        """Process one section (topic).

        Do not consolidate array-like values.

        Parameters
        ----------
        section : `configparser.SectionProxy`
            One section (telemetry topic) of the TMA config file.

        Returns
        -------
        topic_id_field_info : `tuple` [`int`, `FieldsType`]
            A tuple of:

            * topic_id: integer topic ID
            * fields: a dict of field name: FieldInfo

        Notes
        -----
        Add the ``timestamp`` field, which is not in the TMA telemetry config
        (except for internal use in a few cases; ignore that one).

        In addition to a common ``timestamp`` field, add one timestampt field
        for each individual telemetry attribute. The field name is composed
        of the attribute name appended by ``Timestamp``.
        """
        topic_sal_name = section.name[0].lower() + section.name[1:].replace(" ", "")
        topic_attr_name = f"tel_{topic_sal_name}"
        topic_id: int | None = None

        # Dict of field_name: FieldInfo
        fields = dict()
        name = None
        units = None
        description = None
        for i, (key, value) in enumerate(section.items()):
            if not value.startswith('"'):
                raise RuntimeError(
                    f'All values must start with ": key={key}; value={value!r}; '
                    f"section={section.name}"
                )
            value = value[1:-1]
            key_kind = key.rsplit(".", 1)[-1]
            if i == 0:
                if key != "TopicID":
                    raise RuntimeError(
                        "The first line of each section must be TopicID: "
                        f"key={key}; section={section.name}"
                    )
                topic_id = int(value)
            elif key in {
                "TopicFrequencyMultiple50ms",
                "Int64 Array Telemetry Data.<size(s)>",
                "INT32 Telemetry Data.<size(s)>",
                "String Array Telemetry Data.<size(s)>",
            }:
                pass
            elif key_kind == "<size(s)>":
                pass
            elif key_kind == "url":
                pass
            elif key_kind == "Unit":
                units = value
            elif key_kind == "TCP_PublishName":
                name = value
            elif key_kind == "Comments":
                description = value
            elif key_kind == "TCP_Publish":
                if None in (name, units, description):
                    raise RuntimeError(
                        f"{name=!r}, {units=!r}, and {description=!r} cannot be empty; "
                        f"key={key}; section {section.name}"
                    )
                try:
                    # Ignore psp://192.168.209.10/PXIComm_NSV/CCW timestamp
                    # which Tekniker insists have TCP_Publish = "TRUE"
                    # even though the value is for internal use
                    if value == "TRUE" and name != "timestamp":
                        tma_type = key.split(" ", 1)[0]
                        sal_type = SAL_TYPES[tma_type]
                        if sal_type == "string":
                            print(f"*** skipping string item {topic_attr_name}.{name}")
                            continue
                        if name in fields:
                            raise RuntimeError(
                                f"field name {name!r} already found in section {section.name}"
                            )

                        # WARNING: this will catch a lot of problems:
                        if units == "":
                            raise RuntimeError(
                                f"field name {name!r} needs Unit in section {section.name}"
                            )
                        if description == "":
                            raise RuntimeError(
                                f"field name {name!r} needs Comment in section {section.name}"
                            )
                        units = UNITS_TRANSLATION_DICT.get(units, units)
                        field_info = FieldInfo(
                            name=name,
                            sal_type=sal_type,
                            count=1,
                            units=units,
                            description=description,
                        )
                        fields[name] = field_info
                finally:
                    name = None
                    units = None
                    description = None
            else:
                raise RuntimeError(f"unrecognized key {key} in section {section.name}")

        if topic_id is None:
            raise RuntimeError(
                f"Bug: topic_id is None at end of section {section.name}"
            )
        if "timestamp" in fields:
            raise RuntimeError(
                f"Bug: timestamp field found and not skipped in section {section.name}"
            )
        fields["timestamp"] = FieldInfo(
            name="timestamp",
            sal_type="double",
            count=1,
            units="second",
            description="Time at which the data was measured (TAI, unix seconds).",
        )
        return (topic_id, fields)

    def consolidate_fields(self, fields: FieldsType) -> FieldsType:
        """Try to consolidate array-like fields into a new dict.

        Fields whose numeric suffixes make a sequence from 1 to N
        are consolidated into a single array topic. All other fields
        are copied from `fields` "as is", i.e. as scalars.

        Parameters
        ----------
        fields : `FieldsType`
            Information for all fields.

        Returns
        -------
        consolidated_fieids : `FieldsType`
            Consolidated field info.
        """
        consolidated_fields: FieldsType = dict()
        sequence_base_name: str | None = None
        sequence_numbers: list[int] = list()
        sequence_field_names: list[str] = list()
        for field_name in sorted(fields.keys()):
            base_name, sequence_number_str = NAME_INT_RE.match(field_name).groups()
            sequence_number = 0 if not sequence_number_str else int(sequence_number_str)

            if (
                sequence_number == 1
                and sequence_base_name is None
                and len(sequence_number_str) < 3
            ):
                # Start accumulating a potential sequence.
                sequence_base_name = base_name
                sequence_field_names = [field_name]
                sequence_numbers = [1]
            elif sequence_base_name is None:
                # Not part of a sequence and not accumulating a sequence
                consolidated_fields[field_name] = fields[field_name]
            elif base_name == sequence_base_name:
                sequence_numbers.append(sequence_number)
            else:
                # Done accumulating this potential sequence; process it.
                self._consolidate_sequence(
                    fields=fields,
                    consolidated_fields=consolidated_fields,
                    base_name=sequence_base_name,
                    sequence_numbers=sequence_numbers,
                    sequence_field_names=sequence_field_names,
                )
                sequence_base_name = None

                # Process the new field
                if sequence_number == 1:
                    # Start a new potential sequence.
                    sequence_base_name = base_name
                    sequence_field_names = [field_name]
                    sequence_numbers = [1]
                else:
                    consolidated_fields[field_name] = fields[field_name]

        # If processing a sequence, finish the job
        if sequence_base_name is not None:
            self._consolidate_sequence(
                fields=fields,
                consolidated_fields=consolidated_fields,
                base_name=sequence_base_name,
                sequence_numbers=sequence_numbers,
                sequence_field_names=sequence_field_names,
            )
            sequence_base_name = None

        # Add timestamps
        timestamp_fields = dict()
        for field_name in consolidated_fields:
            if "timestamp" in field_name:
                continue
            else:
                print(f"Adding timestamp field for {field_name}.")
                field = consolidated_fields[field_name]
                field_timestamp_name = f"{field_name}Timestamp"
                field_timestap = FieldInfo(
                    name=f"{field_name}Timestamp",
                    sal_type="double",
                    count=field.count,
                    units="second",
                    description=f"Time at which {field_name} was measured (TAI, unix seconds).",
                )
                timestamp_fields[field_timestamp_name] = field_timestap
        consolidated_fields.update(timestamp_fields)
        consolidated_fields = {
            key: value for key, value in sorted(consolidated_fields.items())
        }

        return consolidated_fields

    def _consolidate_sequence(
        self,
        fields: FieldsType,
        consolidated_fields: FieldsType,
        base_name: str,
        sequence_field_names: list[str],
        sequence_numbers: list[int],
    ) -> None:
        """Consolidate a sequence of fields, if possible.

        Parameters
        ----------
        fields : `FieldsType`
            Complete unconsolidated field info.
        consolidated_fields : `FieldsType`
            Partial consolidated field info.
            This method adds the specified sequence
            as a single new entry (if a legit sequence)
            else a set of entries, one per field name.
        base_name: `str`
            Base component of the field name; the first part
            up to the integer suffix.
        sequence_field_names : `list` [`str`]
            All field names that may be part of the sequence.
            The base names all match.
        sequence_numbers : `list` [`int`]
            The sequence number of each entry in sequence_field_names.
            The sequence is written as a single entry in consolidated_fields,
            but only if this list starts at 1 and increases by 1s.
        """
        num_members = len(sequence_numbers)
        if len(sequence_numbers) > 1 and sorted(sequence_numbers) == list(
            range(1, 1 + num_members)
        ):
            # Sequence is legit!
            print(f"{base_name} sequence {sequence_numbers} is legit")
            first_field_info = fields[sequence_field_names[0]]
            consolidated_fields[base_name] = FieldInfo(
                name=base_name,
                sal_type=first_field_info.sal_type,
                count=num_members,
                units=first_field_info.units,
                description=first_field_info.description,
            )
        else:
            # Sequence is not legit; add all candidates separately
            print(f"{base_name} sequence {sequence_numbers} is NOT legit")
            for field_name in sequence_field_names:
                consolidated_fields[field_name] = fields[field_name]

    def process_file(self) -> TopicsType:
        topics: TopicsType = dict()
        # Dict of topic TMA name: topic_id
        topic_ids = dict()
        for section in self.config.values():
            if section.name == "DEFAULT":
                continue
            topic_sal_name = TOPIC_NAME_TRANSLATION_DICT.get(section.name)
            if topic_sal_name is None:
                topic_sal_name = section.name[0].lower() + section.name[1:].replace(
                    " ", ""
                )
            topic_attr_name = f"tel_{topic_sal_name}"

            # Dict of field_name: FieldInfo
            topic_id, fields = self.make_basic_fields(section)
            topic_ids[section.name] = topic_id

            if len(fields) > 1:  # 1 for "timestamp"
                # Try to consolidate array-like scalar fields,
                # but only if their numeric suffix starts with 1
                # and increases by 1s (no missing values).
                # Process field names in alphabetical order, to more easily
                # identify sequences.
                consolidated_fields = self.consolidate_fields(fields=fields)

                print(
                    f"{topic_attr_name} has {len(consolidated_fields)} fields, "
                    f"consolidated from {len(fields)}"
                )

                topics[topic_attr_name] = TelemetryTopicInfo(
                    topic_id=topic_id,
                    sal_name=topic_sal_name,
                    fields=consolidated_fields,
                )
            else:
                print(f"{topic_attr_name} has no published fields; skipping")
        return topics

    def write_xml(
        self,
        topics: TopicsType,
        xml_telemetry_path: pathlib.Path | str,
    ) -> None:
        """Write data for MTMount_Telemetry.xml

        Parameters
        ----------
        topics : `TopicsType`
            Topics data as a dict of topic name: TopicInfo.
        xml_telemetry_path : `str` | `pathlib.Path`
            Path to output file, which should be named MTMount_Telemetry.xml.
        """
        with open(xml_telemetry_path, "w") as outfile:
            outfile.write(
                """<?xml version="1.0" encoding="UTF-8"?>
<?xml-stylesheet type="text/xsl" href="http://lsst-sal.tuc.noao.edu/schema/SALTelemetrySet.xsl"?>
<SALTelemetrySet xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://lsst-sal.tuc.noao.edu/schema/SALTelemetrySet.xsd">
  <SALTelemetry>
    <Subsystem>MTMount</Subsystem>
    <EFDB_Topic>MTMount_telemetryClientHeartbeat</EFDB_Topic>
    <Description>Heartbeat output by the MTMount CSC's telemetry client (not the TMA).</Description>
  </SALTelemetry>"""  # noqa
            )
            for topic_info in topics.values():
                outfile.write(
                    f"""
  <SALTelemetry>
    <Subsystem>MTMount</Subsystem>
    <EFDB_Topic>MTMount_{topic_info.sal_name}</EFDB_Topic>
    <Description>{topic_info.sal_name} data.</Description>"""
                )
                for field_info in topic_info.fields.values():
                    # Add terminating "." to field description, if needed.
                    field_description = field_info.description
                    if field_description and field_description[-1] not in {
                        ".",
                        "?",
                        "!",
                    }:
                        field_description += "."
                    outfile.write(
                        f"""
    <item>
      <EFDB_Name>{field_info.name}</EFDB_Name>
      <Description>{field_description}</Description>
      <IDL_Type>{field_info.sal_type}</IDL_Type>
      <Units>{field_info.units}</Units>
      <Count>{field_info.count}</Count>
    </item>"""
                    )

                outfile.write(
                    """
  </SALTelemetry>"""
                )

            outfile.write(
                """
</SALTelemetrySet>
"""
            )

    def write_telemetry_map(
        self,
        topics: TopicsType,
    ) -> None:
        """Write the data for RAW_TELEMETRY_MAP in telemetry_map.py

        Parameters
        ----------
        topics : `TopicsType`
            Topics data as a dict of topic name: TopicInfo.
        """
        outfile = sys.stdout
        print("*** BEGIN RAW_TELEMETRY_MAP DATA ***")
        for topic_info in topics.values():
            outfile.write(f"\n{topic_info.topic_id}:\n")
            outfile.write(f"- {topic_info.sal_name}\n")
            field_prefix = "- "
            for field_name, field_info in topic_info.fields.items():
                outfile.write(f"{field_prefix}{field_name}: {field_info.count}\n")
                field_prefix = "  "
        print("\n*** END RAW_TELEMETRY_MAP DATA ***")


def run_tma_telemetry_config_parser():
    """Generate MTMount_Telemetry.xml from the TMA's telemetry config file."""
    parser = argparse.ArgumentParser(
        "Convert the TMA TelemetryTopicsConfiguration.ini file to MTMount_Telemetry.xml"
    )
    parser.add_argument(
        "tma_config_path", help="path to TMA TelemetryTopicsConfiguration.ini file"
    )
    parser.add_argument(
        "--output",
        default="MTMount_Telemetry.xml",
        help="Output path for MTMount_Telemetry.xml",
    )
    args = parser.parse_args()
    processor = TMATelemetryConfigParser(args.tma_config_path)
    topics = processor.process_file()
    processor.write_xml(xml_telemetry_path=args.output, topics=topics)
    processor.write_telemetry_map(topics=topics)
