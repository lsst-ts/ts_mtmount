#!/usr/bin/env python
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

import argparse
import asyncio
import functools

from lsst.ts import salobj

num_messages_by_topic_name = dict()


def topic_callback(data, topic_name, skip):
    """Handle data for a telemetry topic.

    Intended to be used as a salobj.topics.ReadTopic callback function
    (specifying topic_name and skip using functools.partial).

    Parameters
    ----------
    topic_name : `str`
        Telemetry topic name, e.g. "azimuth".
    skip : `int`
        How many messages to skip.
    """
    global num_messages_by_topic_name
    num_messages = num_messages_by_topic_name.get(topic_name, 0)
    num_messages += 1
    num_messages_by_topic_name[topic_name] = num_messages
    if num_messages % (skip + 1) == 0:
        dt = data.private_sndStamp - data.timestamp
        print(f"Read {num_messages} {topic_name} messages; {dt=:0.3f} sec; {data=}")


async def _monitor_mtmount_telemetry(topic_names, duration, skip):
    """Monitor specified MTMount Telemetry topics.

    Parameters
    ----------
    topic_names : `list` [`str`]
        Telemetry topic names, e.g. "azimuth".
    duration : `float`
        How long to monitor?
    skip : `int`
        How many messages to skip (for each topic).
    """
    print(f"monitor topics {topic_names} for {duration} seconds")

    async with salobj.Domain() as domain, salobj.Remote(
        domain=domain, name="MTMount", include=topic_names
    ) as remote:
        for topic_name in topic_names:
            topic = getattr(remote, f"tel_{topic_name}")
            topic.callback = functools.partial(
                topic_callback, topic_name=topic_name, skip=skip
            )
        await asyncio.sleep(duration)


def monitor_mtmount_telemetry():
    """Monitor a specified set of topics for a specified duration."""
    parser = argparse.ArgumentParser("Monitor a few telemetry topics")
    parser.add_argument(
        "topics",
        nargs="*",
        default=["azimuth", "cameraCableWrap"],
        help="Which topics to monitor, e.g. 'azimuth'",
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        help="How long to monitor (seconds)?",
        default=10,
    )
    parser.add_argument(
        "-s",
        "--skip",
        type=int,
        help="How many messages to skip? Must be >= 0",
        default=9,
    )
    args = parser.parse_args()
    assert args.skip >= 0
    asyncio.run(
        _monitor_mtmount_telemetry(
            topic_names=args.topics, duration=args.duration, skip=args.skip
        )
    )
