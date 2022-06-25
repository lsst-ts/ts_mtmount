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

import asyncio

from lsst.ts import salobj

num_messages = 0


def topic_callback(data):
    global num_messages
    num_messages += 1
    if num_messages % 10 == 0:
        print(f"Read {num_messages} messages")


async def monitor_tel_azimuth() -> None:
    """A trivial method to moninor azimuth telemetry for 600s seconds."""
    async with salobj.Domain() as domain, salobj.Remote(
        domain=domain,
        name="MTMount",
        include=["azimuth"],
    ) as remote:
        print("monitor_mtmount_telemetry monitoring the azimuth topic")
        remote.tel_azimuth.callback = topic_callback
        await asyncio.sleep(10 * 60)


def monitor_mtmount_telemetry():
    """Run Monitor MTMount Telemetry."""
    asyncio.run(monitor_tel_azimuth())
    print("monitor_mtmount_telemetry done")
