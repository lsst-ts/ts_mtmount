#!/usr/bin/env python
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
import argparse
import asyncio
import logging

from lsst.ts import MTMount


async def amain():
    parser = argparse.ArgumentParser("Simulate Tekniker's TMA")
    parser.add_argument(
        "--log-level",
        type=int,
        default=logging.INFO,
        help="Log level (DEBUG=10, INFO=20, WARNING=30).",
    )
    namespace = parser.parse_args()
    log = logging.getLogger("TMASimulator")
    log.setLevel(namespace.log_level)
    print(f"Mock TMA controller: command_port={MTMount.CSC_COMMAND_PORT}")
    mock_controller = MTMount.mock.Controller(
        command_port=MTMount.CSC_COMMAND_PORT, log=log, reconnect=True,
    )
    print("Mock TMA controller starting")
    await mock_controller.start_task
    print("Mock TMA controller running")
    await asyncio.Future()  # Wait forever.


asyncio.run(amain())
