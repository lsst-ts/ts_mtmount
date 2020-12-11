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

logging.basicConfig()


async def amain():
    parser = argparse.ArgumentParser("Simulate Tekniker's TMA")
    parser.add_argument(
        "--command-port",
        type=int,
        default=MTMount.CSC_COMMAND_PORT,
        help="TCP port for commands.",
    )
    parser.add_argument(
        "--loglevel",
        type=int,
        default=logging.INFO,
        help="Log level (DEBUG=10, INFO=20, WARNING=30).",
    )
    parser.add_argument(
        "--noreconnect",
        action="store_true",
        help="Shut down when the the CSC disconnects?",
    )
    namespace = parser.parse_args()
    log = logging.getLogger("TMASimulator")
    log.setLevel(namespace.loglevel)
    print(
        f"Mock TMA controller: command_port={namespace.command_port}"
        f"; reconnect={not namespace.noreconnect}"
    )
    mock_controller = MTMount.mock.Controller(
        command_port=namespace.command_port,
        log=log,
        reconnect=not namespace.noreconnect,
    )
    try:
        print("Mock TMA controller starting")
        await mock_controller.start_task
        print("Mock TMA controller running")
        await mock_controller.done_task
    except asyncio.CancelledError:
        print("Mock TMA controller done")
    except Exception as e:
        print(f"Mock TMA controller failed: {e}")


asyncio.run(amain())
