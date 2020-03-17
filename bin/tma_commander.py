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
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""A simple command-line script that sends commands to the Operation Manager.

Must be connected to a low-level port on the Operation Manger.
At this time the only available port is for the hand-held device,
but Tekniker plans to make an additional port available for our CSC.

Warning: this should not be used while the CSC is running.

For more information:

tma_commander.py --help
"""

import argparse
import asyncio
import logging
import sys

from lsst.ts import salobj
from lsst.ts import MTMount

logging.basicConfig()


async def stdin_generator():
    """Thanks to http://blog.mathieu-leplatre.info
    """
    loop = asyncio.get_running_loop()
    reader = asyncio.StreamReader(loop=loop)
    reader_protocol = asyncio.StreamReaderProtocol(reader)
    await loop.connect_read_pipe(lambda: reader_protocol, sys.stdin)
    while True:
        line = await reader.readline()
        if not line:  # EOF.
            break
        yield line.decode("utf-8").strip()


def print_connected(descr, communicator):
    """Print state of a connection.

    Parameters
    ----------
    descr : `str`
        Brief description, such as "TMA commander".
    communicator : `Communicator`
        Communicator
    """

    def connected_str(connected):
        return "connected" if connected else "disconnected"

    print(
        f"{descr}: "
        f"client {connected_str(communicator.client_connected)}, "
        f"server {connected_str(communicator.server_connected)}"
    )


class Commander:
    """Communicate via TCP/IP with a Tekniker Operations Manager

    Parameters
    ----------
    host : `str`
        IP address of the operation manager.
    command_port : `int`
        IP port of the operation manager.
    reply_port : `int`
        IP port for this end (the commander).
    log_level : `int`
        Logging level.
    simulate : `bool`
        Run in simulation mode?
        If True then ``host`` is ignored.
    """

    def __init__(self, host, command_port, reply_port, log_level, simulate):
        self.log = logging.getLogger()
        self.log.setLevel(log_level)

        self.done_task = asyncio.Future()

        self.simulator = None
        if simulate:
            host = salobj.LOCAL_HOST
            self.simulator = TrivialSimulator(
                command_port=command_port, reply_port=reply_port, log=self.log
            )

        self.communicator = MTMount.Communicator(
            name="tma_commander",
            client_host=host,
            client_port=command_port,
            server_host=None,
            server_port=reply_port,
            log=self.log,
            read_replies=True,
            connect_client=True,
            connect_callback=self.connect_callback,
        )
        self.read_loop_task = asyncio.create_task(self.read_loop())
        self.command_loop_task = asyncio.create_task(self.command_loop())
        self.command_dict = dict(
            ccw_power=MTMount.commands.CameraCableWrapPower,
            ccw_stop=MTMount.commands.CameraCableWrapStop,
            ccw_move=MTMount.commands.CameraCableWrapMove,
            ccw_drive_enable=MTMount.commands.CameraCableWrapDriveEnable,
            ccw_enable_tracking=MTMount.commands.CameraCableWrapEnableTracking,
            ccw_track=MTMount.commands.CameraCableWrapTrack,
            disable=MTMount.commands.Disable,
            enable=MTMount.commands.Enable,
        )
        self.help_text = f"""Send commands to the telescope mount assemply.

TMA Commands (omit the tai_time argument, if shown):
{self.get_command_help()}

Other commands:
exit  # Exit from this commander
help  # Print this help
"""

    async def close(self):
        try:
            self.read_loop_task.cancel()
            self.command_loop_task.cancel()
            if self.simulator is not None:
                await self.simulator.close()
            await self.communicator.close()
            self.done_task.set_result(None)
        except Exception as e:
            self.done_task.set_exception(e)

    def get_command_help(self):
        help_strs = [
            f"{cmd_name} {self.get_argument_names(cmd_name)}"
            for cmd_name in self.command_dict
        ]
        return "\n".join(help_strs)

    def get_argument_names(self, cmd_name):
        CommandClass = self.command_dict[cmd_name]
        arg_infos = CommandClass.field_infos[5:]
        return " ".join(arg.name for arg in arg_infos)

    async def handle_command(self, cmd_name, args):
        """Parse arguments and issue a command.

        Parameters
        ----------
        cmd_name : `str`
            Command name.
        args : `List` [`str`]
            Arguments, as strings.

        Raises
        ------
        ValueError
            If ``args`` are not valid.
        """
        try:
            CommandClass = self.command_dict[cmd_name]
        except KeyError:
            raise ValueError(f"Unrecognized command {cmd_name}")
        arg_infos = CommandClass.field_infos[5:]
        has_tai_time_argument = isinstance(
            arg_infos[-1], MTMount.field_info.TimeFieldInfo
        )
        if has_tai_time_argument:
            arg_infos = arg_infos[0:-1]
        if len(args) != len(arg_infos):
            raise ValueError(
                f"Command {cmd_name} needs {len(arg_infos)} arguments, but got {len(args)}"
            )
        kwargs = {
            info.name: info.value_from_str(arg) for arg, info in zip(args, arg_infos)
        }
        if has_tai_time_argument:
            kwargs["tai_time"] = MTMount.get_tai_time()

        cmd = CommandClass(**kwargs)
        await self.communicator.write(cmd)

    async def read_loop(self):
        print("tma_commander waiting to connect")
        await self.communicator.connect_task
        print("tma_commander connected")
        while True:
            read_message = await self.communicator.read()
            print(f"read {read_message}")

    def connect_callback(self, communicator):
        print_connected(descr="TMA commander", communicator=communicator)
        if communicator.connected:
            asyncio.create_task(self.send_sample_replies())

    async def command_loop(self):
        try:
            print(f"Waiting to connect to the operation manager")
            await self.communicator.connect_task
            print(f"\n{self.help_text}")
            async for line in salobj.stream_as_generator(stream=sys.stdin):
                line = line.strip()
                # Strip trailing comment, if any.
                if "#" in line:
                    line = line.split("#", maxsplit=1)[0].strip()
                if not line:
                    continue
                tokens = line.split()
                cmd_name = tokens[0]
                args = tokens[1:]
                try:
                    if cmd_name == "exit":
                        break
                    elif cmd_name == "help":
                        print(self.help_text)
                    else:
                        await self.handle_command(cmd_name, args)
                except Exception as e:
                    print(f"Command {cmd_name} failed: {e}")
                    continue
        finally:
            await self.close()


class TrivialSimulator:
    """Simulate the most basic responses from the Operation Manager.

    Acknowledge all commands and mark as done.
    Also output a few other replies, to exercise the code.
    """

    def __init__(self, command_port, reply_port, log):
        self.log = log.getChild("simulator")
        self.communicator = MTMount.Communicator(
            name="trivial_simulator",
            client_host=salobj.LOCAL_HOST,
            client_port=reply_port,
            server_host=salobj.LOCAL_HOST,
            server_port=command_port,
            log=self.log,
            read_replies=False,
            connect_client=True,
            connect_callback=self.connect_callback,
        )
        self.read_loop_task = asyncio.create_task(self.read_loop())

    async def close(self):
        self.read_loop_task.cancel()
        await self.communicator.close()

    async def read_loop(self):
        print("trivial simulator: waiting for connection")
        await self.communicator.connect_task
        print("trivial simulator: connected")
        while True:
            read_command = await self.communicator.read()
            self.log.debug(f"read {read_command}")
            reply = MTMount.replies.AckReply(
                sequence_id=read_command.sequence_id, timeout_ms=100
            )
            await self.communicator.write(reply)
            reply = MTMount.replies.DoneReply(sequence_id=read_command.sequence_id)
            await self.communicator.write(reply)

    def connect_callback(self, communicator):
        print_connected(descr="Trivial simulator", communicator=communicator)
        if communicator.connected:
            asyncio.create_task(self.send_sample_replies())

    async def send_sample_replies(self):
        """Send one of each kind of reply to the TMA commander.
        """
        for reply in (
            MTMount.replies.AckReply(sequence_id=99997, timeout_ms=1234),
            MTMount.replies.NoAckReply(sequence_id=99998, explanation="Example NoAck"),
            MTMount.replies.DoneReply(sequence_id=99999),
            MTMount.replies.WarningReply(
                active=False, code=1, extra_data=["Example warning"]
            ),
            MTMount.replies.ErrorReply(
                on=False, active=False, code=2, extra_data=["Example error"]
            ),
            MTMount.replies.OnStateInfoReply(description="Example onStateInfo"),
            MTMount.replies.InPositionReply(in_position=0),
        ):
            await self.communicator.write(reply)


async def amain():
    parser = argparse.ArgumentParser(f"Send commands to Tekniker's TMA")
    parser.add_argument(
        "--host", default="192.168.0.1", help="TMA operation manager IP address."
    )
    parser.add_argument(
        "--reply-port",
        type=int,
        default=MTMount.CSC_REPLY_PORT,
        help="TCP port for replies.",
    )
    parser.add_argument(
        "--command-port",
        type=int,
        default=MTMount.CSC_COMMAND_PORT,
        help="TCP port for commands.",
    )
    parser.add_argument(
        "--log-level",
        type=int,
        default=logging.INFO,
        help="Log level (DEBUG=10, INFO=20, WARNING=30).",
    )
    parser.add_argument(
        "-s", "--simulate", action="store_true", help="Run in simuation mode?"
    )
    namespace = parser.parse_args()
    commander = Commander(
        host=namespace.host,
        reply_port=namespace.reply_port,
        command_port=namespace.command_port,
        log_level=namespace.log_level,
        simulate=namespace.simulate,
    )
    await commander.done_task


asyncio.run(amain())
