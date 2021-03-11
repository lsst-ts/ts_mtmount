# This file is part of ts_MTMount.
#
# Developed for Vera Rubin Observatory.
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

"""A simple command-line script that sends commands to
the low-level controller (Operation Manager).

Must be connected to a low-level port on the Operation Manger.
At this time the only available port is for the hand-held device,
but Tekniker plans to make an additional port available for our CSC.

Warning: this should not be used while the CSC is running.

For more information:

tma_commander.py --help
"""
__all__ = ["TmaCommander"]

import argparse
import asyncio
import logging
import sys
import traceback

from lsst.ts import salobj
from lsst.ts import simactuators

from . import constants
from . import command_futures
from . import commands
from . import communicator
from . import mock
from . import replies

logging.basicConfig()

TRACK_INTERVAL = 0.1  # interval between tracking updates (seconds)

# How far in advance to set the time field of tracking commands (seconds)
TRACK_ADVANCE_TIME = 0.05

# Extra time to wait for commands to be done (sec)
TIMEOUT_BUFFER = 5

# Timeout for command ack
ACK_TIMEOUT = 5


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
        yield line.decode(errors="ignore").strip()


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


class TmaCommander:
    """Command a Tekniker Operation Manager.

    Read commands from a user on the command line
    and send them to the low-level controller (Operation Manager).

    Warning: this is only intended as a short-term hack.
    It will be deleted once MTMountCsc has been shown to work
    with the Operation Manager.

    Parameters
    ----------
    host : `str`
        IP address of the operation manager.
    log_level : `int`
        Logging level.
    simulate : `bool`
        Run in simulation mode?
        If True then ``host`` is ignored.
    """

    def __init__(self, host, log_level, simulate):
        self.log = logging.getLogger()
        self.log.setLevel(log_level)

        self.done_task = asyncio.Future()
        self.command_loop_task = salobj.make_done_future()
        self.read_loop_task = salobj.make_done_future()
        self.tracking_task = salobj.make_done_future()

        # Dict of command_id: CommandFuture
        self.command_futures_dict = dict()

        self.simulator = None
        if simulate:
            host = salobj.LOCAL_HOST
            self.simulator = mock.Controller(
                command_port=constants.CSC_COMMAND_PORT, log=self.log
            )

        self.communicator = communicator.Communicator(
            name="tma_commander",
            client_host=host,
            client_port=constants.CSC_COMMAND_PORT,
            server_host=None,
            # Tekniker uses repy port = command port + 1
            server_port=constants.CSC_COMMAND_PORT + 1,
            log=self.log,
            read_replies=True,
            connect=True,
            connect_callback=self.connect_callback,
        )
        self.command_dict = dict(
            ask_for_command=commands.AskForCommand,
            az_drive_enable=commands.AzimuthAxisDriveEnable,
            az_drive_reset=commands.AzimuthAxisDriveReset,
            az_enable_tracking=commands.AzimuthAxisEnableTracking,
            az_home=commands.AzimuthAxisHome,
            az_move=commands.AzimuthAxisMove,
            az_power=commands.AzimuthAxisPower,
            az_reset_alarm=commands.AzimuthAxisResetAlarm,
            az_stop=commands.AzimuthAxisStop,
            az_track=commands.AzimuthAxisTrack,
            ccw_drive_enable=commands.CameraCableWrapDriveEnable,
            ccw_drive_reset=commands.CameraCableWrapDriveReset,
            ccw_enable_tracking=commands.CameraCableWrapEnableTracking,
            ccw_move=commands.CameraCableWrapMove,
            ccw_power=commands.CameraCableWrapPower,
            ccw_reset_alarm=commands.CameraCableWrapResetAlarm,
            ccw_stop=commands.CameraCableWrapStop,
            ccw_track=commands.CameraCableWrapTrack,
            el_drive_enable=commands.ElevationAxisDriveEnable,
            el_drive_reset=commands.ElevationAxisDriveReset,
            el_enable_tracking=commands.ElevationAxisEnableTracking,
            el_home=commands.ElevationAxisHome,
            el_move=commands.ElevationAxisMove,
            el_power=commands.ElevationAxisPower,
            el_reset_alarm=commands.ElevationAxisResetAlarm,
            el_stop=commands.ElevationAxisStop,
            el_track=commands.ElevationAxisTrack,
            mc_deploy=commands.MirrorCoversDeploy,
            mc_power=commands.MirrorCoversPower,
            mc_reset_alarm=commands.MirrorCoversResetAlarm,
            mc_retract=commands.MirrorCoversRetract,
            mcl_move_all=commands.MirrorCoverLocksMoveAll,
            mcl_power=commands.MirrorCoverLocksPower,
            mcl_reset_alarm=commands.MirrorCoverLocksResetAlarm,
            mps_power=commands.MainPowerSupplyPower,
            mps_reset_alarm=commands.MainPowerSupplyResetAlarm,
            oss_power=commands.OilSupplySystemPower,
            oss_reset_alarm=commands.OilSupplySystemResetAlarm,
            safety_reset=commands.SafetyReset,
            tec_power=commands.TopEndChillerPower,
            tec_reset_alarm=commands.TopEndChillerResetAlarm,
        )
        self.help_text = f"""Send commands to the telescope mount assemply.

TMA Commands (omit the tai argument, if shown):
{self.get_command_help()}

Other commands:
ccw_ramp start_position end_position speed  # make CCW track a ramp
ccw_cosine center_position amplitude max_speed # make CCW track one cycle of a cosine wave
stop  # stop ccw_ramp or ccw_cosine
exit  # Exit from this commander
help  # Print this help

Before commanding the TMA you must take control with:
ask_for_command 3
"""
        self.start_task = asyncio.create_task(self.start())

    @classmethod
    async def amain(cls):
        """Parse command-line arguments and run the TMA commander.
        """
        parser = argparse.ArgumentParser("Send commands to Tekniker's TMA")
        parser.add_argument(
            "--host", default="127.0.0.1", help="TMA operation manager IP address."
        )
        parser.add_argument(
            "--loglevel",
            type=int,
            default=logging.INFO,
            help="Log level (DEBUG=10, INFO=20, WARNING=30).",
        )
        parser.add_argument(
            "-s", "--simulate", action="store_true", help="Run in simuation mode?"
        )
        namespace = parser.parse_args()
        commander = cls(
            host=namespace.host,
            log_level=namespace.loglevel,
            simulate=namespace.simulate,
        )
        await commander.start_task
        await commander.done_task

    async def close(self):
        """Shut down this TMA commander."""
        try:
            print("Closing")
            await self.stop_tracking(verbose=False)
            self.read_loop_task.cancel()
            self.command_loop_task.cancel()
            while self.command_futures_dict:
                cmd_futures = self.command_futures_dict.popitem()[1]
                cmd_futures.setdone()
            if self.simulator is not None:
                await self.simulator.close()
            await self.communicator.close()
            self.done_task.set_result(None)
            print("Done")
        except Exception as e:
            self.done_task.set_exception(e)

    async def command_loop(self):
        """Read commands from the user and send them to the operations manager.
        """
        try:
            print("Waiting to connect to the operation manager")
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
                    elif cmd_name == "ccw_ramp":
                        await self.start_ccw_ramp(args)
                    elif cmd_name == "ccw_cosine":
                        await self.start_ccw_cosine(args)
                    elif cmd_name == "stop":
                        await self.stop_tracking(verbose=True)
                    else:
                        await self.handle_command(cmd_name, args)
                except Exception as e:
                    print(f"Command {cmd_name} failed: {e!r}")
                    print(traceback.format_exc())
                    continue
        finally:
            await self.close()

    def connect_callback(self, communicator):
        """Callback function for changes in communicator connection state.

        Parameters
        ----------
        communicator : `MTMount.Communicator`
            The communicator whose connection state has changed.
        """
        print_connected(descr="TMA commander", communicator=communicator)
        if communicator.connected:
            self.read_loop_task = asyncio.create_task(self.read_loop())

    def get_argument_names(self, cmd_name):
        """Get the argument names from a command, for printing help.

        Parameters
        ----------
        cmd_name : `str`
            The command name, as the key in self.command_dict.
        """
        # Get FieldInfos for the command arguments (if any) from the
        # command class, and use that to generate a list of argument names.
        CommandClass = self.command_dict[cmd_name]
        arg_infos = CommandClass.field_infos[commands.NUM_HEADER_FIELDS :]
        return " ".join(arg_info.name for arg_info in arg_infos)

    def get_command_help(self):
        """Get help for all commands in self.command_dict, as a single string.
        """
        help_strs = [
            f"{cmd_name} {self.get_argument_names(cmd_name)}"
            for cmd_name in self.command_dict
        ]
        return "\n".join(help_strs)

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
        # Get FieldInfos for the command arguments (if any) from the
        # command class, and use them to cast and validate the arguments.
        arg_infos = CommandClass.field_infos[commands.NUM_HEADER_FIELDS :]
        # If the final argument for the command is TAI time,
        # then set it to the current time.
        has_tai_argument = arg_infos and arg_infos[-1].name == "tai"
        if has_tai_argument:
            arg_infos = arg_infos[0:-1]
        if len(args) != len(arg_infos):
            raise ValueError(
                f"Command {cmd_name} needs {len(arg_infos)} arguments, but got {len(args)}"
            )
        kwargs = {
            info.name: info.value_from_str(arg) for arg, info in zip(args, arg_infos)
        }
        if has_tai_argument:
            kwargs["tai"] = salobj.current_tai()

        cmd = CommandClass(**kwargs)
        await self.write_command(cmd)

    async def read_loop(self):
        """Read replies from the operations manager.
        """
        try:
            await self.communicator.connect_task
            while True:
                reply = await self.communicator.read()
                print(f"Read {reply}")

                # Handle command ack, if relevant (only commands issued
                # with do_wait=True are put in the command dict).
                if isinstance(reply, replies.AckReply):
                    # Command acknowledged. Set timeout but leave
                    # cmd_futures in command_futures_dict.
                    cmd_futures = self.command_futures_dict.get(reply.sequence_id, None)
                    if cmd_futures is not None:
                        cmd_futures.setack(reply.timeout_ms / 1000)
                elif isinstance(reply, replies.NoAckReply):
                    # Command failed. Pop the command_futures_dict entry
                    # and report failure.
                    cmd_futures = self.command_futures_dict.pop(reply.sequence_id, None)
                    if cmd_futures is not None:
                        cmd_futures.setnoack(reply.explanation)
                elif isinstance(reply, replies.DoneReply):
                    cmd_futures = self.command_futures_dict.pop(reply.sequence_id, None)
                    if cmd_futures is not None:
                        cmd_futures.setdone()

        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"tma_commander read loop failed with {e!r}")

    async def start(self):
        await self.communicator.start_task
        self.command_loop_task = asyncio.create_task(self.command_loop())

    async def start_ccw_ramp(self, args):
        """Start the camera cable wrap tracking a linear ramp.
        """
        self.tracking_task.cancel()
        arg_names = ("start_position", "end_position", "speed")
        if len(args) != len(arg_names):
            arg_names_str = ", ".join(arg_names)
            raise ValueError(
                f"need {len(arg_names)} arguments: {arg_names_str}; got {args}"
            )
        args = [float(arg) for arg in args]
        kwargs = {name: arg for name, arg in zip(arg_names, args)}
        self.tracking_task = asyncio.ensure_future(self._ccw_ramp(**kwargs))

    async def start_ccw_cosine(self, args):
        """Start the camera cable wrap tracking one cycle of a cosine wave.
        """
        self.tracking_task.cancel()
        arg_names = ("center_position", "amplitude", "max_speed")
        if len(args) != len(arg_names):
            arg_names_str = ", ".join(arg_names)
            raise ValueError(
                f"need {len(arg_names)} arguments: {arg_names_str}; got {args}"
            )
        args = [float(arg) for arg in args]
        kwargs = {name: arg for name, arg in zip(arg_names, args)}
        self.tracking_task = asyncio.ensure_future(self._ccw_cosine(**kwargs))

    async def stop_tracking(self, verbose):
        if not self.tracking_task.done():
            print("Stop tracking")
            self.tracking_task.cancel()
            # Give time for the axis to be stopped
            await asyncio.sleep(0.5)
        elif verbose:
            print("Tracking not running; nothing done")

    async def write_command(self, command, do_wait=False):
        """Issue a command.

        If do_wait true then wait for the command to finish.
        """
        print(f"Write {command}")
        if command.sequence_id in self.command_futures_dict:
            raise RuntimeError(
                f"Bug! Duplicate sequence_id {command.sequence_id} in command_futures_dict"
            )
        await self.communicator.write(command)
        if not do_wait:
            return

        cmd_futures = command_futures.CommandFutures()
        self.command_futures_dict[command.sequence_id] = cmd_futures
        await asyncio.wait_for(cmd_futures.ack, ACK_TIMEOUT)
        if command.command_code in commands.AckOnlyCommandCodes:
            # This command only receives an Ack; mark it done.
            cmd_futures.done.set_result(None)
        else:
            try:
                await asyncio.wait_for(
                    cmd_futures.done, timeout=cmd_futures.timeout + TIMEOUT_BUFFER
                )
            except asyncio.TimeoutError:
                self.command_futures_dict.pop(command.sequence_id, None)
                raise asyncio.TimeoutError(
                    f"Timed out after {cmd_futures.timeout + TIMEOUT_BUFFER} seconds "
                    f"waiting for the Done reply to {command}"
                )

    async def _ccw_ramp(self, start_position, end_position, speed):
        """Make the camera cable wrap track a linear ramp.

        Parameters
        ----------
        start_position : `float`
            Starting position of ramp (deg).
        end_position : `float`
            Ending position of ramp (deg).
        speed : `float`
            Speed of motion along the ramp (deg/sec).
            The sign is ignored.
        """
        try:
            ramp_generator = simactuators.RampGenerator(
                start_positions=[start_position],
                end_positions=[end_position],
                speeds=[speed],
                advance_time=TRACK_ADVANCE_TIME,
            )
            print(
                f"Tracking a ramp from {start_position} to {end_position} at speed {speed}; "
                f"this will take {ramp_generator.duration:0.2f} seconds"
            )

            await self.write_command(
                commands.CameraCableWrapEnableTracking(on=True), do_wait=True
            )
            for positions, velocities, tai in ramp_generator():
                track_command = commands.CameraCableWrapTrack(
                    position=positions[0], velocity=velocities[0], tai=tai,
                )
                await self.write_command(track_command, do_wait=True)
                await asyncio.sleep(TRACK_INTERVAL)
        except asyncio.CancelledError:
            print("ccw_ramp cancelled")
        except Exception as e:
            print(f"ccw_ramp failed: {e!r}")
        finally:
            # Wait a bit in case there is a tracking command to be acked.
            await asyncio.sleep(0.1)
            await self.write_command(commands.CameraCableWrapStop())

    async def _ccw_cosine(self, center_position, amplitude, max_speed):
        """Make the camera cable wrap track one cycle of a cosine wave.

        The range of motion is center_position +/- amplitude,
        plus whatever motion is required to slew to the path.

        Parameters
        ----------
        center_position : `float`
            Midpoint of cosine wave (deg).
        amplitude : `float`
            Amplitude of cosine wave (deg).
        max_speed : `float`
            Maximum speed along the path (deg/sec).
        """
        try:
            cosine_generator = simactuators.CosineGenerator(
                center_positions=[center_position],
                amplitudes=[amplitude],
                max_speeds=[max_speed],
                advance_time=TRACK_ADVANCE_TIME,
            )
            print(
                f"Tracking one cycle of a cosine wave centered at {center_position} "
                f"with amplitude {amplitude}; "
                f"this will take {cosine_generator.duration:0.2f} seconds"
            )
            await self.write_command(
                commands.CameraCableWrapEnableTracking(on=True), do_wait=True
            )
            for positions, velocities, tai in cosine_generator():
                track_command = commands.CameraCableWrapTrack(
                    position=positions[0], velocity=velocities[0], tai=tai,
                )
                await self.write_command(track_command, do_wait=True)
                await asyncio.sleep(TRACK_INTERVAL)
        except asyncio.CancelledError:
            print("ccw_cosine cancelled")
        except Exception as e:
            print(f"ccw_cosine failed: {e!r}")
        finally:
            # Wait a bit in case there is a tracking command to be acked.
            await asyncio.sleep(0.1)
            await self.write_command(commands.CameraCableWrapStop())
