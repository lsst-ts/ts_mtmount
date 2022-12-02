# This file is part of ts_mtmount.
#
# Developed for the LSST Telescope and Site Systems.
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
import contextlib
import itertools
import json
import logging
import time
import types
import unittest
import unittest.mock
import warnings

import numpy.testing
import pytest
from lsst.ts import mtmount, salobj, utils
from lsst.ts.idl.enums.MTMount import (
    AxisMotionState,
    DeployableMotionState,
    ElevationLockingPinMotionState,
    PowerState,
    System,
)

START_TIMEOUT = 20  # Time for startup (sec)
STD_TIMEOUT = 2  # Timeout for short operations (sec)
# Padding for the time limit returned by device do_methods
TIMEOUT_PADDING = 5

logging.basicConfig()


UNSUPPORTED_COMMAND_CODE = (
    mtmount.enums.CommandCode.TRANSFER_FUNCTION_AZIMUTH_EXCITATION
)


class UnsupportedCommand(mtmount.commands.Command):
    field_infos = mtmount.commands.make_command_field_infos(UNSUPPORTED_COMMAND_CODE)


class MockControllerTestCase(unittest.IsolatedAsyncioTestCase):
    @contextlib.asynccontextmanager
    async def make_controller(self, commander=mtmount.Source.CSC):
        """Make a mock controller as self.controller.

        Parameters
        ----------if name in ("actualPosition
        commander : `Source`, optional
            Who initially has command. Defaults to `Source.CSC`,
            so tests need not issue the ``ASK_FOR_COMMAND`` command
            before issuing other commands.

        Other special values:

        * `Source.NONE`: this is now the real system starts up.
        * `Source.HHD`: the ``ASK_FOR_COMMAND`` command is rejected
          from any other source. This reflects the real system, because
          nobody can take command from the handheld device. This offers
          a convenient way to test ``ASK_FOR_COMMAND`` failures.
        """
        log = logging.getLogger()
        self.controller = mtmount.mock.Controller(
            log=log, commander=commander, random_ports=True
        )
        t0 = time.monotonic()
        await asyncio.wait_for(self.controller.start_task, timeout=START_TIMEOUT)

        # Dict of task_id: parsed data
        self.telemetry_dict = {topic_id: None for topic_id in mtmount.TelemetryTopicId}

        # Connect to the command port
        connect_coro = asyncio.open_connection(
            host=salobj.LOCAL_HOST, port=self.controller.command_server.port
        )
        self.command_reader, self.command_writer = await asyncio.wait_for(
            connect_coro, timeout=STD_TIMEOUT
        )

        # Connect to the telemetry port
        telemetry_connect_coro = asyncio.open_connection(
            host=salobj.LOCAL_HOST, port=self.controller.telemetry_server.port
        )
        self.telemetry_reader, self.telemetry_writer = await asyncio.wait_for(
            telemetry_connect_coro, timeout=STD_TIMEOUT
        )
        self.telemetry_task = asyncio.create_task(self.telemetry_read_loop())
        dt = time.monotonic() - t0

        print(f"Time to start up: {dt:0.2f} sec")
        try:
            yield
        finally:
            self.telemetry_writer.close()
            try:
                await asyncio.wait_for(self.telemetry_writer.wait_closed(), timeout=1)
            except asyncio.TimeoutError:
                print("Timed out waiting for the telemetry writer to close; continuing")
            self.command_writer.close()
            try:
                await asyncio.wait_for(self.command_writer.wait_closed(), timeout=1)
            except asyncio.TimeoutError:
                print("Timed out waiting for the command writer to close; continuing")
            await self.controller.close()

    async def flush_replies(self):
        """Flush all replies."""
        try:
            for i in range(2):
                await asyncio.wait_for(self.command_reader.read(), timeout=0.01)
            self.fail("flush read did not time out")
        except asyncio.TimeoutError:
            return

    async def read_one_reply(self, timeout=STD_TIMEOUT):
        """Read, parse, and return one reply from the mock controller.

        Return it as a types.SimpleNamespace with the parameters
        hoisted to the top level.
        """
        read_bytes = await asyncio.wait_for(
            self.command_reader.readuntil(mtmount.LINE_TERMINATOR), timeout=timeout
        )
        reply_dict = json.loads(read_bytes)
        return types.SimpleNamespace(
            id=mtmount.ReplyId(reply_dict["id"]),
            timestamp=reply_dict["timestamp"],
            **reply_dict["parameters"],
        )

    async def read_replies(
        self, wait_reply_codes, other_reply_codes=(), return_all_replies=False
    ):
        """Wait for one or more replies of the specified types.

        Return a list of the all replies read.

        Parameters
        ----------

        wait_reply_codes : `list` [`mtmount.Reply`]
            Reply codes of messages to wait for, one reply per entry,
            but not necessarily in order.
            If a reply code is listed N times then wait for N such replies.
            Extra replies with these reply codes are ignored,
            unless the reply code also appears in ``other_reply_codes``.
        other_reply_codes : `list` [`mtmount.Reply`], optional
            Reply codes of other messages to return, if seen.
            These are not "used up" they simply acts as a filter.
            Duplicates are silently ignored.
        return_all_replies : `bool`, optional
            Return all replies?
            If True then other_reply_codes must be empty.
        timeout : `float`, optional
            Timeout waiting for the Done reply (second).
            Ignored for other reads.

        Returns
        -------
        replies : `List` [`mtmount.Reply`]
            The read replies.
        """
        nonack_reply_codes_remaining = wait_reply_codes.copy()
        other_reply_codes_set = set(other_reply_codes)
        replies = []
        while nonack_reply_codes_remaining:
            reply = await self.read_one_reply()
            if reply.id in nonack_reply_codes_remaining:
                nonack_reply_codes_remaining.remove(reply.id)
                replies.append(reply)
            elif return_all_replies or reply.id in other_reply_codes_set:
                replies.append(reply)
        return replies

    async def run_command(
        self,
        command,
        use_read_loop,
        flush=True,
        final_reply_code=mtmount.ReplyId.CMD_SUCCEEDED,
        wait_reply_codes=(),
        other_reply_codes=(),
        return_all_replies=False,
        timeout=STD_TIMEOUT,
    ):
        """Run one command and check the replies.

        Return a list of non-Ack/Done/NoAck replies.

        Parameters
        ----------
        command : `mtmount.Command`
            Command to send.
        use_read_loop : `bool`
            If True then send the command via TCP/IP.
            If False then call self.controller.handle_command directly.
        flush : `bool`, optional
            If True then flush replies before issuing the command.
        final_reply_code : `mtmount.ReplyId`, optional
            The expected final CMD_x reply code.
            Defaults to CMD_SUCCEEDED.
        wait_reply_codes : `list` [`mtmount.Reply`], optional
            non-CMD_x reply codes of messages to wait for, one reply per entry,
            but not necessarily in order.
            If a reply code is listed N times then wait for N such replies.
            Extra replies with these reply codes are ignored,
            unless the reply code also appears in ``other_reply_codes``.
        other_reply_codes : `list` [`mtmount.Reply`], optional
            Non-CMD_x reply codes of other messages to return, if seen,
            in addition to those in ``wait_reply_codes``.
            Unlike wait_reply_codes, each code need only be listed once
            in order to get all replies with that code.
            Duplicates are accepted, but generate a warning.
        return_all_replies : `bool`, optional
            Return all replies?
            If True then other_reply_codes must be empty.
        timeout : `float`
            Timeout waiting for the Done reply (second).
            Ignored for other reads.

        Returns
        -------
        non_cmd_replies : `List` [`mtmount.Reply`]
            All non-CMD_x replies read. This list will definitely contain
            the types listed in ``wait_reply_codes`` and may contain
            other replies as well.
        """
        if return_all_replies and other_reply_codes:
            raise ValueError("return_all_replies true, other_reply_codes must be empty")

        cmd_reply_codes = frozenset(
            (
                mtmount.ReplyId.CMD_ACKNOWLEDGED,
                mtmount.ReplyId.CMD_REJECTED,
                mtmount.ReplyId.CMD_SUCCEEDED,
                mtmount.ReplyId.CMD_FAILED,
                mtmount.ReplyId.CMD_SUPERSEDED,
            )
        )
        if final_reply_code not in cmd_reply_codes:
            raise ValueError(
                f"final_reply_code={final_reply_code} not a command reply code"
            )

        bad_nonack_codes = set(wait_reply_codes) & cmd_reply_codes
        if bad_nonack_codes:
            raise ValueError(
                f"wait_reply_codes={wait_reply_codes} contains "
                f"{bad_nonack_codes}, which are cmd replies"
            )

        other_reply_codes_set = set(other_reply_codes)
        if len(other_reply_codes_set) < len(other_reply_codes):
            warnings.warn(
                "There is no need to specify duplicate codes "
                f"in other_reply_codes={other_reply_codes}"
            )

        bad_nonack_codes = other_reply_codes_set & cmd_reply_codes
        if bad_nonack_codes:
            raise ValueError(
                f"other_reply_codes={other_reply_codes} contains "
                f"{bad_nonack_codes}, which are cmd replies"
            )

        nonack_reply_codes_remaining = list(wait_reply_codes)

        if flush:
            await self.flush_replies()
        if use_read_loop:
            self.command_writer.write(command.encode())
            await self.command_writer.drain()
        else:
            await self.controller.handle_command(command)
        non_cmd_replies = list()
        while True:
            reply = await asyncio.wait_for(self.read_one_reply(), timeout=STD_TIMEOUT)
            if reply.id in cmd_reply_codes:
                break
            elif reply.id in nonack_reply_codes_remaining:
                nonack_reply_codes_remaining.remove(reply.id)
                non_cmd_replies.append(reply)
            elif return_all_replies or reply.id in other_reply_codes_set:
                non_cmd_replies.append(reply)
        if final_reply_code == mtmount.ReplyId.CMD_REJECTED:
            # Should fail before ack
            assert reply.id == mtmount.ReplyId.CMD_REJECTED
            assert reply.sequenceId == command.sequence_id
        else:
            explanation = getattr(reply, "explanation", "")
            assert reply.id == mtmount.ReplyId.CMD_ACKNOWLEDGED, explanation
            assert reply.sequenceId == command.sequence_id
            if final_reply_code == mtmount.ReplyId.CMD_ACKNOWLEDGED:
                # This command is done when acknowledged
                return non_cmd_replies

            while True:
                reply = await asyncio.wait_for(self.read_one_reply(), timeout=timeout)
                if reply.id in cmd_reply_codes:
                    break
                elif reply.id in nonack_reply_codes_remaining:
                    nonack_reply_codes_remaining.remove(reply.id)
                    non_cmd_replies.append(reply)
                elif return_all_replies or reply.id in other_reply_codes_set:
                    non_cmd_replies.append(reply)
            explanation = getattr(reply, "explanation", "")
            assert reply.id == final_reply_code, explanation
            assert reply.sequenceId == command.sequence_id
        if nonack_reply_codes_remaining:
            non_cmd_replies += await asyncio.wait_for(
                self.read_replies(
                    wait_reply_codes=nonack_reply_codes_remaining,
                    other_reply_codes=other_reply_codes,
                    return_all_replies=return_all_replies,
                ),
                timeout=timeout,
            )
        if return_all_replies or other_reply_codes_set:
            assert len(non_cmd_replies) >= len(wait_reply_codes)
        else:
            assert len(non_cmd_replies) == len(wait_reply_codes)
        return non_cmd_replies

    async def telemetry_read_loop(self):
        """Read telemetry and add it to ``self.telemetry_dict``."""
        while True:
            try:
                data = await self.telemetry_reader.readuntil(mtmount.LINE_TERMINATOR)
                decoded_data = data.decode()
                llv_data = json.loads(decoded_data)
                topic_id = llv_data.get("topicID")
                self.telemetry_dict[topic_id] = llv_data
            except asyncio.CancelledError:
                return
            except (ConnectionResetError, asyncio.IncompleteReadError):
                return
            except Exception as e:
                self.fail(f"telemetry_read_loop failed: {e!r}")
                return

    async def test_ask_for_command_ok(self):
        async with self.make_controller(commander=mtmount.Source.NONE):
            # Until AskForCommand is issued, all other commands should fail;
            # try a sampling of commands.
            sample_commands = (
                mtmount.commands.CameraCableWrapPower(on=True),
                mtmount.commands.MainAxesPowerSupplyPower(on=True),
                mtmount.commands.MirrorCoverLocksPower(drive=-1, on=True),
            )
            for command in sample_commands:
                await self.run_command(
                    command=command,
                    final_reply_code=mtmount.ReplyId.CMD_REJECTED,
                    use_read_loop=True,
                )
            non_cmd_replies = await self.run_command(
                command=mtmount.commands.AskForCommand(commander=mtmount.Source.CSC),
                wait_reply_codes=[mtmount.ReplyId.COMMANDER],
                use_read_loop=True,
            )
            assert len(non_cmd_replies) == 1
            reply = non_cmd_replies[0]
            assert reply.id == mtmount.ReplyId.COMMANDER
            assert reply.actualCommander == mtmount.Source.CSC
            for command in sample_commands:
                await self.run_command(
                    command=command,
                    use_read_loop=True,
                )

    async def test_ask_for_command_rejected(self):
        async with self.make_controller(commander=mtmount.Source.HHD):
            # commander=HHD prevents assigning command to any other commander.
            sample_commands = (
                mtmount.commands.AskForCommand(commander=mtmount.Source.NONE),
                mtmount.commands.AskForCommand(commander=mtmount.Source.CSC),
                mtmount.commands.AskForCommand(commander=mtmount.Source.EUI),
                mtmount.commands.AskForCommand(),  # defaults to CSC
                mtmount.commands.MirrorCoverLocksPower(drive=-1, on=True),
                mtmount.commands.AzimuthPower(on=True),
                mtmount.commands.ElevationPower(on=True),
            )
            for command in sample_commands:
                await self.run_command(
                    command=command,
                    final_reply_code=mtmount.ReplyId.CMD_REJECTED,
                    use_read_loop=True,
                )
            # Asking for command by the HHD should work, though it is a no-op.
            await self.run_command(
                command=mtmount.commands.AskForCommand(commander=mtmount.Source.HHD),
                use_read_loop=True,
            )

    async def test_both_axes_commands(self):
        """Test both_axes_x commands, but test motion elsewhere."""
        async with self.make_controller():
            azimuth_device = self.controller.device_dict[System.AZIMUTH]
            elevation_device = self.controller.device_dict[System.ELEVATION]
            devices = (azimuth_device, elevation_device)
            for device in devices:
                assert not device.alarm_on
                assert not device.power_on
                assert not device.enabled
                assert not device.homed
                assert not device.tracking_enabled

            # Most commands are rejected if the main axes power supply
            # is powered off. Note:
            # * BothAxesEnableTracking is also rejected
            #   because the axes have not been homed
            # * BothAxesTrackTarget is also rejected
            #   because tracking has not been enabled
            # * All motion-related commands are also rejected
            #   because the axes are not powered on.
            for command in (
                mtmount.commands.BothAxesEnableTracking(),
                mtmount.commands.BothAxesHome(),
                mtmount.commands.BothAxesMove(azimuth=45, elevation=45),
                mtmount.commands.BothAxesPower(on=True),
                mtmount.commands.BothAxesStop(),
                mtmount.commands.BothAxesTrackTarget(
                    azimuth=45,
                    elevation=45,
                    azimuth_velocity=0,
                    elevation_velocity=0,
                    tai=utils.current_tai(),
                ),
            ):
                await self.run_command(
                    command=command,
                    use_read_loop=True,
                    final_reply_code=mtmount.ReplyId.CMD_REJECTED,
                )

            # BothAxesResetAlarm has no effect if the main axes power supply
            # is powered off (unfortunately; I wish it was rejected)
            for device in devices:
                device.alarm_on = True
            await self.run_command(
                command=mtmount.commands.BothAxesResetAlarm(), use_read_loop=True
            )
            for device in devices:
                assert device.alarm_on

            # Turn on main power
            await self.run_command(
                command=mtmount.commands.MainAxesPowerSupplyPower(on=True),
                use_read_loop=True,
            )
            await self.run_command(
                command=mtmount.commands.BothAxesResetAlarm(), use_read_loop=True
            )
            for device in devices:
                assert not device.alarm_on
                assert not device.power_on
                assert not device.enabled
                assert not device.homed
                assert not device.tracking_enabled

            # Motion-related commands are rejected if either or both
            # axes are powered off. Note:
            # * BothAxesEnableTracking is also rejected
            #   because the axes have not been homed
            # * BothAxesTrackTarget is also rejected
            #   because tracking has not been enabled
            for azimuth_on, elevation_on in itertools.product(
                (False, True), (False, True)
            ):
                if azimuth_on and elevation_on:
                    continue
                await self.run_command(
                    command=mtmount.commands.AzimuthPower(on=azimuth_on),
                    use_read_loop=True,
                )
                await self.run_command(
                    command=mtmount.commands.ElevationPower(on=elevation_on),
                    use_read_loop=True,
                )
                assert azimuth_device.power_on == azimuth_on
                assert elevation_device.power_on == elevation_on

                for command in (
                    mtmount.commands.BothAxesEnableTracking(),
                    mtmount.commands.BothAxesHome(),
                    mtmount.commands.BothAxesMove(azimuth=45, elevation=45),
                    mtmount.commands.BothAxesTrackTarget(
                        azimuth=45,
                        elevation=45,
                        azimuth_velocity=0,
                        elevation_velocity=0,
                        tai=utils.current_tai(),
                    ),
                    mtmount.commands.BothAxesStop(),
                ):
                    await self.run_command(
                        command=command,
                        use_read_loop=True,
                        final_reply_code=mtmount.ReplyId.CMD_REJECTED,
                    )

            await self.run_command(
                command=mtmount.commands.BothAxesPower(on=True), use_read_loop=True
            )
            for device in devices:
                assert not device.alarm_on
                assert device.power_on
                assert device.enabled
                assert not device.homed
                assert not device.tracking_enabled

            # Tracking command are rejected unless the axes are homed
            # (eventually BothAxesMove move may also be rejected,
            # but for now it is not). Note:
            # * BothAxesTrackTarget is also rejected
            #   because tracking has not been enabled
            for command in (
                mtmount.commands.BothAxesEnableTracking(),
                mtmount.commands.BothAxesTrackTarget(
                    azimuth=45,
                    elevation=45,
                    azimuth_velocity=0,
                    elevation_velocity=0,
                    tai=utils.current_tai(),
                ),
            ):
                await self.run_command(
                    command=command,
                    use_read_loop=True,
                    final_reply_code=mtmount.ReplyId.CMD_REJECTED,
                )

            await self.run_command(
                command=mtmount.commands.BothAxesHome(), use_read_loop=True
            )
            for device in devices:
                assert not device.alarm_on
                assert device.power_on
                assert device.enabled
                assert device.homed
                assert not device.tracking_enabled

            await self.run_command(
                command=mtmount.commands.BothAxesEnableTracking(), use_read_loop=True
            )
            for device in devices:
                assert not device.alarm_on
                assert device.power_on
                assert device.enabled
                assert device.homed
                assert device.tracking_enabled

    async def test_both_axes_move(self):
        """Test the BothAxesMove command and InPosition replies."""
        async with self.make_controller():

            n_in_position = len(self.controller.in_position_dict)
            n_motion_controller_state = len(
                self.controller.motion_controller_state_dict
            )
            n_axis_motion_state = len(self.controller.axis_motion_state_dict)
            n_power_state = len(self.controller.power_state_dict)
            replies = await asyncio.wait_for(
                self.read_replies(
                    wait_reply_codes=[mtmount.ReplyId.IN_POSITION] * n_in_position
                    + [mtmount.ReplyId.MOTION_CONTROLLER_STATE]
                    * n_motion_controller_state
                    + [mtmount.ReplyId.POWER_STATE] * n_power_state
                    + [mtmount.ReplyId.AXIS_MOTION_STATE] * n_axis_motion_state,
                ),
                timeout=START_TIMEOUT,
            )
            for reply in replies:
                if reply.id == mtmount.ReplyId.IN_POSITION:
                    assert reply.axis in self.controller.axis_motion_state_dict
                    assert not reply.inPosition
                elif reply.id == mtmount.ReplyId.MOTION_CONTROLLER_STATE:
                    assert (
                        reply.system in self.controller.motion_controller_state_dict
                        or reply.system == System.AZIMUTH_CABLE_WRAP
                    )
                    nelts = self.controller.motion_controller_state_nelts[reply.system]
                    assert reply.motionControllerState == [PowerState.OFF] * nelts
                elif reply.id == mtmount.ReplyId.POWER_STATE:
                    assert (
                        reply.system in self.controller.power_state_dict
                        or reply.system == System.AZIMUTH_CABLE_WRAP
                    )
                    assert reply.powerState == PowerState.OFF
                elif reply.id == mtmount.ReplyId.AXIS_MOTION_STATE:
                    assert reply.axis in self.controller.axis_motion_state_dict
                    assert reply.state == AxisMotionState.STOPPED

            both_axis_device_ids = (System.AZIMUTH, System.ELEVATION)
            devices = [
                self.controller.device_dict[device_id]
                for device_id in both_axis_device_ids
            ]
            start_positions = [
                device.actuator.path.at(utils.current_tai()).position
                for device in devices
            ]
            end_positions = [pos + 1 for pos in start_positions]
            move_command = mtmount.commands.BothAxesMove(
                azimuth=end_positions[0], elevation=end_positions[1]
            )
            await self.run_command(
                command=mtmount.commands.MainAxesPowerSupplyPower(on=True),
                use_read_loop=True,
                wait_reply_codes=[mtmount.ReplyId.POWER_STATE],
            )
            non_cmd_replies = await self.run_command(
                command=mtmount.commands.BothAxesPower(on=True),
                use_read_loop=True,
                wait_reply_codes=[
                    mtmount.ReplyId.MOTION_CONTROLLER_STATE,
                    mtmount.ReplyId.MOTION_CONTROLLER_STATE,
                    mtmount.ReplyId.POWER_STATE,
                    mtmount.ReplyId.POWER_STATE,
                ],
            )
            assert len(non_cmd_replies) == 4
            assert len(set(reply.id for reply in non_cmd_replies)) == 2
            for reply in non_cmd_replies:
                if reply.id == mtmount.ReplyId.MOTION_CONTROLLER_STATE:
                    nelts = self.controller.motion_controller_state_nelts[reply.system]
                    assert reply.motionControllerState == [PowerState.ON] * nelts
                elif reply.id == mtmount.ReplyId.POWER_STATE:
                    assert reply.powerState == PowerState.ON
                    assert not hasattr(reply, "elementsPowerState")
                else:
                    raise AssertionError(f"unrecognized reply ID {reply['id']}")
            for device in devices:
                assert device.power_on
                assert device.enabled
                assert not device.tracking_enabled

            estimated_move_time = 2  # seconds
            t0 = time.monotonic()
            non_cmd_replies = await self.run_command(
                command=move_command,
                use_read_loop=True,
                timeout=estimated_move_time + STD_TIMEOUT,
                wait_reply_codes=[
                    mtmount.ReplyId.IN_POSITION,
                    mtmount.ReplyId.IN_POSITION,
                ],
            )
            dt = time.monotonic() - t0
            print(f"Move duration={dt:0.2f} second")
            assert len(non_cmd_replies) == 2
            for device in devices:
                assert device.power_on
                assert device.enabled
                assert not device.tracking_enabled
            for reply in non_cmd_replies:
                assert reply.inPosition
            assert {reply.axis for reply in non_cmd_replies} == set(
                both_axis_device_ids
            )

            # Test telemetry after move
            axis_telems = [
                await self.next_telemetry(topic_id)
                for topic_id in (
                    mtmount.TelemetryTopicId.AZIMUTH,
                    mtmount.TelemetryTopicId.ELEVATION,
                )
            ]
            for i in (0, 1):
                for name in (
                    "actualPosition",
                    "demandPosition",
                    "actualVelocity",
                    "demandVelocity",
                    "actualTorque",
                ):
                    if name in {"actualPosition", "demandPosition"}:
                        desired_value = end_positions[i]
                    else:
                        desired_value = 0
                    assert axis_telems[i][name] == pytest.approx(desired_value)

            # Test that out-of-range positions are rejected.
            for i, device in enumerate(devices):
                for bad_axis_position in (
                    device.cmd_limits.min_position - 0.001,
                    device.cmd_limits.max_position + 0.001,
                ):
                    bad_positions = end_positions[:]
                    bad_positions[i] = bad_axis_position
                    await self.run_command(
                        command=mtmount.commands.BothAxesMove(
                            azimuth=bad_positions[0], elevation=bad_positions[1]
                        ),
                        use_read_loop=True,
                        final_reply_code=mtmount.ReplyId.CMD_REJECTED,
                    )
                assert device.power_on
                assert device.enabled
                assert not device.tracking_enabled

    async def test_both_axes_track_target(self):
        """Test the BothAxesTrackTarget command and InPosition replies."""
        async with self.make_controller():
            # Wait for IN_POSITION replies for azimuth, elevation,
            # and camera cable wrap.
            seen_axes = set()
            replies = await asyncio.wait_for(
                self.read_replies(
                    wait_reply_codes=[mtmount.ReplyId.IN_POSITION] * 3,
                ),
                timeout=START_TIMEOUT,
            )
            for reply in replies:
                assert not reply.inPosition
                seen_axes.add(reply.axis)
            assert seen_axes == {
                System.AZIMUTH,
                System.ELEVATION,
                System.CAMERA_CABLE_WRAP,
            }

            both_axis_device_ids = (System.AZIMUTH, System.ELEVATION)
            devices = [
                self.controller.device_dict[device_id]
                for device_id in both_axis_device_ids
            ]

            await self.run_command(
                command=mtmount.commands.MainAxesPowerSupplyPower(on=True),
                use_read_loop=True,
            )
            for device in devices:
                assert not device.power_on
                assert not device.enabled
                assert not device.homed
                assert not device.tracking_enabled

            await self.run_command(
                command=mtmount.commands.BothAxesPower(on=True), use_read_loop=True
            )
            for device in devices:
                assert device.power_on
                assert device.enabled
                assert not device.homed
                assert not device.tracking_enabled

            non_cmd_replies = await self.run_command(
                command=mtmount.commands.BothAxesHome(),
                use_read_loop=True,
                wait_reply_codes=[
                    mtmount.ReplyId.IN_POSITION,
                    mtmount.ReplyId.IN_POSITION,
                ],
            )
            for device in devices:
                assert device.power_on
                assert device.enabled
                assert device.homed
                assert not device.tracking_enabled
            assert len(non_cmd_replies) == 2
            for reply in non_cmd_replies:
                assert reply.id == mtmount.ReplyId.IN_POSITION
                assert reply.inPosition
            assert {reply.axis for reply in non_cmd_replies} == {0, 1}

            await self.run_command(
                command=mtmount.commands.BothAxesEnableTracking(),
                use_read_loop=True,
            )
            for device in devices:
                assert device.power_on
                assert device.enabled
                assert device.tracking_enabled

            start_positions = [
                device.actuator.path.at(utils.current_tai()).position
                for device in devices
            ]
            end_positions = [start_position + 3 for start_position in start_positions]
            previous_tai = 0
            # Send tracking updates until an IN_POSITION reply is seen.
            not_in_position_axes = set()
            in_position_axes = set()
            # How many tracking commands to send
            # once the slew is done
            num_tracking_remaining = 10
            t0 = utils.current_tai()
            # The maximum time for slewing and tracking (sec);
            # be generous, because this is used as a timeout
            max_slewing_and_tracking_duration = 10
            timeout_tai = t0 + max_slewing_and_tracking_duration
            while num_tracking_remaining > 0:
                tai = utils.current_tai()
                assert tai < timeout_tai, "Timed out while slewing and tracking"
                # Work around non-monotonic clocks, which are
                # sometimes seen when running Docker on macOS.
                if tai <= previous_tai:
                    tai = previous_tai + 0.001
                track_command = mtmount.commands.BothAxesTrackTarget(
                    azimuth=end_positions[0],
                    elevation=end_positions[1],
                    azimuth_velocity=0,
                    elevation_velocity=0,
                    tai=tai,
                )
                non_cmd_replies = await self.run_command(
                    command=track_command,
                    final_reply_code=mtmount.ReplyId.CMD_ACKNOWLEDGED,
                    flush=False,
                    use_read_loop=True,
                    other_reply_codes=[mtmount.ReplyId.IN_POSITION],
                )
                for reply in non_cmd_replies:
                    if reply.axis in not_in_position_axes:
                        assert reply.id == mtmount.ReplyId.IN_POSITION
                        in_position_axes.add(reply.axis)
                    else:
                        not_in_position_axes.add(reply.axis)
                if len(in_position_axes) == 2:
                    num_tracking_remaining -= 1
                previous_tai = tai
                await asyncio.sleep(0.1)
            dt = utils.current_tai() - t0
            print(f"Slewing and tracking took {dt:0.2f} seconds")

            # Test that out-of-range tracking updates are rejected,
            # leaving the device enabled and in tracking mode.
            print("***** starting out-of-bounds tracking test; going verbose")
            for device in devices:
                device.verbose = True
            for i, device in enumerate(devices):
                for bad_axis_position in (
                    device.cmd_limits.min_position - 0.001,
                    device.cmd_limits.max_position + 0.001,
                ):
                    bad_positions = end_positions[:]
                    bad_positions[i] = bad_axis_position
                    tai = utils.current_tai()
                    if tai < previous_tai:
                        tai = previous_tai + 0.001
                    bad_track_command = mtmount.commands.BothAxesTrackTarget(
                        azimuth=bad_positions[0],
                        elevation=bad_positions[1],
                        azimuth_velocity=0,
                        elevation_velocity=0,
                        tai=tai,
                    )
                    await self.run_command(
                        bad_track_command,
                        use_read_loop=True,
                        final_reply_code=mtmount.ReplyId.CMD_REJECTED,
                    )
                    previous_tai = tai

            for i, device in enumerate(devices):
                for bad_velocity in (
                    -device.cmd_limits.max_velocity - 0.001,
                    device.cmd_limits.max_velocity + 0.001,
                ):
                    bad_velocities = [0, 0]
                    bad_velocities[i] = bad_velocity
                    tai = utils.current_tai()
                    if tai < previous_tai:
                        tai = previous_tai + 0.001
                    bad_track_command = mtmount.commands.BothAxesTrackTarget(
                        azimuth=end_positions[0],
                        elevation=end_positions[1],
                        azimuth_velocity=bad_velocities[0],
                        elevation_velocity=bad_velocities[1],
                        velocity=bad_velocity,
                        tai=tai,
                    )
                    await self.run_command(
                        bad_track_command,
                        use_read_loop=True,
                        final_reply_code=mtmount.ReplyId.CMD_REJECTED,
                    )
                    previous_tai = tai

            for device in devices:
                assert device.power_on
                assert device.enabled
                assert device.tracking_enabled

            non_cmd_replies = await self.run_command(
                command=mtmount.commands.BothAxesStop(),
                use_read_loop=True,
                wait_reply_codes=[mtmount.ReplyId.IN_POSITION],
            )
            assert device.power_on
            assert device.enabled
            assert not device.tracking_enabled
            assert len(non_cmd_replies) == 1
            reply = non_cmd_replies[0]
            assert reply.id == mtmount.ReplyId.IN_POSITION
            assert not reply.inPosition
            assert reply.axis == 0

    async def test_command_failed(self):
        async with self.make_controller():
            device = self.controller.device_dict[System.MIRROR_COVERS]
            await self.run_command(
                command=mtmount.commands.MirrorCoversPower(drive=-1, on=True),
                use_read_loop=True,
            )

            # Issue a background command that should fail
            # (after being acknowledged).
            # The mirror covers start deployed, so this will fail quickly.
            device.fail_next_command = True
            await self.run_command(
                command=mtmount.commands.MirrorCoversDeploy(drive=-1),
                use_read_loop=True,
                final_reply_code=mtmount.ReplyId.CMD_FAILED,
            )

    async def test_command_superseded(self):
        async with self.make_controller():
            device = self.controller.device_dict[System.MIRROR_COVERS]

            await self.run_command(
                command=mtmount.commands.MirrorCoversPower(drive=-1, on=True),
                use_read_loop=True,
            )

            # Issue a background command that should take some time
            # then supersede it.
            task = asyncio.create_task(
                self.run_command(
                    command=mtmount.commands.MirrorCoversRetract(drive=-1),
                    use_read_loop=True,  # False is also fine
                    final_reply_code=mtmount.ReplyId.CMD_SUPERSEDED,
                )
            )
            await asyncio.sleep(0.1)

            # Supersede the command by directly commanding the device,
            # because run_command is too primitive to handle
            # more than one command at a time.
            stop_command = mtmount.commands.MirrorCoversStop(drive=-1)
            device.do_stop(stop_command)
            await task

    async def test_handle_command(self):
        await self.check_command_sequence(use_read_loop=False)

    async def test_read_loop(self):
        await self.check_command_sequence(use_read_loop=True)

    async def test_get_actual_settings(self):
        # Check that the GET_ACTUAL_SETTINGS command is accepted
        # regardless of commander by setting a different commander.
        async with self.make_controller(commander=mtmount.Source.HHD):
            replies = await self.run_command(
                command=mtmount.commands.GetActualSettings(),
                return_all_replies=True,
                use_read_loop=True,
            )
            saw_detailed_settings_applied = False
            for reply in replies:
                if reply.id == mtmount.ReplyId.DETAILED_SETTINGS_APPLIED:
                    saw_detailed_settings_applied = True
                    # Make a dict that only has the parameters
                    # (no "id" or "timestamp" field)
                    reply_dict = vars(reply)
                    del reply_dict["id"]
                    del reply_dict["timestamp"]
                    assert reply_dict == self.controller.detailed_settings

                    ccw_device = self.controller.device_dict[System.CAMERA_CABLE_WRAP]
                    ccw_actuator = ccw_device.actuator
                    ccw_cmd_limits = ccw_device.cmd_limits
                    ccw_settings = reply.CW["CCW"]
                    assert ccw_settings["MinPosition"] == ccw_cmd_limits.min_position
                    assert ccw_settings["MaxPosition"] == ccw_cmd_limits.max_position
                    assert ccw_settings["MaxSpeed"] == ccw_cmd_limits.max_velocity
                    assert (
                        ccw_settings["MaxAcceleration"]
                        == ccw_cmd_limits.max_acceleration
                    )
                    assert ccw_settings["TrackingSpeed"] == ccw_actuator.max_velocity
                    assert (
                        ccw_settings["TrackingAcceleration"]
                        == ccw_actuator.max_acceleration
                    )

                    for axis_name, system_id in (
                        ("Azimuth", System.AZIMUTH),
                        ("Elevation", System.ELEVATION),
                    ):
                        axis_device = self.controller.device_dict[system_id]
                        axis_actuator = axis_device.actuator
                        axis_cmd_limits = axis_device.cmd_limits
                        axis_settings = reply.MainAxis[axis_name]
                        for limit_enable_name in (
                            "LimitsMinPositionEnable",
                            "LimitsMaxPositionEnable",
                            "LimitsNegativeSoftwareLimitEnable",
                            "LimitsPositiveSoftwareLimitEnable",
                            "LimitsNegativeAdjustableSoftwareLimitEnable",
                            "LimitsPositiveAdjustableSoftwareLimitEnable",
                            "LimitsNegativeLimitSwitchEnable",
                            "LimitsPositiveLimitSwitchEnable",
                        ):
                            assert axis_settings[limit_enable_name]
                        for limit_enable_name in (
                            "LimitsNegativeOperationalLimitSwitchEnable",
                            "LimitsPositiveOperationalLimitSwitchEnable",
                        ):
                            if axis_name == "Azimuth":
                                # Azimuth has no operational L2 limits,
                                # but still reports this setting.
                                assert not axis_settings[limit_enable_name]
                            else:
                                assert axis_settings[limit_enable_name]
                        assert (
                            axis_settings["LimitsMinPositionValue"]
                            == axis_cmd_limits.min_position
                        )
                        assert (
                            axis_settings["LimitsMaxPositionValue"]
                            == axis_cmd_limits.max_position
                        )
                        assert (
                            axis_settings["TcsMaxVelocity"]
                            == axis_cmd_limits.max_velocity
                        )
                        assert (
                            axis_settings["TcsMaxAcceleration"]
                            == axis_cmd_limits.max_acceleration
                        )
                        assert (
                            axis_settings["SoftmotionTrackingMaxSpeed"]
                            == axis_actuator.max_velocity
                        )
                        assert (
                            axis_settings["SoftmotionTrackingMaxAcceleration"]
                            == axis_actuator.max_acceleration
                        )
            assert saw_detailed_settings_applied

    async def test_state_info_command(self):
        # Check that the STATE_INFO command is accepted
        # regardless of commander by setting a different commander.
        async with self.make_controller(commander=mtmount.Source.HHD):
            # Wait for one iteration of telemetry,
            # to avoid duplicate message.
            await asyncio.wait_for(
                self.controller.wait_telemetry(), timeout=STD_TIMEOUT
            )

            replies = await self.run_command(
                command=mtmount.commands.StateInfo(),
                return_all_replies=True,
                use_read_loop=True,
            )
            limits_systems = []
            motion_state_axes = []
            chiller_state_systems = []
            motion_controller_state_systems = []
            power_state_systems = []
            for reply in replies:
                if reply.id == mtmount.ReplyId.AVAILABLE_SETTINGS:
                    assert len(reply.sets) == len(self.controller.available_settings)

                    for data, desired in zip(
                        reply.sets, self.controller.available_settings
                    ):
                        assert data.keys() == desired.keys()
                        for key in ("name", "description"):
                            assert isinstance(data[key], str)
                            assert data[key] == desired[key]
                        for key in ("createdDate", "modifiedDate"):
                            assert data[key] == desired[key].iso
                        assert desired["modifiedDate"] >= desired["createdDate"]

                elif reply.id == mtmount.ReplyId.AXIS_MOTION_STATE:
                    motion_state_axes.append(reply.axis)
                    assert reply.state == AxisMotionState.STOPPED
                elif reply.id == mtmount.ReplyId.AZIMUTH_TOPPLE_BLOCK:
                    assert not reply.reverse
                    assert not reply.forward
                elif reply.id == mtmount.ReplyId.CHILLER_STATE:
                    chiller_state_systems.append(reply.system)
                    nelts = self.controller.chiller_state_nelts[reply.system]
                    desired_trackAmbient = [True] * nelts
                    desired_temperature = [self.controller.ambient_temperature] * nelts
                    assert reply.trackAmbient == desired_trackAmbient
                    numpy.testing.assert_allclose(
                        reply.temperature, desired_temperature
                    )
                elif reply.id == mtmount.ReplyId.COMMANDER:
                    assert reply.actualCommander == mtmount.Source.HHD
                elif reply.id == mtmount.ReplyId.DEPLOYABLE_PLATFORMS_MOTION_STATE:
                    assert reply.state == DeployableMotionState.RETRACTED
                    assert reply.elementsState == [DeployableMotionState.RETRACTED] * 2

                elif reply.id == mtmount.ReplyId.ELEVATION_LOCKING_PIN_MOTION_STATE:
                    assert reply.state == ElevationLockingPinMotionState.UNLOCKED
                    assert (
                        reply.elementsState
                        == [ElevationLockingPinMotionState.UNLOCKED] * 2
                    )
                elif reply.id == mtmount.ReplyId.LIMITS:
                    limits_systems.append(reply.system)
                    assert reply.limits == [0]
                elif reply.id in (
                    mtmount.ReplyId.MIRROR_COVERS_MOTION_STATE,
                    mtmount.ReplyId.MIRROR_COVER_LOCKS_MOTION_STATE,
                ):
                    assert reply.state == DeployableMotionState.DEPLOYED
                    assert reply.elementsState == [DeployableMotionState.DEPLOYED] * 4
                elif reply.id == mtmount.ReplyId.MOTION_CONTROLLER_STATE:
                    motion_controller_state_systems.append(reply.system)
                    nelts = self.controller.motion_controller_state_nelts[reply.system]
                    expected_state = PowerState.OFF
                    assert reply.motionControllerState == [expected_state] * nelts
                elif reply.id == mtmount.ReplyId.POWER_STATE:
                    power_state_systems.append(reply.system)
                    nelts = self.controller.power_state_nelts[reply.system]
                    if reply.system in {
                        System.AZIMUTH_DRIVES_THERMAL,
                        System.ELEVATION_DRIVES_THERMAL,
                        System.AZ0101_CABINET_THERMAL,
                        System.MODBUS_TEMPERATURE_CONTROLLERS,
                        System.MAIN_CABINET_THERMAL,
                    }:
                        expected_state = PowerState.ON
                    else:
                        expected_state = PowerState.OFF
                    assert reply.powerState == expected_state
                    if nelts > 1:
                        assert reply.elementsPowerState == [expected_state] * nelts
                    else:
                        assert not hasattr(reply, "elementsPowerState")

                elif reply.id == mtmount.ReplyId.SAFETY_INTERLOCKS:
                    params_dict = vars(reply)
                    del params_dict["id"]
                    del params_dict["timestamp"]
                    expected_fields = (
                        "causes",
                        "subcausesEmergencyStop",
                        "subcausesLimitSwitch",
                        "subcausesDeployablePlatform",
                        "subcausesDoorHatchLadder",
                        "subcausesMirrorCover",
                        "subcausesLockingPin",
                        "subcausesCapacitorDoor",
                        "subcausesBrakesFailed",
                        "effects",
                    )
                    assert set(expected_fields) == params_dict.keys()
                    for field in expected_fields:
                        assert params_dict[field] == 0

            axes_systems = set(
                [
                    System.ELEVATION,
                    System.AZIMUTH,
                    System.CAMERA_CABLE_WRAP,
                ]
            )
            assert len(set(limits_systems)) == len(limits_systems)
            assert set(limits_systems) >= axes_systems
            assert len(set(motion_state_axes)) == len(motion_state_axes)
            assert set(motion_state_axes) >= axes_systems
            assert len(set(chiller_state_systems)) == len(chiller_state_systems)
            assert (
                set(chiller_state_systems) == self.controller.chiller_state_nelts.keys()
            )
            assert len(set(motion_controller_state_systems)) == len(
                motion_controller_state_systems
            )
            assert (
                set(motion_controller_state_systems)
                == self.controller.motion_controller_state_nelts.keys()
            )
            assert len(set(power_state_systems)) == len(power_state_systems)
            assert set(power_state_systems) == self.controller.power_state_nelts.keys()

    async def check_command_sequence(self, use_read_loop):
        async with self.make_controller():
            device = self.controller.device_dict[System.MIRROR_COVER_LOCKS]

            # Issue a synchronous command
            assert not device.power_on
            on_command = mtmount.commands.MirrorCoverLocksPower(drive=-1, on=True)
            await self.run_command(command=on_command, use_read_loop=use_read_loop)
            assert device.power_on

            # Issue a background command
            assert device.actuator.position() == pytest.approx(device.deployed_position)
            deploy_command = mtmount.commands.MirrorCoverLocksMoveAll(
                drive=-1, deploy=False
            )
            await self.run_command(command=deploy_command, use_read_loop=use_read_loop)
            assert device.actuator.position() == pytest.approx(
                device.retracted_position
            )

            # Issue a command (AzimuthTrackTarget) that gets no Done reply
            # but first enable the device and tracking
            device = self.controller.device_dict[System.AZIMUTH]
            assert not device.power_on
            assert not device.enabled
            assert not device.homed
            assert not device.tracking_enabled
            await self.run_command(
                mtmount.commands.MainAxesPowerSupplyPower(on=True),
                use_read_loop=use_read_loop,
            )
            await self.run_command(
                command=mtmount.commands.AzimuthPower(on=True),
                use_read_loop=use_read_loop,
            )
            assert device.power_on
            assert device.enabled
            assert not device.homed
            assert not device.tracking_enabled
            await self.run_command(
                command=mtmount.commands.AzimuthHome(),
                use_read_loop=use_read_loop,
            )
            assert device.power_on
            assert device.enabled
            assert device.homed
            assert not device.tracking_enabled
            await self.run_command(
                command=mtmount.commands.AzimuthEnableTracking(on=True),
                use_read_loop=use_read_loop,
            )
            assert device.power_on
            assert device.enabled
            assert device.tracking_enabled
            track_command = mtmount.commands.AzimuthTrackTarget(
                position=45, velocity=0, tai=utils.current_tai()
            )
            await self.run_command(
                command=track_command,
                final_reply_code=mtmount.ReplyId.CMD_ACKNOWLEDGED,
                use_read_loop=use_read_loop,
            )
            # Issue one more command to be sure we really didn't get
            # a Done reply for the previous command
            await self.run_command(
                command=mtmount.commands.AzimuthStop(), use_read_loop=use_read_loop
            )
            assert device.power_on
            assert device.enabled
            assert not device.tracking_enabled

            # Try a command that will fail
            # (tracking while tracking not enabled)
            await self.run_command(
                command=track_command,
                final_reply_code=mtmount.ReplyId.CMD_REJECTED,
                use_read_loop=use_read_loop,
            )

            # Try a command not supported by the mock controller.
            # Add it to mtmount.commands.CommandDict so the mock controller
            # can parse it as a command.
            unsupported_command = UnsupportedCommand()
            assert unsupported_command.command_code not in mtmount.commands.CommandDict
            new_command_dict = mtmount.commands.CommandDict.copy()
            new_command_dict[unsupported_command.command_code] = UnsupportedCommand
            with unittest.mock.patch(
                "lsst.ts.mtmount.commands.CommandDict", new_command_dict
            ):
                await self.run_command(
                    command=unsupported_command,
                    final_reply_code=mtmount.ReplyId.CMD_REJECTED,
                    use_read_loop=use_read_loop,
                )

    async def next_telemetry(self, topic_id, timeout=STD_TIMEOUT):
        """Wait for a new instance of the specified telemetry topic."""
        topic_id = mtmount.TelemetryTopicId(topic_id)
        self.telemetry_dict[topic_id] = None
        t0 = time.monotonic()
        data = None
        while True:
            dt = time.monotonic() - t0
            if dt > timeout:
                self.fail(f"No new {topic_id!r} telemetry sample seen in {timeout} sec")
            await asyncio.sleep(0.1)
            data = self.telemetry_dict[topic_id]
            if data is not None:
                return data

    async def test_command_queue(self):
        async with self.make_controller():
            assert self.controller.command_queue is None

            # Create the command queue with specified maxsize.
            maxsize = 47
            self.controller.set_command_queue(
                queue_heartbeat_commands=True, maxsize=maxsize
            )
            assert isinstance(self.controller.command_queue, asyncio.Queue)
            assert self.controller.command_queue.maxsize == maxsize
            assert self.controller.queue_heartbeat_commands

            # Run a heartbeat command and see that it is queued.
            # Note that the heartbeat command receives no ack,
            # so simply write it.
            heartbeat_command = mtmount.commands.Heartbeat()
            self.command_writer.write(heartbeat_command.encode())
            await self.command_writer.drain()
            queued_heartbeat_command = await asyncio.wait_for(
                self.controller.command_queue.get(), timeout=STD_TIMEOUT
            )
            assert queued_heartbeat_command == heartbeat_command

            # Replace the command queue with default maxsize (unlimited).
            self.controller.set_command_queue(
                queue_heartbeat_commands=False,
            )
            assert isinstance(self.controller.command_queue, asyncio.Queue)
            assert self.controller.command_queue.maxsize == 0
            assert not self.controller.queue_heartbeat_commands

            # Run a heartbeat command; it should not be queued,
            # as we will see when we retrieve other commands.
            # Note that the heartbeat command receives no ack,
            # so simply write it.
            heartbeat_command = mtmount.commands.Heartbeat()
            self.command_writer.write(heartbeat_command.encode())
            await self.command_writer.drain()

            # Run two non-heartbeat commands and check that they are queued.
            device = self.controller.device_dict[System.MIRROR_COVER_LOCKS]
            assert not device.power_on
            on_command = mtmount.commands.MirrorCoverLocksPower(drive=-1, on=True)
            await self.run_command(command=on_command, use_read_loop=True)
            assert device.power_on
            off_command = mtmount.commands.MirrorCoverLocksPower(drive=-1, on=False)
            await self.run_command(command=off_command, use_read_loop=True)
            assert not device.power_on

            # Check the two commands on the command queue.
            queued_on_command = await asyncio.wait_for(
                self.controller.command_queue.get(), timeout=STD_TIMEOUT
            )
            assert queued_on_command == on_command
            queued_off_command = await asyncio.wait_for(
                self.controller.command_queue.get(), timeout=STD_TIMEOUT
            )
            assert queued_off_command == off_command
            assert on_command != off_command
            assert self.controller.command_queue.empty()

            # Delete the command queue.
            self.controller.delete_command_queue()
            assert self.controller.command_queue is None

    async def test_initial_telemetry(self):
        async with self.make_controller():
            for topic_id, prefix in (
                (mtmount.TelemetryTopicId.AZIMUTH, "az"),
                (mtmount.TelemetryTopicId.ELEVATION, "el"),
            ):
                # Elevation does not start at position 0,
                # so read the position.
                system_id = {
                    mtmount.TelemetryTopicId.AZIMUTH: System.AZIMUTH,
                    mtmount.TelemetryTopicId.ELEVATION: System.ELEVATION,
                }[topic_id]
                device = self.controller.device_dict[system_id]
                # Work around Docker time issues on macOS with an offset
                tai0 = utils.current_tai() - 0.1
                axis_telem = await self.next_telemetry(topic_id)
                for name in (
                    "actualPosition",
                    "demandPosition",
                    "actualVelocity",
                    "demandVelocity",
                    "actualTorque",
                ):
                    if name in {"actualPosition", "demandPosition"}:
                        desired_value = device.actuator.path.at(tai0).position
                    else:
                        desired_value = 0
                    assert axis_telem[name] == pytest.approx(desired_value)
                assert axis_telem["timestamp"] > tai0

            # Work around Docker time issues on macOS with an offset
            tai0 = utils.current_tai() - 0.1
            ccw_telem = await self.next_telemetry(
                mtmount.TelemetryTopicId.CAMERA_CABLE_WRAP
            )
            for name in (
                "actualPosition",
                "actualVelocity",
                "actualAcceleration",
                "demandPosition",
                "demandVelocity",
                "actualTorquePercentage1",
                "actualTorquePercentage2",
            ):
                assert ccw_telem[name] == 0
            assert ccw_telem["timestamp"] > tai0

    async def test_mirror_cover_system_commands(self):
        async with self.make_controller():
            covers_device = self.controller.device_dict[System.MIRROR_COVERS]
            locks_device = self.controller.device_dict[System.MIRROR_COVER_LOCKS]

            # Should fail until both devices are on
            for covers_on, locks_on in itertools.product((False, True), (False, True)):
                await self.run_command(
                    command=mtmount.commands.MirrorCoversPower(on=covers_on),
                    use_read_loop=True,
                )
                assert covers_device.power_on == covers_on
                await self.run_command(
                    command=mtmount.commands.MirrorCoverLocksPower(on=locks_on),
                    use_read_loop=True,
                )
                assert locks_device.power_on == locks_on
                if covers_on and locks_on:
                    await self.run_command(
                        command=mtmount.commands.MirrorCoverSystemDeploy(),
                        use_read_loop=True,
                    )
                    await self.run_command(
                        command=mtmount.commands.MirrorCoverSystemRetract(),
                        use_read_loop=True,
                    )
                else:
                    await self.run_command(
                        command=mtmount.commands.MirrorCoverSystemDeploy(),
                        use_read_loop=True,
                        final_reply_code=mtmount.ReplyId.CMD_FAILED,
                    )
                    await self.run_command(
                        command=mtmount.commands.MirrorCoverSystemRetract(),
                        use_read_loop=True,
                        final_reply_code=mtmount.ReplyId.CMD_FAILED,
                    )
