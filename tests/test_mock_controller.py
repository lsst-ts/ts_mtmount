# This file is part of ts_MTMount.
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
import json
import logging
import time
import types
import unittest
import unittest.mock

import numpy.testing

from lsst.ts import salobj
from lsst.ts import MTMount
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
    MTMount.enums.CommandCode.TRANSFER_FUNCTION_AZIMUTH_EXCITATION
)


class UnsupportedCommand(MTMount.commands.Command):
    field_infos = MTMount.commands.make_command_field_infos(UNSUPPORTED_COMMAND_CODE)


class MockControllerTestCase(unittest.IsolatedAsyncioTestCase):
    @contextlib.asynccontextmanager
    async def make_controller(self, commander=MTMount.Source.CSC):
        """Make a mock controller as self.controller.

        Parameters
        ----------
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
        self.controller = MTMount.mock.Controller(
            log=log, commander=commander, random_ports=True
        )
        t0 = time.monotonic()
        await asyncio.wait_for(self.controller.start_task, timeout=START_TIMEOUT)

        # Dict of task_id: parsed data
        self.telemetry_dict = {topic_id: None for topic_id in MTMount.TelemetryTopicId}

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
            self.command_reader.readuntil(MTMount.LINE_TERMINATOR), timeout=timeout
        )
        reply_dict = json.loads(read_bytes)
        return types.SimpleNamespace(
            id=MTMount.ReplyId(reply_dict["id"]),
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

        wait_reply_codes : `list` [`MTMount.Reply`]
            Reply codes of messages to wait for, one reply per entry,
            but not necessarily in order.
            If a reply code is listed N times then wait for N such replies.
            Extra replies with these reply codes are ignored,
            unless the reply code also appears in ``other_reply_codes``.
        other_reply_codes : `list` [`MTMount.Reply`], optional
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
        replies : `List` [`MTMount.Reply`]
            The read replies.
        """
        nonack_reply_codes_remaining = wait_reply_codes.copy()
        other_reply_codes = set(other_reply_codes)
        replies = []
        while nonack_reply_codes_remaining:
            reply = await self.read_one_reply()
            if reply.id in nonack_reply_codes_remaining:
                nonack_reply_codes_remaining.remove(reply.id)
                replies.append(reply)
            elif return_all_replies or reply.id in other_reply_codes:
                replies.append(reply)
        return replies

    async def run_command(
        self,
        command,
        use_read_loop,
        flush=True,
        final_reply_code=MTMount.ReplyId.CMD_SUCCEEDED,
        wait_reply_codes=(),
        other_reply_codes=(),
        return_all_replies=False,
        timeout=STD_TIMEOUT,
    ):
        """Run one command and check the replies.

        Return a list of non-Ack/Done/NoAck replies.

        Parameters
        ----------
        command : `MTMount.Command`
            Command to send.
        use_read_loop : `bool`
            If True then send the command via TCP/IP.
            If False then call self.controller.handle_command directly.
        flush : `bool`, optional
            If True then flush replies before issuing the command.
        final_reply_code : `MTMount.ReplyId`, optional
            The expected final CMD_x reply code.
            Defaults to CMD_SUCCEEDED.
        wait_reply_codes : `list` [`MTMount.Reply`], optional
            non-CMD_x reply codes of messages to wait for, one reply per entry,
            but not necessarily in order.
            If a reply code is listed N times then wait for N such replies.
            Extra replies with these reply codes are ignored,
            unless the reply code also appears in ``other_reply_codes``.
        other_reply_codes : `list` [`MTMount.Reply`], optional
            Non-CMD_x reply codes of other messages to return, if seen,
            in addition to those in ``wait_reply_codes``.
            Duplicates are silently ignored.
        return_all_replies : `bool`, optional
            Return all replies?
            If True then other_reply_codes must be empty.
        timeout : `float`
            Timeout waiting for the Done reply (second).
            Ignored for other reads.

        Returns
        -------
        non_cmd_replies : `List` [`MTMount.Reply`]
            All non-CMD_x replies read. This list will definitely contain
            the types listed in ``wait_reply_codes`` and may contain
            other replies as well.
        """
        cmd_reply_codes = frozenset(
            (
                MTMount.ReplyId.CMD_ACKNOWLEDGED,
                MTMount.ReplyId.CMD_REJECTED,
                MTMount.ReplyId.CMD_SUCCEEDED,
                MTMount.ReplyId.CMD_FAILED,
                MTMount.ReplyId.CMD_SUPERSEDED,
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
        if return_all_replies and other_reply_codes:
            raise ValueError("return_all_replies true, other_reply_codes must be empty")
        other_reply_codes = set(other_reply_codes)
        bad_nonack_codes = other_reply_codes & cmd_reply_codes
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
            elif return_all_replies or reply.id in other_reply_codes:
                non_cmd_replies.append(reply)
        if final_reply_code == MTMount.ReplyId.CMD_REJECTED:
            # Should fail before ack
            self.assertEqual(reply.id, MTMount.ReplyId.CMD_REJECTED)
            self.assertEqual(reply.sequenceId, command.sequence_id)
        else:
            self.assertEqual(reply.id, MTMount.ReplyId.CMD_ACKNOWLEDGED)
            self.assertEqual(reply.sequenceId, command.sequence_id)
            if final_reply_code == MTMount.ReplyId.CMD_ACKNOWLEDGED:
                # This command is done when acknowledged
                return non_cmd_replies

            while True:
                reply = await asyncio.wait_for(self.read_one_reply(), timeout=timeout)
                if reply.id in cmd_reply_codes:
                    break
                elif reply.id in nonack_reply_codes_remaining:
                    nonack_reply_codes_remaining.remove(reply.id)
                    non_cmd_replies.append(reply)
                elif return_all_replies or reply.id in other_reply_codes:
                    non_cmd_replies.append(reply)
            explanation = getattr(reply, "explanation", "")
            self.assertEqual(reply.id, final_reply_code, explanation)
            self.assertEqual(reply.sequenceId, command.sequence_id)
        if nonack_reply_codes_remaining:
            non_cmd_replies += await asyncio.wait_for(
                self.read_replies(
                    wait_reply_codes=nonack_reply_codes_remaining,
                    other_reply_codes=other_reply_codes,
                    return_all_replies=return_all_replies,
                ),
                timeout=timeout,
            )
        if return_all_replies or other_reply_codes:
            self.assertGreaterEqual(len(non_cmd_replies), len(wait_reply_codes))
        else:
            self.assertEqual(len(non_cmd_replies), len(wait_reply_codes))
        return non_cmd_replies

    async def telemetry_read_loop(self):
        """Read telemetry and add it to ``self.telemetry_dict``."""
        while True:
            try:
                data = await self.telemetry_reader.readuntil(MTMount.LINE_TERMINATOR)
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
        async with self.make_controller(commander=MTMount.Source.NONE):
            # Until AskForCommand is issued, all other commands should fail;
            # try a sampling of commands.
            sample_commands = (
                MTMount.commands.MirrorCoverLocksPower(drive=-1, on=True),
                MTMount.commands.AzimuthPower(on=True),
                MTMount.commands.ElevationPower(on=True),
            )
            for command in sample_commands:
                await self.run_command(
                    command=command,
                    final_reply_code=MTMount.ReplyId.CMD_REJECTED,
                    use_read_loop=True,
                )
            non_cmd_replies = await self.run_command(
                command=MTMount.commands.AskForCommand(commander=MTMount.Source.CSC),
                wait_reply_codes=[MTMount.ReplyId.COMMANDER],
                use_read_loop=True,
            )
            self.assertEqual(len(non_cmd_replies), 1)
            reply = non_cmd_replies[0]
            self.assertEqual(reply.id, MTMount.ReplyId.COMMANDER)
            self.assertEqual(reply.actualCommander, MTMount.Source.CSC)
            for command in sample_commands:
                await self.run_command(
                    command=command,
                    use_read_loop=True,
                )

    async def test_ask_for_command_rejected(self):
        async with self.make_controller(commander=MTMount.Source.HHD):
            # commander=HHD prevents assigning command to any other commander.
            sample_commands = (
                MTMount.commands.AskForCommand(commander=MTMount.Source.NONE),
                MTMount.commands.AskForCommand(commander=MTMount.Source.CSC),
                MTMount.commands.AskForCommand(commander=MTMount.Source.EUI),
                MTMount.commands.AskForCommand(),  # defaults to CSC
                MTMount.commands.MirrorCoverLocksPower(drive=-1, on=True),
                MTMount.commands.AzimuthPower(on=True),
                MTMount.commands.ElevationPower(on=True),
            )
            for command in sample_commands:
                await self.run_command(
                    command=command,
                    final_reply_code=MTMount.ReplyId.CMD_REJECTED,
                    use_read_loop=True,
                )
            # Asking for command by the HHD should work, though it is a no-op.
            await self.run_command(
                command=MTMount.commands.AskForCommand(commander=MTMount.Source.HHD),
                use_read_loop=True,
            )

    async def test_command_failed(self):
        async with self.make_controller():
            device = self.controller.device_dict[System.MIRROR_COVERS]
            await self.run_command(
                command=MTMount.commands.MirrorCoversPower(drive=-1, on=True),
                use_read_loop=True,
            )

            # Issue a background command that should fail
            # (after being acknowledged).
            # The mirror covers start deployed, so this will fail quickly.
            device.fail_next_command = True
            await self.run_command(
                command=MTMount.commands.MirrorCoversDeploy(drive=-1),
                use_read_loop=True,
                final_reply_code=MTMount.ReplyId.CMD_FAILED,
            )

    async def test_command_superseded(self):
        async with self.make_controller():
            device = self.controller.device_dict[System.MIRROR_COVERS]

            await self.run_command(
                command=MTMount.commands.MirrorCoversPower(drive=-1, on=True),
                use_read_loop=True,
            )

            # Issue a background command that should take some time
            # then supersede it.
            task = asyncio.create_task(
                self.run_command(
                    command=MTMount.commands.MirrorCoversRetract(drive=-1),
                    use_read_loop=True,  # False is also fine
                    final_reply_code=MTMount.ReplyId.CMD_SUPERSEDED,
                )
            )
            await asyncio.sleep(0.1)

            # Supersede the command by directly commanding the device,
            # because run_command is too primitive to handle
            # more than one command at a time.
            stop_command = MTMount.commands.MirrorCoversStop(drive=-1)
            device.do_stop(stop_command)
            await task

    async def test_handle_command(self):
        await self.check_command_sequence(use_read_loop=False)

    async def test_read_loop(self):
        await self.check_command_sequence(use_read_loop=True)

    async def test_state_info_command(self):
        # The STATE_INFO command is accepted regardless of the commander.

        async with self.make_controller(commander=MTMount.Source.HHD):
            # Wait for one iteration of telemetry,
            # to avoid duplicate message.
            await asyncio.wait_for(
                self.controller.wait_telemetry(), timeout=STD_TIMEOUT
            )

            replies = await self.run_command(
                command=MTMount.commands.StateInfo(),
                return_all_replies=True,
                use_read_loop=True,
            )
            limits_systems = []
            motion_state_axes = []
            chiller_state_systems = []
            motion_controller_state_systems = []
            power_state_systems = []
            for reply in replies:
                if reply.id == MTMount.ReplyId.AVAILABLE_SETTINGS:
                    self.assertEqual(
                        len(reply.sets), len(self.controller.available_settings)
                    )
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

                elif reply.id == MTMount.ReplyId.AXIS_MOTION_STATE:
                    motion_state_axes.append(reply.axis)
                    self.assertEqual(reply.motionState, AxisMotionState.STOPPED)
                elif reply.id == MTMount.ReplyId.AZIMUTH_TOPPLE_BLOCK:
                    self.assertEqual(reply.reverse, False)
                    self.assertEqual(reply.forward, False)
                elif reply.id == MTMount.ReplyId.CHILLER_STATE:
                    chiller_state_systems.append(reply.system)
                    nelts = self.controller.chiller_state_nelts[reply.system]
                    if nelts == 1:
                        desired_trackAmbient = True
                        desired_temperature = self.controller.ambient_temperature
                    else:
                        desired_trackAmbient = [True] * nelts
                        desired_temperature = [
                            self.controller.ambient_temperature
                        ] * nelts
                    self.assertEqual(reply.trackAmbient, desired_trackAmbient)
                    numpy.testing.assert_allclose(
                        reply.temperature, desired_temperature
                    )
                elif reply.id == MTMount.ReplyId.COMMANDER:
                    self.assertEqual(reply.actualCommander, MTMount.Source.HHD)
                elif reply.id == MTMount.ReplyId.DEPLOYABLE_PLATFORMS_MOTION_STATE:
                    self.assertEqual(reply.state, DeployableMotionState.RETRACTED)
                    self.assertEqual(
                        list(reply.elementState), [DeployableMotionState.RETRACTED] * 2
                    )
                elif reply.id == MTMount.ReplyId.ELEVATION_LOCKING_PIN_MOTION_STATE:
                    self.assertEqual(
                        reply.state, ElevationLockingPinMotionState.UNLOCKED
                    )
                    self.assertEqual(
                        list(reply.elementState),
                        [ElevationLockingPinMotionState.UNLOCKED] * 2,
                    )
                elif reply.id == MTMount.ReplyId.LIMITS:
                    limits_systems.append(reply.system)
                    self.assertEqual(reply.limits, 0)
                elif reply.id in (
                    MTMount.ReplyId.MIRROR_COVERS_MOTION_STATE,
                    MTMount.ReplyId.MIRROR_COVER_LOCKS_MOTION_STATE,
                ):
                    self.assertEqual(reply.state, DeployableMotionState.DEPLOYED)
                    self.assertEqual(
                        list(reply.elementState), [DeployableMotionState.DEPLOYED] * 4
                    )
                elif reply.id == MTMount.ReplyId.MOTION_CONTROLLER_STATE:
                    motion_controller_state_systems.append(reply.system)
                    nelts = self.controller.motion_controller_state_nelts[reply.system]
                    expected_state = PowerState.OFF
                    self.assertEqual(
                        reply.motionControllerState, [expected_state] * nelts
                    )
                elif reply.id == MTMount.ReplyId.POWER_STATE:
                    power_state_systems.append(reply.system)
                    nelts = self.controller.power_state_nelts[reply.system]
                    if reply.system in {
                        System.AZIMUTH_DRIVES_THERMAL,
                        System.ELEVATION_DRIVES_THERMAL,
                        System.AZ0101_CABINET_THERMAL,
                        System.MODBUS_TEMPERATURE_CONTROLLERS,
                        System.MAIN_CABINET,
                        System.MAIN_AXES_POWER_SUPPLY,
                    }:
                        expected_state = PowerState.ON
                    else:
                        expected_state = PowerState.OFF
                    self.assertEqual(reply.powerState, expected_state)
                    if nelts > 1:
                        self.assertEqual(
                            reply.elementsPowerState, [expected_state] * nelts
                        )
                    else:
                        self.assertFalse(hasattr(reply, "elementsPowerState"))

                elif reply.id == MTMount.ReplyId.SAFETY_INTERLOCKS:
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
                    self.assertEqual(set(expected_fields), params_dict.keys())
                    for field in expected_fields:
                        self.assertEqual(params_dict[field], 0)

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
            self.assertFalse(device.power_on)
            on_command = MTMount.commands.MirrorCoverLocksPower(drive=-1, on=True)
            await self.run_command(command=on_command, use_read_loop=use_read_loop)
            self.assertTrue(device.power_on)

            # Issue a background command
            self.assertAlmostEqual(device.actuator.position(), device.deployed_position)
            deploy_command = MTMount.commands.MirrorCoverLocksMoveAll(
                drive=-1, deploy=False
            )
            await self.run_command(command=deploy_command, use_read_loop=use_read_loop)
            self.assertAlmostEqual(
                device.actuator.position(), device.retracted_position
            )

            # Issue a command (AzimuthTrack) that gets no Done reply
            # but first enable the device and tracking
            device = self.controller.device_dict[System.AZIMUTH]
            self.assertFalse(device.power_on)
            self.assertFalse(device.enabled)
            self.assertFalse(device.tracking_enabled)
            power_on_command = MTMount.commands.AzimuthPower(on=True)
            await self.run_command(
                command=power_on_command, use_read_loop=use_read_loop
            )
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertFalse(device.tracking_enabled)
            enable_tracking_command = MTMount.commands.AzimuthEnableTracking()
            await self.run_command(
                command=enable_tracking_command, use_read_loop=use_read_loop
            )
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertTrue(device.tracking_enabled)
            track_command = MTMount.commands.AzimuthTrack(
                position=45, velocity=0, tai=salobj.current_tai()
            )
            await self.run_command(
                command=track_command,
                final_reply_code=MTMount.ReplyId.CMD_ACKNOWLEDGED,
                use_read_loop=use_read_loop,
            )
            # Issue one more command to be sure we really didn't get
            # a Done reply for the previous command
            stop_tracking_command = MTMount.commands.AzimuthStop()
            await self.run_command(
                command=stop_tracking_command, use_read_loop=use_read_loop
            )
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertFalse(device.tracking_enabled)

            # Try a command that will fail
            # (tracking while tracking not enabled)
            await self.run_command(
                command=track_command,
                final_reply_code=MTMount.ReplyId.CMD_REJECTED,
                use_read_loop=use_read_loop,
            )

            # Try a command not supported by the mock controller.
            # Add it to MTMount.commands.CommandDict so the mock controller
            # can parse it as a command.
            unsupported_command = UnsupportedCommand()
            self.assertNotIn(
                unsupported_command.command_code, MTMount.commands.CommandDict
            )
            new_command_dict = MTMount.commands.CommandDict.copy()
            new_command_dict[unsupported_command.command_code] = UnsupportedCommand
            with unittest.mock.patch(
                "lsst.ts.MTMount.commands.CommandDict", new_command_dict
            ):
                await self.run_command(
                    command=unsupported_command,
                    final_reply_code=MTMount.ReplyId.CMD_REJECTED,
                    use_read_loop=use_read_loop,
                )

    async def next_telemetry(self, topic_id, timeout=STD_TIMEOUT):
        """Wait for a new instance of the specified telemetry topic."""
        topic_id = MTMount.TelemetryTopicId(topic_id)
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
            self.assertIsNone(self.controller.command_queue)

            # Create the command queue with specified maxsize.
            maxsize = 47
            self.controller.set_command_queue(maxsize=maxsize)
            self.assertIsInstance(self.controller.command_queue, asyncio.Queue)
            self.assertEqual(self.controller.command_queue.maxsize, maxsize)

            # Replace the command queue with default maxsize (unlimited).
            self.controller.set_command_queue()
            self.assertIsInstance(self.controller.command_queue, asyncio.Queue)
            self.assertEqual(self.controller.command_queue.maxsize, 0)

            # Run two commands.
            device = self.controller.device_dict[System.MIRROR_COVER_LOCKS]
            self.assertFalse(device.power_on)
            on_command = MTMount.commands.MirrorCoverLocksPower(drive=-1, on=True)
            await self.run_command(command=on_command, use_read_loop=True)
            self.assertTrue(device.power_on)
            off_command = MTMount.commands.MirrorCoverLocksPower(drive=-1, on=False)
            await self.run_command(command=off_command, use_read_loop=True)
            self.assertFalse(device.power_on)

            # Check the two commands on the command queue.
            queued_on_command = await asyncio.wait_for(
                self.controller.command_queue.get(), timeout=STD_TIMEOUT
            )
            self.assertEqual(queued_on_command, on_command)
            queued_off_command = await asyncio.wait_for(
                self.controller.command_queue.get(), timeout=STD_TIMEOUT
            )
            self.assertEqual(queued_off_command, off_command)
            self.assertNotEqual(on_command, off_command)
            self.assertTrue(self.controller.command_queue.empty())

            # Delete the command queue.
            self.controller.delete_command_queue()
            self.assertIsNone(self.controller.command_queue)

    async def test_initial_telemetry(self):
        async with self.make_controller():
            for topic_id, prefix in (
                (MTMount.TelemetryTopicId.AZIMUTH, "az"),
                (MTMount.TelemetryTopicId.ELEVATION, "el"),
            ):
                # Elevation does not start at position 0,
                # so read the position.
                system_id = {
                    MTMount.TelemetryTopicId.AZIMUTH: System.AZIMUTH,
                    MTMount.TelemetryTopicId.ELEVATION: System.ELEVATION,
                }[topic_id]
                device = self.controller.device_dict[system_id]
                # Work around Docker time issues on macOS with an offset
                tai0 = salobj.current_tai() - 0.1
                axis_telem = await self.next_telemetry(topic_id)
                for name in (
                    "angleActual",
                    "angleSet",
                    "velocityActual",
                    "velocitySet",
                    "accelerationActual",
                    "torqueActual",
                ):
                    if name.startswith("angle"):
                        desired_value = device.actuator.path.at(tai0).position
                    else:
                        desired_value = 0
                    self.assertAlmostEqual(axis_telem[name], desired_value, msg=name)
                self.assertGreater(axis_telem["timestamp"], tai0)

            # Work around Docker time issues on macOS with an offset
            tai0 = salobj.current_tai() - 0.1
            ccw_telem = await self.next_telemetry(
                MTMount.TelemetryTopicId.CAMERA_CABLE_WRAP
            )
            for name in (
                "angle",
                "speed",
                "acceleration",
            ):
                self.assertEqual(ccw_telem[name], 0, msg=name)
            self.assertGreater(ccw_telem["timestamp"], tai0)

    async def test_tracking(self):
        """Test the <axis>AxisTrack command and InPosition replies."""
        async with self.make_controller():
            replies = await asyncio.wait_for(
                self.read_replies(
                    wait_reply_codes=[MTMount.ReplyId.IN_POSITION] * 2,
                ),
                timeout=START_TIMEOUT,
            )
            for reply in replies:
                self.assertFalse(reply.inPosition)
                self.assertIn(reply.axis, (0, 1))

            device = self.controller.device_dict[System.AZIMUTH]
            power_on_command = MTMount.commands.AzimuthPower(on=True)
            await self.run_command(command=power_on_command, use_read_loop=True)
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertFalse(device.tracking_enabled)
            enable_tracking_command = MTMount.commands.AzimuthEnableTracking()
            await self.run_command(command=enable_tracking_command, use_read_loop=True)
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertTrue(device.tracking_enabled)

            start_position = device.actuator.path.at(salobj.current_tai()).position
            end_position = start_position + 3
            previous_tai = 0
            # Send tracking updates until an IN_POSITION reply is seen.
            while True:
                tai = salobj.current_tai()
                # Work around non-monotonic clocks, which are
                # sometimes seen when running Docker on macOS.
                if tai <= previous_tai:
                    tai = previous_tai + 0.001
                track_command = MTMount.commands.AzimuthTrack(
                    position=end_position,
                    velocity=0,
                    tai=tai,
                )
                non_cmd_replies = await self.run_command(
                    command=track_command,
                    final_reply_code=MTMount.ReplyId.CMD_ACKNOWLEDGED,
                    flush=False,
                    use_read_loop=True,
                    other_reply_codes=[MTMount.ReplyId.IN_POSITION],
                )
                if non_cmd_replies:
                    self.assertEqual(len(non_cmd_replies), 1)
                    reply = non_cmd_replies[0]
                    self.assertEqual(reply.id, MTMount.ReplyId.IN_POSITION)
                    self.assertEqual(reply.axis, 0)
                    self.assertTrue(reply.inPosition)
                    break
                previous_tai = tai
                await asyncio.sleep(0.1)

            stop_tracking_command = MTMount.commands.AzimuthStop()
            non_cmd_replies = await self.run_command(
                command=stop_tracking_command,
                use_read_loop=True,
                wait_reply_codes=[MTMount.ReplyId.IN_POSITION],
            )
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertFalse(device.tracking_enabled)
            self.assertEqual(len(non_cmd_replies), 1)
            reply = non_cmd_replies[0]
            self.assertEqual(reply.id, MTMount.ReplyId.IN_POSITION)
            self.assertFalse(reply.inPosition)
            self.assertEqual(reply.axis, 0)

    async def test_move(self):
        """Test the <axis>AxisMove command and InPosition replies."""
        async with self.make_controller():

            n_in_position = len(self.controller.in_position_dict)
            n_motion_controller_state = len(
                self.controller.motion_controller_state_dict
            )
            n_axis_motion_state = len(self.controller.axis_motion_state_dict)
            n_power_state = len(self.controller.power_state_dict)
            replies = await asyncio.wait_for(
                self.read_replies(
                    wait_reply_codes=[MTMount.ReplyId.IN_POSITION] * n_in_position
                    + [MTMount.ReplyId.MOTION_CONTROLLER_STATE]
                    * n_motion_controller_state
                    + [MTMount.ReplyId.POWER_STATE] * n_power_state
                    + [MTMount.ReplyId.AXIS_MOTION_STATE] * n_axis_motion_state,
                ),
                timeout=START_TIMEOUT,
            )
            for reply in replies:
                if reply.id == MTMount.ReplyId.IN_POSITION:
                    assert reply.axis in self.controller.axis_motion_state_dict
                    self.assertFalse(reply.inPosition)
                elif reply.id == MTMount.ReplyId.MOTION_CONTROLLER_STATE:
                    assert (
                        reply.system in self.controller.motion_controller_state_dict
                        or reply.system == System.AZIMUTH_CABLE_WRAP
                    )
                    nelts = self.controller.motion_controller_state_nelts[reply.system]
                    assert reply.motionControllerState == [PowerState.OFF] * nelts
                elif reply.id == MTMount.ReplyId.POWER_STATE:
                    assert (
                        reply.system in self.controller.power_state_dict
                        or reply.system == System.AZIMUTH_CABLE_WRAP
                    )
                    assert reply.powerState == PowerState.OFF
                elif reply.id == MTMount.ReplyId.AXIS_MOTION_STATE:
                    assert reply.axis in self.controller.axis_motion_state_dict
                    assert reply.motionState == AxisMotionState.STOPPED

            device = self.controller.device_dict[System.ELEVATION]
            power_on_command = MTMount.commands.ElevationPower(on=True)
            non_cmd_replies = await self.run_command(
                command=power_on_command,
                use_read_loop=True,
                wait_reply_codes=[
                    MTMount.ReplyId.MOTION_CONTROLLER_STATE,
                    MTMount.ReplyId.POWER_STATE,
                ],
            )
            assert len(non_cmd_replies) == 2
            assert len(set(reply.id for reply in non_cmd_replies)) == 2
            for reply in non_cmd_replies:
                if reply.id == MTMount.ReplyId.MOTION_CONTROLLER_STATE:
                    nelts = self.controller.motion_controller_state_nelts[
                        System.ELEVATION
                    ]
                    assert reply.motionControllerState == [PowerState.ON] * nelts
                elif reply.id == MTMount.ReplyId.POWER_STATE:
                    assert reply.powerState == PowerState.ON
                    assert not hasattr(reply, "elementsPowerState")
                else:
                    raise AssertionError(f"unrecognized reply ID {reply['id']}")
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertFalse(device.tracking_enabled)

            start_position = device.actuator.path.at(salobj.current_tai()).position
            end_position = start_position + 1
            move_command = MTMount.commands.ElevationMove(position=end_position)
            estimated_move_time = 2  # seconds
            t0 = time.monotonic()
            non_cmd_replies = await self.run_command(
                command=move_command,
                use_read_loop=True,
                timeout=estimated_move_time + STD_TIMEOUT,
                wait_reply_codes=[MTMount.ReplyId.IN_POSITION],
            )
            dt = time.monotonic() - t0
            print(f"Move duration={dt:0.2f} second")
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertFalse(device.tracking_enabled)
            self.assertEqual(len(non_cmd_replies), 1)
            reply = non_cmd_replies[0]
            self.assertTrue(reply.inPosition)
            self.assertEqual(reply.axis, 1)

            # Test telemetry after move
            axis_telem = await self.next_telemetry(MTMount.TelemetryTopicId.ELEVATION)
            for name in (
                "angleActual",
                "angleSet",
                "velocityActual",
                "velocitySet",
                "accelerationActual",
                "torqueActual",
            ):
                if name.startswith("angle"):
                    desired_value = end_position
                else:
                    desired_value = 0
                self.assertAlmostEqual(axis_telem[name], desired_value, msg=name)


if __name__ == "__main__":
    unittest.main()
