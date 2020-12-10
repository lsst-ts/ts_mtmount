# This file is part of ts_ATDomeTrajectory.
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
import logging
import time
import unittest

import asynctest

from lsst.ts import salobj
from lsst.ts import MTMount

START_TIME = 20  # Time for startup (sec)
STD_TIMEOUT = 2  # Timeout for short operations (sec)
# Padding for the time limit returned by device do_methods
TIMEOUT_PADDING = 2

logging.basicConfig()

port_generator = salobj.index_generator(imin=3000)


UNSUPPORTED_COMMAND_CODE = (
    MTMount.enums.CommandCode.TRANSFER_FUNCTION_AZIMUTH_EXCITATION
)


class UnsupportedCommand(MTMount.commands.Command):
    field_infos = MTMount.commands.make_command_field_infos(UNSUPPORTED_COMMAND_CODE)


class MockControllerTestCase(asynctest.TestCase):
    def setUp(self):
        self.assertNotIn(UNSUPPORTED_COMMAND_CODE, MTMount.commands.CommandDict)
        MTMount.commands.CommandDict[UNSUPPORTED_COMMAND_CODE] = UnsupportedCommand

    def tearDown(self):
        del MTMount.commands.CommandDict[UNSUPPORTED_COMMAND_CODE]

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
        command_port = next(port_generator)
        self.communicator = MTMount.Communicator(
            name="communicator",
            client_host=salobj.LOCAL_HOST,
            client_port=command_port,
            server_host=salobj.LOCAL_HOST,
            # Tekniker uses repy port = command port + 1
            server_port=command_port + 1,
            log=log,
            read_replies=True,
            connect=False,
            connect_callback=None,
        )
        self.controller = MTMount.mock.Controller(
            command_port=command_port, log=log, commander=commander
        )
        connect_task = asyncio.create_task(self.communicator.connect())
        t0 = time.monotonic()
        await asyncio.wait_for(
            asyncio.gather(self.controller.start_task, connect_task), timeout=START_TIME
        )
        dt = time.monotonic() - t0
        print(f"Time to start up: {dt:0.2f} sec")
        try:
            yield
        finally:
            await self.communicator.close()
            await self.controller.close()

    async def read_replies(self, reply_types, return_others, timeout=STD_TIMEOUT):
        """Wait for one or more replies of the specified types.

        Return a list of the all replies read.

        Parameters
        ----------

        reply_types : `list` [`MTMount.Reply`]
            Types of replies to wait for.
            If a type is listed N times then wait for N such replies.
        return_others : `bool`
            Return other replies, in addition to those specified in
            ``reply_types``?
        timeout : `float`
            Timeout waiting for the Done reply (second).
            Ignored for other reads.

        Returns
        -------
        replies : `List` [`MTMount.Reply`]
            The read replies.
        """
        reply_types_remaining = reply_types.copy()
        replies = []
        while reply_types_remaining:
            reply = await asyncio.wait_for(self.communicator.read(), timeout=timeout)
            if type(reply) in reply_types_remaining:
                reply_types_remaining.remove(type(reply))
                replies.append(reply)
            elif return_others:
                replies.append(reply)
        return replies

    async def run_command(
        self,
        command,
        use_read_loop,
        should_fail=False,
        read_done=True,
        noack_reply_types=(),
        return_others=False,
        timeout=STD_TIMEOUT,
    ):
        """Run one command and check the replies.

        Return a list of non-Ack/Done/NoAck replies.

        Parameters
        ----------
        command : `MTMount.Command`
            Command to send.
        use_read_loop : `bool`
            If True then send the command via the communicator.
            If False then call self.controller.handle_command directly.
        should_fail : `bool`
            Should the command fail before the first ask?
            If True then ``read_done``is ignored.
        read_done : `bool`
            Should the command get a DoneReply reply?
            Most of them do, but a few do not.
        noack_reply_types : `list` [`MTMount.Reply`]
            Types of additional noack replies to wait for.
            If a type is listed N times then wait for N such replies.
            Any such replies found are returned in a list.
            This list must not contain `AckReply`, `DoneReply`,
            or `NoAckReply`.
        return_others : `bool`
            If True return all noack replies seen.
            If False return only those listed in ``noack_reply_types``.
        timeout : `float`
            Timeout waiting for the Done reply (second).
            Ignored for other reads.

        Returns
        -------
        nonack_replies : `List` [`MTMount.Reply`]
            All non-ack replies read. This list will definitely contain
            the types listed in ``noack_reply_types`` and may contain
            other replies as well.
        """
        ack_reply_types = set(
            (
                MTMount.replies.AckReply,
                MTMount.replies.DoneReply,
                MTMount.replies.NoAckReply,
            )
        )

        reply_types_remaining = list(noack_reply_types)
        for reply_type in noack_reply_types:
            if reply_type in ack_reply_types:
                raise ValueError(
                    f"noack_reply_types={noack_reply_types} contains "
                    f"{reply_type}, which is an ack reply"
                )

        if use_read_loop:
            await self.communicator.write(command)
        else:
            await self.controller.handle_command(command)
        nonack_replies = list()
        reply = None
        while reply is None or type(reply) not in ack_reply_types:
            if reply is None:
                pass
            elif type(reply) in reply_types_remaining:
                reply_types_remaining.remove(type(reply))
                nonack_replies.append(reply)
            elif return_others:
                nonack_replies.append(reply)
            reply = await asyncio.wait_for(
                self.communicator.read(), timeout=STD_TIMEOUT
            )
        if should_fail:
            self.assertIsInstance(reply, MTMount.replies.NoAckReply)
            self.assertEqual(reply.sequence_id, command.sequence_id)
        else:
            self.assertIsInstance(reply, MTMount.replies.AckReply)
            self.assertEqual(reply.sequence_id, command.sequence_id)
            if read_done:
                reply = None
                while reply is None or type(reply) not in ack_reply_types:
                    if reply is None:
                        pass
                    elif type(reply) in reply_types_remaining:
                        reply_types_remaining.remove(type(reply))
                        nonack_replies.append(reply)
                    elif return_others:
                        nonack_replies.append(reply)
                    reply = await asyncio.wait_for(
                        self.communicator.read(), timeout=timeout
                    )
                self.assertIsInstance(reply, MTMount.replies.DoneReply)
                self.assertEqual(reply.sequence_id, command.sequence_id)
        if reply_types_remaining:
            nonack_replies += await self.read_replies(
                reply_types_remaining, timeout=timeout, return_others=True
            )
        if return_others:
            self.assertGreaterEqual(len(nonack_replies), len(noack_reply_types))
        else:
            self.assertEqual(len(nonack_replies), len(noack_reply_types))
        return nonack_replies

    async def test_ask_for_command_ok(self):
        async with self.make_controller(commander=MTMount.Source.NONE):
            # Until AskForCommand is issued, all other commands should fail;
            # try a sampling of commands.
            sample_commands = (
                MTMount.commands.MirrorCoverLocksPower(drive=-1, on=True),
                MTMount.commands.AzimuthAxisPower(on=True),
                MTMount.commands.ElevationAxisPower(on=True),
            )
            for command in sample_commands:
                await self.run_command(command, should_fail=True, use_read_loop=True)
            await self.run_command(MTMount.commands.AskForCommand(), use_read_loop=True)
            for command in sample_commands:
                await self.run_command(command, use_read_loop=True)

    async def test_ask_for_command_fail(self):
        async with self.make_controller(commander=MTMount.Source.HHD):
            # commander=HHD prevents assigning command to any other commander.
            sample_commands = (
                MTMount.commands.AskForCommand(commander=MTMount.Source.NONE),
                MTMount.commands.AskForCommand(commander=MTMount.Source.CSC),
                MTMount.commands.AskForCommand(commander=MTMount.Source.EUI),
                MTMount.commands.AskForCommand(),  # defaults to CSC
                MTMount.commands.MirrorCoverLocksPower(drive=-1, on=True),
                MTMount.commands.AzimuthAxisPower(on=True),
                MTMount.commands.ElevationAxisPower(on=True),
            )
            for command in sample_commands:
                await self.run_command(command, should_fail=True, use_read_loop=True)
            # Asking for command by the HHD should work, though it is a no-op.
            await self.run_command(
                MTMount.commands.AskForCommand(commander=MTMount.Source.HHD),
                use_read_loop=True,
            )

    async def test_read_loop(self):
        await self.check_command_sequence(use_read_loop=True)

    async def test_handle_command(self):
        await self.check_command_sequence(use_read_loop=False)

    async def check_command_sequence(self, use_read_loop):
        async with self.make_controller():
            device = self.controller.device_dict[MTMount.DeviceId.MIRROR_COVER_LOCKS]

            # Issue a synchronous command
            self.assertFalse(device.power_on)
            on_command = MTMount.commands.MirrorCoverLocksPower(drive=-1, on=True)
            await self.run_command(on_command, use_read_loop=use_read_loop)
            self.assertTrue(device.power_on)

            # Issue a background command
            self.assertAlmostEqual(
                device.actuator.position(), device.actuator.min_position
            )
            deploy_command = MTMount.commands.MirrorCoverLocksMoveAll(
                drive=-1, deploy=True
            )
            await self.run_command(deploy_command, use_read_loop=use_read_loop)
            self.assertAlmostEqual(
                device.actuator.position(), device.actuator.max_position
            )

            # Issue a command (AzimuthAxisTrack) that gets no Done reply
            # but first enable the device and tracking
            device = self.controller.device_dict[MTMount.DeviceId.AZIMUTH_AXIS]
            self.assertFalse(device.power_on)
            self.assertFalse(device.enabled)
            self.assertFalse(device.tracking_enabled)
            power_on_command = MTMount.commands.AzimuthAxisPower(on=True)
            await self.run_command(power_on_command, use_read_loop=use_read_loop)
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertFalse(device.tracking_enabled)
            enable_tracking_command = MTMount.commands.AzimuthAxisEnableTracking(
                on=True
            )
            await self.run_command(enable_tracking_command, use_read_loop=use_read_loop)
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertTrue(device.tracking_enabled)
            track_command = MTMount.commands.AzimuthAxisTrack(
                position=45, velocity=0, tai=salobj.current_tai()
            )
            await self.run_command(
                track_command, read_done=False, use_read_loop=use_read_loop
            )
            # Issue one more command to be sure we really didn't get
            # a Done reply for the previous command
            disable_tracking_command = MTMount.commands.AzimuthAxisEnableTracking(
                on=False
            )
            await self.run_command(
                disable_tracking_command, use_read_loop=use_read_loop
            )
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertFalse(device.tracking_enabled)

            # Try a command that will fail
            # (tracking while tracking not enabled)
            await self.run_command(
                track_command, should_fail=True, use_read_loop=use_read_loop
            )

            # Try an unsupported command
            unsupported_command = UnsupportedCommand()
            await self.run_command(
                unsupported_command, should_fail=True, use_read_loop=use_read_loop
            )

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
            device = self.controller.device_dict[MTMount.DeviceId.MIRROR_COVER_LOCKS]
            self.assertFalse(device.power_on)
            on_command = MTMount.commands.MirrorCoverLocksPower(drive=-1, on=True)
            await self.run_command(on_command, use_read_loop=True)
            self.assertTrue(device.power_on)
            off_command = MTMount.commands.MirrorCoverLocksPower(drive=-1, on=False)
            await self.run_command(off_command, use_read_loop=True)
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

    async def test_tracking(self):
        """Test the <axis>AxisTrack command and InPosition replies.
        """
        async with self.make_controller():
            replies = await self.read_replies(
                reply_types=[MTMount.replies.InPositionReply] * 2,
                timeout=10,
                return_others=False,
            )
            for reply in replies:
                self.assertFalse(reply.in_position)
                self.assertIn(reply.what, (0, 1))

            device = self.controller.device_dict[MTMount.DeviceId.AZIMUTH_AXIS]
            power_on_command = MTMount.commands.AzimuthAxisPower(on=True)
            await self.run_command(power_on_command, use_read_loop=True)
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertFalse(device.tracking_enabled)
            enable_tracking_command = MTMount.commands.AzimuthAxisEnableTracking(
                on=True
            )
            await self.run_command(enable_tracking_command, use_read_loop=True)
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertTrue(device.tracking_enabled)

            start_position = device.actuator.path.at(salobj.current_tai()).position
            end_position = start_position + 3
            previous_tai = 0
            # Send tracking updates until an InPosition reply is seen.
            while True:
                tai = salobj.current_tai()
                # Work around non-monotonic clocks, which are
                # sometimes seen when running Docker on macOS.
                if tai <= previous_tai:
                    tai = previous_tai + 0.001
                track_command = MTMount.commands.AzimuthAxisTrack(
                    position=end_position, velocity=0, tai=tai,
                )
                nonack_replies = await self.run_command(
                    track_command,
                    read_done=False,
                    use_read_loop=True,
                    return_others=True,
                )
                if nonack_replies:
                    self.assertGreaterEqual(len(nonack_replies), 1)
                    num_in_position_replies = 0
                    for reply in nonack_replies:
                        if isinstance(reply, MTMount.replies.InPositionReply):
                            self.assertEqual(reply.what, 0)
                            self.assertTrue(reply.in_position)
                            num_in_position_replies += 1
                    self.assertEqual(num_in_position_replies, 1)
                    break
                previous_tai = tai
                await asyncio.sleep(0.1)

            disable_tracking_command = MTMount.commands.AzimuthAxisEnableTracking(
                on=False
            )
            nonack_replies = await self.run_command(
                disable_tracking_command,
                use_read_loop=True,
                noack_reply_types=[MTMount.replies.InPositionReply],
                return_others=False,
            )
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertFalse(device.tracking_enabled)
            self.assertEqual(len(nonack_replies), 1)
            reply = nonack_replies[0]
            self.assertIsInstance(reply, MTMount.replies.InPositionReply)
            self.assertFalse(reply.in_position)
            self.assertEqual(reply.what, 0)

    async def test_move(self):
        """Test the <axis>AxisMove command and InPosition replies.
        """
        async with self.make_controller():
            replies = await self.read_replies(
                reply_types=[MTMount.replies.InPositionReply] * 2,
                timeout=10,
                return_others=False,
            )
            self.assertEqual(len(replies), 2)
            for reply in replies:
                self.assertFalse(reply.in_position)
                self.assertIn(reply.what, (0, 1))

            device = self.controller.device_dict[MTMount.DeviceId.ELEVATION_AXIS]
            power_on_command = MTMount.commands.ElevationAxisPower(on=True)
            await self.run_command(power_on_command, use_read_loop=True)
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertFalse(device.tracking_enabled)

            start_position = device.actuator.path.at(salobj.current_tai()).position
            end_position = start_position + 1
            move_command = MTMount.commands.ElevationAxisMove(position=end_position)
            estimated_move_time = 2  # seconds
            t0 = time.monotonic()
            nonack_replies = await self.run_command(
                move_command,
                use_read_loop=True,
                timeout=estimated_move_time + STD_TIMEOUT,
                noack_reply_types=[MTMount.replies.InPositionReply],
                return_others=False,
            )
            dt = time.monotonic() - t0
            print(f"Move duration={dt:0.2f} second")
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertFalse(device.tracking_enabled)
            self.assertEqual(len(nonack_replies), 1)
            reply = nonack_replies[0]
            self.assertTrue(reply.in_position)
            self.assertEqual(reply.what, 1)


if __name__ == "__main__":
    unittest.main()
