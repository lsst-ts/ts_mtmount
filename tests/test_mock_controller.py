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

START_TIME = 5  # Time for startup (sec)
STD_TIMEOUT = 2  # Timeout for short operations (sec)
# Padding for the time limit returned by device do_methods
TIMEOUT_PADDING = 2

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
    async def make_controller(self):
        log = logging.getLogger()
        reply_port = next(port_generator)
        self.communicator = MTMount.Communicator(
            name="communicator",
            client_host=salobj.LOCAL_HOST,
            client_port=reply_port + 1,
            server_host=salobj.LOCAL_HOST,
            server_port=reply_port,
            log=log,
            read_replies=True,
            connect_client=False,
            connect_callback=None,
        )
        self.controller = MTMount.mock.Controller(reply_port=reply_port, log=log)
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

    async def run_command(
        self, command, use_read_loop, should_fail=False, read_done=True
    ):
        """Run one command and check the replies.

        Parameters
        ----------
        command : `MTMount.Command`
            Command to send.
        use_read_loop : `bool`
            If True then send the command via the communicator.
            If False then call self.controller.handle_command directly.
        should_fail : `bool`
            Should the command fail before the first ask?
            If True then read_done is ignored.
        read_done : `bool`
            Should the command get a DoneReply reply?
            Most of them do, but a few do not.
        """
        if use_read_loop:
            await self.communicator.write(command)
        else:
            await self.controller.handle_command(command)
        reply = await asyncio.wait_for(self.communicator.read(), timeout=STD_TIMEOUT)
        if should_fail:
            self.assertIsInstance(reply, MTMount.replies.NoAckReply)
            self.assertEqual(reply.sequence_id, command.sequence_id)
            return

        self.assertIsInstance(reply, MTMount.replies.AckReply)
        self.assertEqual(reply.sequence_id, command.sequence_id)
        if read_done:
            reply = await asyncio.wait_for(
                self.communicator.read(), timeout=STD_TIMEOUT
            )
            self.assertIsInstance(reply, MTMount.replies.DoneReply)
            self.assertEqual(reply.sequence_id, command.sequence_id)

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
                device.actuator.current_position, device.actuator.min_position
            )
            deploy_command = MTMount.commands.MirrorCoverLocksMoveAll(
                drive=-1, deploy=True
            )
            await self.run_command(deploy_command, use_read_loop=use_read_loop)
            self.assertAlmostEqual(
                device.actuator.current_position, device.actuator.max_position
            )

            # Issue a command (AzimuthAxisTrack) that gets no Done reply
            # but first enable the device and tracking
            device = self.controller.device_dict[MTMount.DeviceId.AZIMUTH_AXIS]
            self.assertFalse(device.power_on)
            self.assertFalse(device.enabled)
            self.assertFalse(device.tracking_enabled)
            enable_command = MTMount.commands.AzimuthAxisDriveEnable(drive=-1, on=True)
            await self.run_command(enable_command, use_read_loop=use_read_loop)
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
                position=45, velocity=0, tai_time=MTMount.get_tai_time()
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


if __name__ == "__main__":
    unittest.main()
