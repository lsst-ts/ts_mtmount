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

import asyncio
import pathlib
import logging

import asynctest

from lsst.ts import salobj
from lsst.ts import MTMount

STD_TIMEOUT = 2  # standard command timeout (sec)
LONG_TIMEOUT = 20  # timeout for starting SAL components (sec)
TEST_CONFIG_DIR = pathlib.Path(__file__).parents[1] / "tests" / "data" / "config"

port_generator = salobj.index_generator(imin=3200)

logging.basicConfig()


class CscTestCase(salobj.BaseCscTestCase, asynctest.TestCase):
    def basic_make_csc(self, initial_state, config_dir, simulation_mode):
        return MTMount.MTMountCsc(
            initial_state=initial_state,
            config_dir=config_dir,
            simulation_mode=simulation_mode,
            mock_reply_port=next(port_generator),
        )
        # the next port is used for commands
        next(port_generator)

    async def test_standard_state_transitions(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await self.check_standard_state_transitions(
                enabled_commands=(
                    "closeMirrorCovers",
                    "openMirrorCovers",
                    "disableCameraCableWrapTracking",
                    "enableCameraCableWrapTracking",
                    "moveToTarget",
                    "trackTarget",
                    "stop",
                )
            )

    async def next_lowlevel_command(self, timeout=STD_TIMEOUT):
        """Get the next low level command.
        """
        return await asyncio.wait_for(
            self.csc.mock_controller.command_queue.get(), timeout=timeout
        )

    async def test_camera_cable_wrap_tracking(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(
                remote=self.remote, state=salobj.State.ENABLED
            )
            self.csc.mock_controller.set_command_queue(maxsize=0)
            async with salobj.Controller(name="Rotator") as rotator:
                self.assertTrue(self.csc.mock_controller.command_queue.empty())

                # Check that rotator targets are ignored before
                # tracking is enabled.
                for i in range(2):
                    rotator.tel_Application.set_put(Position=45)
                    await asyncio.sleep(0.1)
                self.assertTrue(self.csc.mock_controller.command_queue.empty())

                # Enable camera cable wrap tracking
                await self.remote.cmd_enableCameraCableWrapTracking.start(
                    timeout=STD_TIMEOUT
                )
                command = await self.next_lowlevel_command()
                self.assertEqual(
                    command.command_code,
                    MTMount.CommandCode.CAMERA_CABLE_WRAP_ENABLE_TRACK_CAMERA,
                )
                self.assertTrue(command.on)
                self.assertTrue(self.csc.mock_controller.command_queue.empty())

                position0 = 45
                velocity = 0.1
                tai_time0 = MTMount.get_tai_time()
                num_rotator_samples = 10
                prev_position = None
                prev_tai_time = None
                prev_delay = None
                for i in range(num_rotator_samples):
                    tai_time = MTMount.get_tai_time()
                    dt = tai_time - tai_time0
                    position = position0 + velocity * dt.sec
                    rotator.tel_Application.set_put(Position=position)
                    command = await self.next_lowlevel_command()
                    delay = MTMount.get_tai_time() - tai_time

                    self.assertEqual(
                        command.command_code,
                        MTMount.CommandCode.CAMERA_CABLE_WRAP_TRACK_CAMERA,
                    )
                    self.assertLessEqual(command.tai_time - tai_time, delay)
                    self.assertAlmostEqual(command.position, position)
                    if i == 0:
                        self.assertAlmostEqual(command.velocity, 0)
                    else:
                        nominal_dt = tai_time - prev_tai_time
                        min_dt = nominal_dt - delay - prev_delay
                        max_dt = nominal_dt + delay + prev_delay
                        min_vel = (position - prev_position) / max_dt.sec
                        max_vel = (position - prev_position) / min_dt.sec
                        max_vel_error = max_vel - min_vel
                        self.assertAlmostEqual(
                            command.velocity, velocity, delta=max_vel_error
                        )
                    await asyncio.sleep(0.1)
                    prev_position = position
                    prev_tai_time = tai_time
                    prev_delay = delay

                await self.remote.cmd_disableCameraCableWrapTracking.start(
                    timeout=STD_TIMEOUT
                )
                command = await self.next_lowlevel_command()
                self.assertEqual(
                    command.command_code,
                    MTMount.CommandCode.CAMERA_CABLE_WRAP_ENABLE_TRACK_CAMERA,
                )
                self.assertFalse(command.on)
                self.assertTrue(self.csc.mock_controller.command_queue.empty())

                # Check that rotator targets are ignored after
                # tracking is disabled.
                for i in range(2):
                    rotator.tel_Application.set_put(Position=45)
                    await asyncio.sleep(0.1)
                self.assertTrue(self.csc.mock_controller.command_queue.empty())
