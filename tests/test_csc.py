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
import time

import asynctest

from lsst.ts import salobj
from lsst.ts import MTMount

STD_TIMEOUT = 2  # standard command timeout (sec)
MIRROR_COVER_TIMEOUT = 5  # timeout for opening or closing mirror covers (sec)
TEST_CONFIG_DIR = pathlib.Path(__file__).parents[1] / "tests" / "data" / "config"

port_generator = salobj.index_generator(imin=3200)

logging.basicConfig()


class CscTestCase(salobj.BaseCscTestCase, asynctest.TestCase):
    def basic_make_csc(self, initial_state, config_dir, simulation_mode):
        return MTMount.MTMountCsc(
            initial_state=initial_state,
            config_dir=config_dir,
            simulation_mode=simulation_mode,
            mock_command_port=next(port_generator),
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
                    "startTracking",
                    "trackTarget",
                    "stopTracking",
                    "stop",
                )
            )

    def make_track_target_kwargs(
        self,
        azimuth,
        elevation,
        azimuthVelocity=0,
        elevationVelocity=0,
        taiTime=None,
        trackId=1,
        tracksys="sidereal",
        radesys="ICRS",
    ):
        """Make keyword argumetns for the trackTarget command.
        """
        if taiTime is None:
            taiTime = salobj.current_tai()
        return dict(
            azimuth=azimuth,
            elevation=elevation,
            azimuthVelocity=azimuthVelocity,
            elevationVelocity=elevationVelocity,
            taiTime=taiTime,
            trackId=trackId,
            tracksys=tracksys,
            radesys=radesys,
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
                    MTMount.CommandCode.CAMERA_CABLE_WRAP_ENABLE_TRACKING,
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
                        MTMount.CommandCode.CAMERA_CABLE_WRAP_TRACK,
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
                    MTMount.CommandCode.CAMERA_CABLE_WRAP_ENABLE_TRACKING,
                )
                self.assertFalse(command.on)
                self.assertTrue(self.csc.mock_controller.command_queue.empty())

                # Check that rotator targets are ignored after
                # tracking is disabled.
                for i in range(2):
                    rotator.tel_Application.set_put(Position=45)
                    await asyncio.sleep(0.1)
                self.assertTrue(self.csc.mock_controller.command_queue.empty())

    async def test_mirror_covers(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(
                remote=self.remote, state=salobj.State.ENABLED
            )
            self.csc.mock_controller.set_command_queue(maxsize=0)

            mock_device = self.csc.mock_controller.device_dict[
                MTMount.DeviceId.MIRROR_COVERS
            ]
            actuator = mock_device.actuator

            self.assertAlmostEqual(actuator.current_position, 0)
            self.assertFalse(actuator.moving)

            # Open the mirror covers.
            # Instead of waiting for the command to finish,
            # check the intermediate state and obtain the expected duration,
            # then wait for the command to finish.
            t0 = time.monotonic()
            await self.remote.cmd_openMirrorCovers.start(timeout=MIRROR_COVER_TIMEOUT)
            dt = time.monotonic() - t0
            print(f"opening the mirror covers took {dt:0.2f} sec")

            # Open the mirror covers again; this should be quick.
            t0 = time.monotonic()
            await self.remote.cmd_openMirrorCovers.start(timeout=STD_TIMEOUT)
            dt = time.monotonic() - t0
            print(f"opening the mirror covers again took {dt:0.2f} sec")
            self.assertAlmostEqual(actuator.current_position, 100)
            self.assertFalse(actuator.moving)

            # Close the mirror covers.
            t0 = time.monotonic()
            await self.remote.cmd_closeMirrorCovers.start(timeout=MIRROR_COVER_TIMEOUT)
            dt = time.monotonic() - t0
            print(f"closing the mirror covers took {dt:0.2f} sec")
            self.assertAlmostEqual(actuator.current_position, 0)
            self.assertFalse(actuator.moving)

            # Close the mirror covers again;
            # the locks are retracted and engaged so it takes some time.
            t0 = time.monotonic()
            await self.remote.cmd_closeMirrorCovers.start(timeout=MIRROR_COVER_TIMEOUT)
            dt = time.monotonic() - t0
            print(f"closing the mirror covers again took {dt:0.2f} sec")
            self.assertAlmostEqual(actuator.current_position, 0)
            self.assertFalse(actuator.moving)

    async def test_move_to_target(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(
                remote=self.remote, state=salobj.State.ENABLED
            )

            mock_azimuth = self.csc.mock_controller.device_dict[
                MTMount.DeviceId.AZIMUTH_AXIS
            ]
            mock_elevation = self.csc.mock_controller.device_dict[
                MTMount.DeviceId.ELEVATION_AXIS
            ]

            tai_unix = salobj.current_tai()
            azimuth_pvt = mock_azimuth.actuator.path.at(tai_unix)
            elevation_pvt = mock_elevation.actuator.path.at(tai_unix)
            self.assertAlmostEqual(elevation_pvt.velocity, 0)
            self.assertAlmostEqual(azimuth_pvt.velocity, 0)

            # Move the axes to a specified position.
            # Use a short move to speed up the test
            # Instead of waiting for the command to finish,
            # check the intermediate state and obtain the expected duration,
            # then wait for the command to finish.
            target_azimuth = azimuth_pvt.position + 2
            target_elevation = elevation_pvt.position + 2
            print(
                f"start test_moveToTarget(azimuth={target_azimuth:0.2f}, "
                f"elevation={target_elevation:0.2f})"
            )
            ackcmd = await self.remote.cmd_moveToTarget.set_start(
                azimuth=target_azimuth,
                elevation=target_elevation,
                timeout=STD_TIMEOUT,
                wait_done=False,
            )
            await asyncio.sleep(0.1)  # Give the command a chance to start
            end_tai = max(mock_elevation.end_tai_unix, mock_azimuth.end_tai_unix)
            duration = 0.1 + end_tai - salobj.current_tai()
            print(f"axis move duration={duration:0.2f} sec")
            await self.remote.cmd_moveToTarget.next_ackcmd(
                ackcmd=ackcmd, timeout=STD_TIMEOUT + duration
            )
            tai_unix = salobj.current_tai()
            elevation_pvt = mock_elevation.actuator.path.at(tai_unix)
            azimuth_pvt = mock_azimuth.actuator.path.at(tai_unix)
            self.assertAlmostEqual(azimuth_pvt.position, target_azimuth)
            self.assertAlmostEqual(elevation_pvt.position, target_elevation)
            self.assertAlmostEqual(azimuth_pvt.velocity, 0)
            self.assertAlmostEqual(elevation_pvt.velocity, 0)

    async def test_tracking(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(
                remote=self.remote, state=salobj.State.ENABLED
            )

            mock_azimuth = self.csc.mock_controller.device_dict[
                MTMount.DeviceId.AZIMUTH_AXIS
            ]
            mock_elevation = self.csc.mock_controller.device_dict[
                MTMount.DeviceId.ELEVATION_AXIS
            ]
            self.assertFalse(mock_azimuth.tracking_enabled)
            self.assertFalse(mock_elevation.tracking_enabled)

            current_tai = salobj.current_tai()
            initial_azimuth = mock_azimuth.actuator.path.at(current_tai).position
            initial_elevation = mock_elevation.actuator.path.at(current_tai).position

            # Check that tracking is rejected if not enabled
            kwargs = self.make_track_target_kwargs(
                azimuth=initial_azimuth,
                elevation=initial_elevation,
                taiTime=current_tai,
            )
            with salobj.assertRaisesAckError():
                await self.remote.cmd_trackTarget.set_start(
                    **kwargs, timeout=STD_TIMEOUT
                )

            # Enable tracking and check mock axis controllers
            await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
            self.assertTrue(mock_azimuth.tracking_enabled)
            self.assertTrue(mock_elevation.tracking_enabled)

            # Specify a few tracking positions
            azimuth_velocity = 0.003
            elevation_velocity = 0.01
            initial_tai = salobj.current_tai()
            for i in range(3):
                await asyncio.sleep(0.2)
                tai_unix = salobj.current_tai()
                dt = tai_unix - initial_tai
                azimuth_position = initial_azimuth + azimuth_velocity * dt
                elevation_position = initial_elevation + elevation_velocity * dt
                kwargs = self.make_track_target_kwargs(
                    azimuth=azimuth_position,
                    azimuthVelocity=azimuth_velocity,
                    elevation=elevation_position,
                    elevationVelocity=elevation_velocity,
                    taiTime=tai_unix,
                )
                await self.remote.cmd_trackTarget.set_start(
                    **kwargs, timeout=STD_TIMEOUT
                )
                self.assertAlmostEqual(
                    mock_azimuth.actuator.target.position, azimuth_position
                )
                self.assertAlmostEqual(
                    mock_azimuth.actuator.target.velocity, azimuth_velocity
                )
                self.assertAlmostEqual(
                    mock_azimuth.actuator.target.tai, tai_unix, delta=0.001
                )
                self.assertAlmostEqual(
                    mock_elevation.actuator.target.position, elevation_position
                )
                self.assertAlmostEqual(
                    mock_elevation.actuator.target.velocity, elevation_velocity
                )
                self.assertAlmostEqual(
                    mock_elevation.actuator.target.tai, tai_unix, delta=0.001
                )
                await self.assert_next_sample(topic=self.remote.evt_target, **kwargs)

            # Disable tracking and check axis controllers
            await self.remote.cmd_stopTracking.start(timeout=STD_TIMEOUT)
            self.assertFalse(mock_azimuth.tracking_enabled)
            self.assertFalse(mock_elevation.tracking_enabled)
