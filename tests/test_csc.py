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
import contextlib
import pathlib
import logging
import time
import unittest

import asynctest

from lsst.ts import salobj
from lsst.ts import MTMount

STD_TIMEOUT = 5  # standard command timeout (sec)
STARTUP_TIMEOUT = 60  # Remote startup time (sec)
MIRROR_COVER_TIMEOUT = 5  # timeout for opening or closing mirror covers (sec)
TEST_CONFIG_DIR = pathlib.Path(__file__).parents[1] / "tests" / "data" / "config"

port_generator = salobj.index_generator(imin=3200)

logging.basicConfig()


class CscTestCase(salobj.BaseCscTestCase, asynctest.TestCase):
    def setUp(self):
        self.mtmount_remote = None
        self.mock_controller = None

    async def tearDown(self):
        if self.mtmount_remote is not None:
            await self.mtmount_remote.close()
        if self.mock_controller is not None:
            await self.mock_controller.close()

    def basic_make_csc(
        self, initial_state, config_dir, simulation_mode, internal_mock_controller
    ):
        mock_command_port = next(port_generator)
        # discard a value for the reply port
        next(port_generator)
        csc = MTMount.MTMountCsc(
            initial_state=initial_state,
            config_dir=config_dir,
            simulation_mode=simulation_mode,
            mock_command_port=mock_command_port,
            run_mock_controller=internal_mock_controller,
        )
        if internal_mock_controller:
            self.mock_controller = csc.mock_controller
        elif simulation_mode != 0:
            self.mock_controller = MTMount.mock.Controller(
                command_port=mock_command_port, log=csc.log
            )
        return csc

    @contextlib.asynccontextmanager
    async def make_csc(
        self,
        initial_state=salobj.State.STANDBY,
        config_dir=None,
        simulation_mode=1,
        internal_mock_controller=False,
    ):
        """Make and return a CSC, an mtmount_remote and possibly a mock
        controller.

        mtmount_remote is needed to read telemetry, which the mock controller
        publishes using the MTMount telemetry topics, instead of NewMTMount.

        Parameters
        ----------
        initial_state : `lsst.ts.salobj.State` or `int`, optional
            The initial state of the CSC. Ignored except in simulation mode
            because in normal operation the initial state is the current state
            of the controller.
        config_dir : `str`, optional
            Directory of configuration files, or None (default)
            for the standard configuration directory (obtained from
            `ConfigureCsc._get_default_config_dir`).
        simulation_mode : `int`, optional
            Simulation mode. Defaults to 1: do simulate.
        internal_mock_controller : `bool`, optional
            Should the CSC run the mock controller?
            Ignored if ``simulation_mode == 0``.
        """
        async with super().make_csc(
            initial_state=initial_state,
            config_dir=config_dir,
            simulation_mode=simulation_mode,
            internal_mock_controller=internal_mock_controller,
        ):
            self.mtmount_remote = salobj.Remote(
                domain=self.csc.salinfo.domain, name="MTMount"
            )
            await asyncio.wait_for(
                self.mtmount_remote.start_task, timeout=STARTUP_TIMEOUT
            )
            yield

    async def test_bin_script(self):
        await self.check_bin_script(
            name="NewMTMount", index=None, exe_name="run_mtmount.py",
        )

    def test_class_attributes(self):
        self.assertEqual(tuple(MTMount.MTMountCsc.valid_simulation_modes), (0, 1))
        self.assertEqual(MTMount.MTMountCsc.version, MTMount.__version__)

    async def test_disconnect(self):
        """Test that the CSC goes to FAULT state if it loses connection
        to the low-level controller.
        """
        async with self.make_csc():
            await salobj.set_summary_state(
                remote=self.remote, state=salobj.State.ENABLED
            )
            await self.assert_next_summary_state(salobj.State.STANDBY)
            await self.assert_next_summary_state(salobj.State.DISABLED)
            await self.assert_next_summary_state(salobj.State.ENABLED)
            await self.mock_controller.close()
            await self.assert_next_summary_state(salobj.State.FAULT)

    async def test_initial_state(self):
        async with self.make_csc():
            await salobj.set_summary_state(
                remote=self.remote, state=salobj.State.ENABLED
            )

            # Test initial telemetry
            data = await self.mtmount_remote.tel_Azimuth.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(data.Azimuth_Angle_Set, 0)
            self.assertEqual(data.Azimuth_Velocity_Set, 0)
            self.assertEqual(data.Azimuth_Angle_Actual, 0)
            self.assertEqual(data.Azimuth_Velocity_Actual, 0)
            self.assertEqual(data.Azimuth_Aceleration_Actual, 0)

            data = await self.mtmount_remote.tel_Elevation.next(
                flush=False, timeout=STD_TIMEOUT
            )
            min_elevation = (
                MTMount.LimitsDict[MTMount.DeviceId.ELEVATION_AXIS]
                .scaled()
                .min_position
            )
            self.assertAlmostEqual(data.Elevation_Angle_Set, min_elevation)
            self.assertEqual(data.Elevation_Velocity_Set, 0)
            self.assertAlmostEqual(data.Elevation_Angle_Actual, min_elevation)
            self.assertEqual(data.Elevation_Velocity_Actual, 0)
            self.assertEqual(data.Elevation_Aceleration_Actual, 0)

            data = await self.mtmount_remote.tel_Camera_Cable_Wrap.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(data.CCW_Angle_1, 0)
            self.assertEqual(data.CCW_Angle_1, 0)
            self.assertEqual(data.CCW_Speed_1, 0)
            self.assertEqual(data.CCW_Speed_1, 0)

    async def test_standard_state_transitions(self):
        async with self.make_csc():
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
            self.mock_controller.command_queue.get(), timeout=timeout
        )

    async def test_camera_cable_wrap_tracking(self):
        async with self.make_csc():
            await salobj.set_summary_state(
                remote=self.remote, state=salobj.State.ENABLED
            )
            self.mock_controller.set_command_queue(maxsize=0)
            ccw_device = self.mock_controller.device_dict[
                MTMount.DeviceId.CAMERA_CABLE_WRAP
            ]
            ccw_actuator = ccw_device.actuator
            self.assertTrue(ccw_device.power_on)
            self.assertTrue(ccw_device.enabled)
            self.assertTrue(ccw_device.tracking_enabled)

            async with salobj.Controller(name="Rotator") as rotator:
                self.assertTrue(self.mock_controller.command_queue.empty())

                # Test regular tracking mode. CCW and Rotator start in sync.
                position0 = 0.0
                velocity = 0.1
                tai0 = salobj.current_tai()
                num_rotator_samples = 10
                prev_position = None
                previous_tai = 0
                previous_delay = None
                for i in range(num_rotator_samples):
                    tai = salobj.current_tai()
                    # Work around non-monotonic clocks, which are
                    # sometimes seen when running Docker on macOS.
                    if tai < previous_tai:
                        tai = previous_tai + 0.001
                    dt = tai - tai0
                    position = position0 + velocity * dt
                    rotator.tel_Application.set_put(Position=position, Demand=position)
                    command = await self.next_lowlevel_command()
                    delay = salobj.current_tai() - tai

                    self.assertEqual(
                        command.command_code,
                        MTMount.CommandCode.CAMERA_CABLE_WRAP_TRACK,
                    )
                    desired_command_tai = (
                        tai + self.csc.config.camera_cable_wrap_advance_time
                    )
                    self.assertLessEqual(command.tai - desired_command_tai, delay)
                    self.assertAlmostEqual(command.position, position)
                    if i == 0 or not self.csc.catch_up_mode:
                        self.assertAlmostEqual(command.velocity, 0.0)
                    else:
                        nominal_dt = tai - previous_tai
                        min_dt = nominal_dt - delay - previous_delay
                        max_dt = nominal_dt + delay + previous_delay
                        min_vel = (position - prev_position) / max_dt
                        max_vel = (position - prev_position) / min_dt
                        max_vel_error = max_vel - min_vel
                        self.assertAlmostEqual(
                            command.velocity, velocity, delta=max_vel_error
                        )

                    # Check camera cable wrap telemetry
                    tel_ccw_data = await self.mtmount_remote.tel_Camera_Cable_Wrap.next(
                        flush=True, timeout=STD_TIMEOUT
                    )
                    actual_segment = ccw_actuator.path.at(tel_ccw_data.timestamp)
                    self.assertAlmostEqual(
                        tel_ccw_data.CCW_Angle_1, actual_segment.position, delta=1.0e-4
                    )
                    self.assertAlmostEqual(
                        tel_ccw_data.CCW_Speed_1, actual_segment.velocity, delta=1.0e-4
                    )
                    self.assertEqual(tel_ccw_data.CCW_Angle_1, tel_ccw_data.CCW_Angle_2)
                    self.assertEqual(tel_ccw_data.CCW_Speed_1, tel_ccw_data.CCW_Speed_2)

                    await asyncio.sleep(0.1)
                    prev_position = position
                    previous_tai = tai
                    previous_delay = delay

                await self.remote.cmd_disableCameraCableWrapTracking.start(
                    timeout=STD_TIMEOUT
                )
                command = await self.next_lowlevel_command()
                self.assertEqual(
                    command.command_code,
                    MTMount.CommandCode.CAMERA_CABLE_WRAP_ENABLE_TRACKING,
                )
                self.assertFalse(command.on)
                self.assertTrue(ccw_device.enabled)
                self.assertFalse(ccw_device.tracking_enabled)
                self.assertTrue(self.mock_controller.command_queue.empty())

                # Check that rotator targets are ignored after
                # tracking is disabled.
                for i in range(2):
                    rotator.tel_Application.set_put(Position=45)
                    await asyncio.sleep(0.1)
                self.assertTrue(self.mock_controller.command_queue.empty())

    @unittest.skip("Not available in this camera-cable-wrap-only version")
    async def test_mirror_covers(self):
        async with self.make_csc():
            await salobj.set_summary_state(
                remote=self.remote, state=salobj.State.ENABLED
            )
            self.mock_controller.set_command_queue(maxsize=0)

            mock_device = self.mock_controller.device_dict[
                MTMount.DeviceId.MIRROR_COVERS
            ]
            actuator = mock_device.actuator

            self.assertAlmostEqual(actuator.position(), 0)
            self.assertFalse(actuator.moving())

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
            self.assertAlmostEqual(actuator.position(), 100)
            self.assertFalse(actuator.moving())

            # Close the mirror covers.
            t0 = time.monotonic()
            await self.remote.cmd_closeMirrorCovers.start(timeout=MIRROR_COVER_TIMEOUT)
            dt = time.monotonic() - t0
            print(f"closing the mirror covers took {dt:0.2f} sec")
            self.assertAlmostEqual(actuator.position(), 0)
            self.assertFalse(actuator.moving())

            # Close the mirror covers again;
            # the locks are retracted and engaged so it takes some time.
            t0 = time.monotonic()
            await self.remote.cmd_closeMirrorCovers.start(timeout=MIRROR_COVER_TIMEOUT)
            dt = time.monotonic() - t0
            print(f"closing the mirror covers again took {dt:0.2f} sec")
            self.assertAlmostEqual(actuator.position(), 0)
            self.assertFalse(actuator.moving())

    @unittest.skip("Not available in this camera-cable-wrap-only version")
    async def test_move_to_target(self):
        async with self.make_csc():
            await salobj.set_summary_state(
                remote=self.remote, state=salobj.State.ENABLED
            )
            await self.assert_next_sample(
                self.remote.evt_axesInPosition, azimuth=False, elevation=False
            )

            mock_azimuth = self.mock_controller.device_dict[
                MTMount.DeviceId.AZIMUTH_AXIS
            ]
            mock_elevation = self.mock_controller.device_dict[
                MTMount.DeviceId.ELEVATION_AXIS
            ]

            tai = salobj.current_tai()
            azimuth_pvt = mock_azimuth.actuator.path.at(tai)
            elevation_pvt = mock_elevation.actuator.path.at(tai)
            self.assertAlmostEqual(elevation_pvt.velocity, 0)
            self.assertAlmostEqual(azimuth_pvt.velocity, 0)

            # Move the axes to a specified position.
            # Use a short move to speed up the test,
            # but make the azimuth move significantly shorter
            # so it finishes first.
            # Instead of waiting for the command to finish,
            # check the intermediate state and obtain the expected duration,
            # then wait for the command to finish.
            target_azimuth = azimuth_pvt.position + 1
            target_elevation = elevation_pvt.position + 2
            estimated_move_time = 2  # seconds
            print(
                f"start test_moveToTarget(azimuth={target_azimuth:0.2f}, "
                f"elevation={target_elevation:0.2f})"
            )
            task = asyncio.create_task(
                self.remote.cmd_moveToTarget.set_start(
                    azimuth=target_azimuth,
                    elevation=target_elevation,
                    timeout=estimated_move_time + STD_TIMEOUT,
                )
            )
            await asyncio.sleep(0.1)  # Give the command a chance to start
            # Print move duration; the elevation move will take longer
            duration = mock_elevation.end_tai - salobj.current_tai()
            print(f"axis move duration={duration:0.2f} sec")
            await self.assert_next_sample(
                self.remote.evt_axesInPosition, azimuth=True, elevation=False,
            )
            await self.assert_next_sample(
                self.remote.evt_axesInPosition, azimuth=True, elevation=True,
            )
            await task
            tai = salobj.current_tai()
            elevation_pvt = mock_elevation.actuator.path.at(tai)
            azimuth_pvt = mock_azimuth.actuator.path.at(tai)
            self.assertAlmostEqual(azimuth_pvt.position, target_azimuth)
            self.assertGreaterEqual(tai, mock_elevation.actuator.path[-1].tai)
            self.assertAlmostEqual(elevation_pvt.position, target_elevation)
            self.assertAlmostEqual(azimuth_pvt.velocity, 0)
            self.assertAlmostEqual(elevation_pvt.velocity, 0)

            # Check that putting the CSC into STANDBY state
            # sends the axes out of position
            await self.remote.cmd_disable.start(timeout=STD_TIMEOUT)
            await self.remote.cmd_standby.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                self.remote.evt_axesInPosition, azimuth=False, elevation=False,
            )

    @unittest.skip("Not available in this camera-cable-wrap-only version")
    async def test_tracking(self):
        async with self.make_csc():
            await salobj.set_summary_state(
                remote=self.remote, state=salobj.State.ENABLED
            )
            await self.assert_next_sample(
                self.remote.evt_axesInPosition, azimuth=False, elevation=False
            )

            mock_azimuth = self.mock_controller.device_dict[
                MTMount.DeviceId.AZIMUTH_AXIS
            ]
            mock_elevation = self.mock_controller.device_dict[
                MTMount.DeviceId.ELEVATION_AXIS
            ]
            self.assertFalse(mock_azimuth.tracking_enabled)
            self.assertFalse(mock_elevation.tracking_enabled)

            tai = salobj.current_tai()
            initial_azimuth = mock_azimuth.actuator.path.at(tai).position
            initial_elevation = mock_elevation.actuator.path.at(tai).position

            # Check that tracking is rejected if not enabled
            kwargs = self.make_track_target_kwargs(
                azimuth=initial_azimuth, elevation=initial_elevation, taiTime=tai,
            )
            with salobj.assertRaisesAckError():
                await self.remote.cmd_trackTarget.set_start(
                    **kwargs, timeout=STD_TIMEOUT
                )

            # Enable tracking and check mock axis controllers
            await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
            self.assertTrue(mock_azimuth.tracking_enabled)
            self.assertTrue(mock_elevation.tracking_enabled)

            # Slew and track until both axes are in position.
            # Make the elevation move significantly smaller,
            # so it is sure to be in position first.
            t0 = time.monotonic()
            estimated_slew_time = 2  # seconds
            tracking_task = asyncio.create_task(
                self.track_target_loop(
                    azimuth=initial_azimuth + 3,
                    elevation=initial_elevation + 1,
                    azimuth_velocity=0.003,
                    elevation_velocity=0.003,
                )
            )
            await self.assert_next_sample(
                self.remote.evt_axesInPosition,
                azimuth=False,
                elevation=True,
                timeout=estimated_slew_time + STD_TIMEOUT,
            )
            dt_elevation = time.monotonic() - t0
            await self.assert_next_sample(
                self.remote.evt_axesInPosition,
                azimuth=True,
                elevation=True,
                timeout=estimated_slew_time + STD_TIMEOUT,
            )
            dt_azimuth = time.monotonic() - t0
            tracking_task.cancel()
            print(
                f"Time to finish slew for elevation={dt_elevation:0.2f}; "
                f"azimuth={dt_azimuth:0.2f} seconds"
            )

            # Disable tracking and check axis controllers
            await self.remote.cmd_stopTracking.start(timeout=STD_TIMEOUT)
            self.assertFalse(mock_azimuth.tracking_enabled)
            self.assertFalse(mock_elevation.tracking_enabled)

            # Check that both axes are no longer in position.
            # We don't yet know the details of Tekniker's InPosition message
            # so make the test succeed whether it must be sent separately
            # for each axis (resulting in two axesInPosition events)
            # or can be sent for both axes at the same time
            # (resulting in a single axesInPosition event).
            data = await self.assert_next_sample(self.remote.evt_axesInPosition,)
            self.assertIn(False, (data.azimuth, data.elevation))
            if True in (data.azimuth, data.elevation):
                await self.assert_next_sample(
                    self.remote.evt_axesInPosition, azimuth=False, elevation=False,
                )

    async def track_target_loop(
        self, azimuth, elevation, azimuth_velocity, elevation_velocity
    ):
        """Provide a stream of trackTarget commands until cancelled.
        """
        # Slew and track until both axes are in position
        mock_azimuth = self.mock_controller.device_dict[MTMount.DeviceId.AZIMUTH_AXIS]
        mock_elevation = self.mock_controller.device_dict[
            MTMount.DeviceId.ELEVATION_AXIS
        ]
        self.assertTrue(mock_azimuth.tracking_enabled)
        self.assertTrue(mock_elevation.tracking_enabled)
        azimuth_actuator = mock_azimuth.actuator
        elevation_actuator = mock_elevation.actuator

        initial_tai = salobj.current_tai()
        previous_tai = 0
        while True:
            await asyncio.sleep(0.1)
            tai = salobj.current_tai()
            # Work around non-monotonic clocks, which are
            # sometimes seen when running Docker on macOS.
            if tai < previous_tai:
                tai = previous_tai + 0.001
            dt = tai - initial_tai
            current_azimuth = azimuth + azimuth_velocity * dt
            current_elevation = elevation + elevation_velocity * dt
            kwargs = self.make_track_target_kwargs(
                azimuth=current_azimuth,
                azimuthVelocity=azimuth_velocity,
                elevation=current_elevation,
                elevationVelocity=elevation_velocity,
                taiTime=tai,
            )
            await self.remote.cmd_trackTarget.set_start(**kwargs, timeout=STD_TIMEOUT)
            self.assertAlmostEqual(
                mock_azimuth.actuator.target.position, current_azimuth
            )
            self.assertAlmostEqual(
                mock_azimuth.actuator.target.velocity, azimuth_velocity
            )
            self.assertAlmostEqual(mock_azimuth.actuator.target.tai, tai, delta=0.001)
            self.assertAlmostEqual(
                mock_elevation.actuator.target.position, current_elevation
            )
            self.assertAlmostEqual(
                mock_elevation.actuator.target.velocity, elevation_velocity
            )
            self.assertAlmostEqual(mock_elevation.actuator.target.tai, tai, delta=0.001)
            await self.assert_next_sample(topic=self.remote.evt_target, **kwargs)

            # Check elevation and azimuth telemetry
            tel_el_data = await self.mtmount_remote.tel_Elevation.next(
                flush=True, timeout=STD_TIMEOUT
            )
            el_actual = elevation_actuator.path.at(tel_el_data.timestamp)
            el_target = elevation_actuator.target.at(tel_el_data.timestamp)
            self.assertAlmostEqual(tel_el_data.Elevation_Angle_Set, el_target.position)
            self.assertAlmostEqual(
                tel_el_data.Elevation_Velocity_Set, el_target.velocity
            )
            self.assertAlmostEqual(
                tel_el_data.Elevation_Angle_Actual, el_actual.position
            )
            self.assertAlmostEqual(
                tel_el_data.Elevation_Velocity_Actual, el_actual.velocity
            )

            tel_az_data = await self.mtmount_remote.tel_Azimuth.next(
                flush=True, timeout=STD_TIMEOUT
            )
            az_actual = azimuth_actuator.path.at(tel_az_data.timestamp)
            az_target = azimuth_actuator.target.at(tel_az_data.timestamp)
            self.assertAlmostEqual(tel_az_data.Azimuth_Angle_Set, az_target.position)
            self.assertAlmostEqual(tel_az_data.Azimuth_Velocity_Set, az_target.velocity)
            self.assertAlmostEqual(tel_az_data.Azimuth_Angle_Actual, az_actual.position)
            self.assertAlmostEqual(
                tel_az_data.Azimuth_Velocity_Actual, az_actual.velocity
            )

            previous_tai = tai
