# This file is part of ts_mtmount.
#
# Developed for Rubin Observatory Telescope and Site Systems.
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
import unittest

import pytest

from lsst.ts import salobj
from lsst.ts import mtmount
from lsst.ts import utils

STD_TIMEOUT = 10  # standard command timeout (sec)
# timeout for opening or closing mirror covers (sec)
MIRROR_COVER_TIMEOUT = STD_TIMEOUT + 2
TEST_CONFIG_DIR = pathlib.Path(__file__).parents[1] / "tests" / "data" / "config"

# timeout for constructing several remotes and controllers (sec)
LONG_TIMEOUT = 60

port_generator = utils.index_generator(imin=3200)

logging.basicConfig()


class CscTestCase(salobj.BaseCscTestCase, unittest.IsolatedAsyncioTestCase):
    def basic_make_csc(
        self, initial_state, config_dir, simulation_mode, internal_mock_controller
    ):
        if simulation_mode != 0 and not internal_mock_controller:
            mock_command_port = self.mock_controller.command_server.port
            mock_telemetry_port = self.mock_controller.telemetry_server.port
        else:
            mock_command_port = None
            mock_telemetry_port = None
        csc = mtmount.MTMountCsc(
            initial_state=initial_state,
            config_dir=config_dir,
            simulation_mode=simulation_mode,
            mock_command_port=mock_command_port,
            mock_telemetry_port=mock_telemetry_port,
            run_mock_controller=internal_mock_controller,
        )
        if simulation_mode != 0 and internal_mock_controller:
            self.mock_controller = csc.mock_controller
        return csc

    @contextlib.asynccontextmanager
    async def make_csc(
        self,
        initial_state,
        config_dir=TEST_CONFIG_DIR,
        simulation_mode=1,
        internal_mock_controller=False,
    ):
        """Make CSC, and optionally a mock controller.

        This override exists to provide preferred defaults.

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
        if simulation_mode != 0 and not internal_mock_controller:
            self.mock_controller = mtmount.mock.Controller(
                log=logging.getLogger(), random_ports=True
            )
            self.addAsyncCleanup(self.mock_controller.close)
            await self.mock_controller.start_task
        else:
            self.mock_controller = None
        async with super().make_csc(
            initial_state=initial_state,
            config_dir=config_dir,
            simulation_mode=simulation_mode,
            internal_mock_controller=internal_mock_controller,
        ):
            yield

    async def test_bin_script(self):
        await self.check_bin_script(
            name="MTMount", index=None, exe_name="run_mtmount.py"
        )

    def test_class_attributes(self):
        assert tuple(mtmount.MTMountCsc.valid_simulation_modes) == (0, 1)
        assert mtmount.MTMountCsc.version == mtmount.__version__

    async def test_disconnect(self):
        """Test that the CSC goes to FAULT state if it loses connection
        to the low-level controller.
        """
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, internal_mock_controller=False
        ):
            await self.assert_next_summary_state(salobj.State.ENABLED)
            await self.mock_controller.close()
            await self.assert_next_summary_state(salobj.State.FAULT)

    async def test_initial_state(self):
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, internal_mock_controller=False
        ):
            await self.assert_next_sample(
                self.remote.evt_softwareVersions,
                cscVersion=mtmount.__version__,
                subsystemVersions="",
            )

            await self.assert_next_sample(
                self.remote.evt_cameraCableWrapFollowing, enabled=False
            )

            # Test initial telemetry
            await self.assert_next_sample(
                topic=self.remote.tel_azimuth,
                flush=False,
                actualPosition=0,
                actualVelocity=0,
                actualAcceleration=0,
                actualTorque=0,
                demandPosition=0,
                demandVelocity=0,
            )

            data = await self.assert_next_sample(
                topic=self.remote.tel_elevation,
                flush=False,
                actualVelocity=0,
                actualAcceleration=0,
                actualTorque=0,
                demandVelocity=0,
            )
            min_elevation = (
                mtmount.CmdLimitsDict[mtmount.DeviceId.ELEVATION_AXIS]
                .scaled()
                .min_position
            )
            assert data.demandPosition == pytest.approx(min_elevation)
            assert data.actualPosition == pytest.approx(min_elevation)

            await self.assert_next_sample(
                topic=self.remote.tel_cameraCableWrap,
                flush=False,
                actualPosition=0,
                actualVelocity=0,
                actualAcceleration=0,
            )

    async def test_standard_state_transitions(self):
        async with self.make_csc(initial_state=salobj.State.STANDBY):
            await self.check_standard_state_transitions(
                enabled_commands=(
                    "closeMirrorCovers",
                    "openMirrorCovers",
                    "disableCameraCableWrapFollowing",
                    "enableCameraCableWrapFollowing",
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
        """Make keyword argumetns for the trackTarget command."""
        if taiTime is None:
            taiTime = utils.current_tai()
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
        """Get the next low level command."""
        return await asyncio.wait_for(
            self.mock_controller.command_queue.get(), timeout=timeout
        )

    async def test_camera_cable_wrap_following(self):
        # Start the CSC in DISABLED state
        # so we can get the rotator remote running
        # before the camera cable wrap loop needs data from the rotator.
        async with self.make_csc(initial_state=salobj.State.DISABLED):
            await self.assert_next_sample(
                self.remote.evt_cameraCableWrapFollowing, enabled=False
            )
            ccw_device = self.mock_controller.device_dict[
                mtmount.DeviceId.CAMERA_CABLE_WRAP
            ]
            ccw_actuator = ccw_device.actuator
            assert not ccw_device.power_on
            assert not ccw_device.enabled

            async with salobj.Controller(name="MTRotator") as rotator:
                await self.remote.cmd_enable.start(timeout=STD_TIMEOUT)
                assert ccw_device.power_on
                assert ccw_device.enabled

                await self.assert_next_sample(
                    self.remote.evt_cameraCableWrapFollowing, enabled=True
                )
                assert ccw_device.tracking_enabled
                self.mock_controller.set_command_queue(maxsize=0)

                assert self.mock_controller.command_queue.empty()

                # Test regular tracking mode. CCW and MTRotator start in sync.
                position0 = 0.0
                velocity = 0.1
                tai0 = utils.current_tai()
                num_rotator_samples = 10
                previous_tai = 0
                for i in range(num_rotator_samples):
                    tai = utils.current_tai()
                    # Work around non-monotonic clocks, which are
                    # sometimes seen when running Docker on macOS.
                    if tai < previous_tai:
                        tai = previous_tai + 0.001
                    dt = tai - tai0
                    position = position0 + velocity * dt
                    await rotator.tel_rotation.set_write(
                        demandPosition=position,
                        demandVelocity=velocity,
                        demandAcceleration=0,
                        actualPosition=position,
                        actualVelocity=velocity,
                        timestamp=tai,
                    )
                    command = await self.next_lowlevel_command()
                    delay = utils.current_tai() - tai
                    assert (
                        command.command_code
                        == mtmount.CommandCode.CAMERA_CABLE_WRAP_TRACK
                    )
                    desired_command_tai = (
                        tai + self.csc.config.camera_cable_wrap_advance_time
                    )
                    assert command.tai - desired_command_tai <= delay

                    # Check camera cable wrap telemetry;
                    # use a crude comparison because a new CCW tracking
                    # command will alter the path.
                    tel_ccw_data = await self.remote.tel_cameraCableWrap.next(
                        flush=True, timeout=STD_TIMEOUT
                    )
                    actual_segment = ccw_actuator.path.at(tel_ccw_data.timestamp)
                    assert tel_ccw_data.actualPosition == pytest.approx(
                        actual_segment.position, abs=0.1
                    )
                    assert tel_ccw_data.actualVelocity == pytest.approx(
                        actual_segment.velocity, abs=0.1
                    )
                    assert tel_ccw_data.actualAcceleration == pytest.approx(
                        actual_segment.acceleration, abs=0.1
                    )

                    await asyncio.sleep(0.1)
                    previous_tai = tai

                # Stop the camera cable wrap from following the rotator.
                await self.remote.cmd_disableCameraCableWrapFollowing.start(
                    timeout=STD_TIMEOUT
                )
                command = await self.next_lowlevel_command()
                assert (
                    command.command_code == mtmount.CommandCode.CAMERA_CABLE_WRAP_STOP
                )
                assert ccw_device.enabled
                assert not ccw_device.tracking_enabled
                assert self.mock_controller.command_queue.empty()

                # Check that rotator targets are ignored after
                # tracking is disabled.
                for i in range(2):
                    await rotator.tel_rotation.set_write(
                        demandPosition=45,
                        demandVelocity=1,
                        demandAcceleration=0,
                        actualPosition=45,
                        actualVelocity=1,
                        timestamp=utils.current_tai(),
                    )
                    await asyncio.sleep(0.1)
                assert self.mock_controller.command_queue.empty()

                # Restart the camera cable wrap following the rotator.
                await self.remote.cmd_enableCameraCableWrapFollowing.start(
                    timeout=STD_TIMEOUT
                )
                assert ccw_device.tracking_enabled

    async def test_camera_cable_wrap_truncation_min(self):
        """Test that camera cable wrap following code truncates position
        and velocity as needed.
        """
        await asyncio.wait_for(
            self.check_camera_cable_wrap_truncation(do_max_limit=False),
            timeout=LONG_TIMEOUT,
        )

    async def test_camera_cable_wrap_truncation_max(self):
        """Test that camera cable wrap following code truncates position
        and velocity as needed.
        """
        await asyncio.wait_for(
            self.check_camera_cable_wrap_truncation(do_max_limit=True),
            timeout=LONG_TIMEOUT,
        )

    async def check_camera_cable_wrap_truncation(self, do_max_limit):
        """Test that camera cable wrap following code truncates position
        and velocity as needed.

        Parameters
        ----------
        do_max_limit : `bool`
            If True then run into the max limit at max velocity,
            else run into the min limit at max negative velocity.
        """
        # Start the CSC in DISABLED state
        # so we can get the rotator remote running
        # before the camera cable wrap loop needs data from the rotator.
        async with self.make_csc(initial_state=salobj.State.DISABLED):
            await self.assert_next_sample(
                self.remote.evt_cameraCableWrapFollowing, enabled=False
            )
            ccw_device = self.mock_controller.device_dict[
                mtmount.DeviceId.CAMERA_CABLE_WRAP
            ]
            assert not ccw_device.power_on
            assert not ccw_device.enabled

            async with salobj.Controller(name="MTRotator") as rotator:
                await self.remote.cmd_enable.start(timeout=STD_TIMEOUT)
                assert ccw_device.power_on
                assert ccw_device.enabled

                await self.assert_next_sample(
                    self.remote.evt_cameraCableWrapFollowing, enabled=True
                )
                assert ccw_device.tracking_enabled
                self.mock_controller.set_command_queue(maxsize=0)

                assert self.mock_controller.command_queue.empty()

                # Test regular tracking mode. CCW and MTRotator start in sync.
                ccw_limits = mtmount.CmdLimitsDict[mtmount.DeviceId.CAMERA_CABLE_WRAP]
                # seconds until the CCW demand hits the position limit;
                # make it large enough that we get a few target events
                # before we reach the position limit.
                trunc_time = 0.5
                if do_max_limit:
                    position_limit = ccw_limits.max_position - self.csc.limits_margin
                    velocity_limit = ccw_limits.max_velocity - self.csc.limits_margin
                else:
                    position_limit = ccw_limits.min_position + self.csc.limits_margin
                    velocity_limit = -ccw_limits.max_velocity + self.csc.limits_margin
                velocity = velocity_limit * 1.1
                position0 = position_limit - (velocity * trunc_time)
                print(
                    f"Position limit={position_limit}, velocity limit={velocity_limit}"
                )

                tai0 = utils.current_tai()
                previous_tai = 0
                while True:
                    tai = utils.current_tai()
                    # Work around non-monotonic clocks, which are
                    # sometimes seen when running Docker on macOS.
                    if tai < previous_tai:
                        tai = previous_tai + 0.001
                    dt = tai - tai0
                    position = position0 + velocity * dt
                    await rotator.tel_rotation.set_write(
                        demandPosition=position,
                        demandVelocity=velocity,
                        demandAcceleration=0,
                        actualPosition=position,
                        actualVelocity=velocity,
                        timestamp=tai,
                    )
                    command = await self.next_lowlevel_command()
                    delay = utils.current_tai() - tai
                    assert (
                        command.command_code
                        == mtmount.CommandCode.CAMERA_CABLE_WRAP_TRACK
                    )
                    desired_command_tai = (
                        tai + self.csc.config.camera_cable_wrap_advance_time
                    )
                    assert command.tai - desired_command_tai <= delay

                    # Check camera cable wrap command
                    ccw_target = await self.remote.evt_cameraCableWrapTarget.next(
                        flush=True, timeout=STD_TIMEOUT
                    )
                    if do_max_limit:
                        assert ccw_target.velocity <= ccw_limits.max_velocity
                        assert ccw_target.position <= position_limit
                    else:
                        assert ccw_target.velocity >= -ccw_limits.max_velocity
                        assert ccw_target.position >= position_limit
                    print(
                        f"CCW target position={ccw_target.position}, velocity={ccw_target.velocity}"
                    )
                    if ccw_target.position == pytest.approx(position_limit):
                        break

                    await asyncio.sleep(0.1)
                    previous_tai = tai

    async def track_target_loop(
        self, azimuth, elevation, azimuth_velocity, elevation_velocity
    ):
        """Provide a stream of trackTarget commands until cancelled."""
        # Slew and track until both axes are in position
        mock_azimuth = self.mock_controller.device_dict[mtmount.DeviceId.AZIMUTH_AXIS]
        mock_elevation = self.mock_controller.device_dict[
            mtmount.DeviceId.ELEVATION_AXIS
        ]
        assert mock_azimuth.tracking_enabled
        assert mock_elevation.tracking_enabled
        azimuth_actuator = mock_azimuth.actuator
        elevation_actuator = mock_elevation.actuator

        initial_tai = utils.current_tai()
        previous_tai = 0
        while True:
            await asyncio.sleep(0.1)
            tai = utils.current_tai()
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
            assert mock_azimuth.actuator.target.position == pytest.approx(
                current_azimuth
            )
            assert mock_azimuth.actuator.target.velocity == pytest.approx(
                azimuth_velocity
            )
            assert mock_azimuth.actuator.target.tai == pytest.approx(tai, abs=0.001)
            assert mock_elevation.actuator.target.position == pytest.approx(
                current_elevation
            )
            assert mock_elevation.actuator.target.velocity == pytest.approx(
                elevation_velocity
            )
            assert mock_elevation.actuator.target.tai == pytest.approx(tai, abs=0.001)
            await self.assert_next_sample(topic=self.remote.evt_target, **kwargs)

            # Check elevation and azimuth telemetry
            tel_el_data = await self.remote.tel_elevation.next(
                flush=True, timeout=STD_TIMEOUT
            )
            el_actual = elevation_actuator.path.at(tel_el_data.timestamp)
            el_target = elevation_actuator.target.at(tel_el_data.timestamp)
            assert tel_el_data.demandPosition == pytest.approx(el_target.position)
            assert tel_el_data.demandVelocity == pytest.approx(el_target.velocity)
            assert tel_el_data.actualPosition == pytest.approx(el_actual.position)
            assert tel_el_data.actualVelocity == pytest.approx(el_actual.velocity)

            tel_az_data = await self.remote.tel_azimuth.next(
                flush=True, timeout=STD_TIMEOUT
            )
            az_actual = azimuth_actuator.path.at(tel_az_data.timestamp)
            az_target = azimuth_actuator.target.at(tel_az_data.timestamp)
            assert tel_az_data.demandPosition == pytest.approx(az_target.position)
            assert tel_az_data.demandVelocity == pytest.approx(az_target.velocity)
            assert tel_az_data.actualPosition == pytest.approx(az_actual.position)
            assert tel_az_data.actualVelocity == pytest.approx(az_actual.velocity)

            previous_tai = tai
