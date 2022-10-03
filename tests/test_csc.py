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
import itertools
import pathlib
import logging
import math
import time
import unittest

import numpy.testing
import pytest

from lsst.ts import utils
from lsst.ts import salobj
from lsst.ts import mtmount
from lsst.ts.idl.enums.MTMount import (
    AxisMotionState,
    DeployableMotionState,
    ElevationLockingPinMotionState,
    PowerState,
    System,
)

STD_TIMEOUT = 60  # standard command timeout (sec)
# timeout for opening or closing mirror covers (sec)
MIRROR_COVER_TIMEOUT = STD_TIMEOUT + 2

# timeout for reading telemetry that should not appear (sec)
NOTELEMETRY_TIMEOUT = 2

# timeout for constructing several remotes and controllers (sec)
LONG_TIMEOUT = 60

TEST_CONFIG_DIR = pathlib.Path(__file__).parents[1] / "tests" / "data" / "config"

SAFETY_INTERLOCKS_FIELDS = (
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
                log=logging.getLogger(),
                random_ports=True,
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

    async def assert_axes_in_position(
        self,
        *,
        elevation=None,
        azimuth=None,
        camera_cable_wrap=None,
        timeout=STD_TIMEOUT,
    ):
        r"""Assert the next _axis_\ inPosition event for one or more axes.

        Parameters
        ----------
        elevation : bool or None
            If not None then the required value for the inPosition field
            of the next elevationInPosition event.
        azimuth : bool or None
            If not None then the required value for the inPosition field
            of the next azimuthInPosition event.
        camera_cable_wrap : bool or None
            If not None then the required value for the inPosition field
            of the next cameraCableWrapInPosition event.
        timeout : float
            Time limit. Applied individually to each axis for which
            the argument is not None.
        """
        if azimuth is not None:
            await self.assert_next_sample(
                topic=self.remote.evt_azimuthInPosition,
                inPosition=azimuth,
                timeout=timeout,
            )
        if elevation is not None:
            await self.assert_next_sample(
                topic=self.remote.evt_elevationInPosition,
                inPosition=elevation,
                timeout=timeout,
            )
        if camera_cable_wrap is not None:
            await self.assert_next_sample(
                topic=self.remote.evt_cameraCableWrapInPosition,
                inPosition=camera_cable_wrap,
                timeout=timeout,
            )

    async def assert_target_cleared(self, timeout=STD_TIMEOUT):
        """Assert that the next target event is in cleared/initial state."""
        data = await self.assert_next_sample(
            topic=self.remote.evt_target,
            trackId=0,
            tracksys="",
            radesys="",
            timeout=timeout,
        )
        for field_name in (
            "azimuth",
            "elevation",
            "azimuthVelocity",
            "elevationVelocity",
            "taiTime",
        ):
            value = getattr(data, field_name)
            assert math.isnan(value)

    async def test_bin_script(self):
        await self.check_bin_script(
            name="MTMount",
            index=None,
            exe_name="run_mtmount",
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
                topic=self.remote.evt_softwareVersions,
                cscVersion=mtmount.__version__,
                subsystemVersions="",
            )
            # TODO DM-36445: remove this hasattr test
            # and assume Remote has an evt_telemetryConnected topic
            if hasattr(self.remote, "evt_telemetryConnected"):
                await self.assert_next_sample(
                    topic=self.remote.evt_telemetryConnected, connected=False
                )
                await self.assert_next_sample(
                    topic=self.remote.evt_telemetryConnected, connected=True
                )
            for axis_name in ("Azimuth", "Elevation"):
                system_id = getattr(System, axis_name.upper())
                topic = getattr(
                    self.remote, f"evt_{axis_name.lower()}ControllerSettings"
                )
                axis_device = self.mock_controller.device_dict[system_id]
                axis_actuator = axis_device.actuator
                axis_cmd_limits = axis_device.cmd_limits
                axis_settings = self.mock_controller.detailed_settings["MainAxis"][
                    axis_name
                ]
                if axis_name == "Elevation":
                    extra_elevation_fields = dict(
                        minOperationalL2LimitEnabled=True,
                        maxOperationalL2LimitEnabled=True,
                    )
                else:
                    extra_elevation_fields = dict()
                await self.assert_next_sample(
                    topic=topic,
                    minCmdPositionEnabled=True,
                    maxCmdPositionEnabled=True,
                    minL1LimitEnabled=True,
                    maxL1LimitEnabled=True,
                    minOperationalL1LimitEnabled=True,
                    maxOperationalL1LimitEnabled=True,
                    minL2LimitEnabled=True,
                    maxL2LimitEnabled=True,
                    minCmdPosition=axis_cmd_limits.min_position,
                    maxCmdPosition=axis_cmd_limits.max_position,
                    minL1Limit=axis_settings[
                        "LimitsNegativeAdjustableSoftwareLimitValue"
                    ],
                    maxL1Limit=axis_settings[
                        "LimitsPositiveAdjustableSoftwareLimitValue"
                    ],
                    maxCmdVelocity=axis_cmd_limits.max_velocity,
                    maxMoveVelocity=axis_settings["TcsDefaultVelocity"],
                    maxMoveAcceleration=axis_settings["TcsDefaultAcceleration"],
                    maxMoveJerk=axis_settings["TcsDefaultJerk"],
                    maxTrackingVelocity=axis_actuator.max_velocity,
                    maxTrackingAcceleration=axis_actuator.max_acceleration,
                    maxTrackingJerk=axis_settings["SoftmotionTrackingMaxJerk"],
                    **extra_elevation_fields,
                )
            ccw_device = self.mock_controller.device_dict[System.CAMERA_CABLE_WRAP]
            ccw_actuator = ccw_device.actuator
            ccw_cmd_limits = ccw_device.cmd_limits
            ccw_settings = self.mock_controller.detailed_settings["CW"]["CCW"]
            await self.assert_next_sample(
                topic=self.remote.evt_cameraCableWrapControllerSettings,
                l1LimitsEnabled=True,
                l2LimitsEnabled=True,
                minCmdPosition=ccw_cmd_limits.min_position,
                maxCmdPosition=ccw_cmd_limits.max_position,
                minL1Limit=ccw_settings["MinSoftwareLimit"],
                maxL1Limit=ccw_settings["MaxSoftwareLimit"],
                maxCmdVelocity=ccw_cmd_limits.max_velocity,
                maxMoveVelocity=ccw_settings["DefaultSpeed"],
                maxMoveAcceleration=ccw_settings["DefaultAcceleration"],
                maxMoveJerk=ccw_settings["DefaultJerk"],
                maxTrackingVelocity=ccw_actuator.max_velocity,
                maxTrackingAcceleration=ccw_actuator.max_acceleration,
                maxTrackingJerk=ccw_settings["TrackingJerk"],
            )

            available_settings = self.mock_controller.available_settings
            await self.assert_next_sample(
                topic=self.remote.evt_availableSettings,
                names=", ".join(item["name"] for item in available_settings),
                createdDates=", ".join(
                    item["createdDate"].iso for item in available_settings
                ),
                modifiedDates=", ".join(
                    item["modifiedDate"].iso for item in available_settings
                ),
            )
            await self.assert_next_sample(
                topic=self.remote.evt_azimuthToppleBlock, reverse=False, forward=False
            )
            await self.assert_next_sample(
                topic=self.remote.evt_cameraCableWrapFollowing, enabled=False
            )
            await self.assert_target_cleared()
            # When the CSC first connects it ask for current state,
            # before it asks to be the commander.
            await self.assert_next_sample(
                topic=self.remote.evt_commander, commander=mtmount.Source.NONE
            )

            for topic in (
                self.remote.evt_elevationLimits,
                self.remote.evt_azimuthLimits,
                self.remote.evt_cameraCableWrapLimits,
            ):
                await self.assert_next_sample(topic=topic, limits=0)

            for topic in (
                self.remote.evt_elevationMotionState,
                self.remote.evt_azimuthMotionState,
                self.remote.evt_cameraCableWrapMotionState,
            ):
                await self.assert_next_sample(
                    topic=topic, state=AxisMotionState.STOPPED
                )

            await self.assert_next_sample(
                topic=self.remote.evt_deployablePlatformsMotionState,
                state=DeployableMotionState.RETRACTED,
                elementsState=[DeployableMotionState.RETRACTED] * 2,
            )

            await self.assert_next_sample(
                topic=self.remote.evt_elevationLockingPinMotionState,
                state=ElevationLockingPinMotionState.UNLOCKED,
                elementsState=[ElevationLockingPinMotionState.UNLOCKED] * 2,
            )

            # Test xSystemState events
            # Set of topic name prefixes for systems that are initially on
            expected_on_topic_attr_name = {
                f"evt_{prefix}SystemState"
                for prefix in (
                    "azimuthDrivesThermal",
                    "elevationDrivesThermal",
                    "az0101CabinetThermal",
                    "modbusTemperatureControllers",
                    "mainCabinetThermal",
                )
            }
            for state_info in self.csc.system_state_dict.values():
                # The topic in state_info is a ControllerEvent;
                # we want the associated event in the remote
                system_state_kwargs = dict()
                topic = getattr(self.remote, state_info.topic.attr_name)
                if state_info.topic.attr_name in expected_on_topic_attr_name:
                    expected_power_state = PowerState.ON
                else:
                    expected_power_state = PowerState.OFF
                nskip = 0
                system_state_kwargs["powerState"] = expected_power_state
                if state_info.num_elements_power_state > 1:
                    system_state_kwargs["elementsPowerState"] = [
                        expected_power_state
                    ] * state_info.num_elements_power_state
                if state_info.num_motion_controller_state > 1:
                    nskip += 1
                    system_state_kwargs["motionControllerState"] = [
                        expected_power_state
                    ] * state_info.num_motion_controller_state
                if state_info.num_thermal > 0:
                    nskip += 1
                    if state_info.num_thermal == 1:
                        system_state_kwargs["trackAmbient"] = True
                    else:
                        system_state_kwargs["trackAmbient"] = [
                            True
                        ] * state_info.num_thermal
                for i in range(nskip):
                    await topic.next(flush=False, timeout=STD_TIMEOUT)
                data = await self.assert_next_sample(topic, **system_state_kwargs)
                if state_info.num_thermal == 1:
                    assert data.setTemperature == pytest.approx(
                        self.mock_controller.ambient_temperature
                    )
                elif state_info.num_thermal > 1:
                    numpy.testing.assert_allclose(
                        data.setTemperature,
                        [self.mock_controller.ambient_temperature]
                        * state_info.num_thermal,
                    )

            for topic in (
                self.remote.evt_mirrorCoverLocksMotionState,
                self.remote.evt_mirrorCoversMotionState,
            ):
                await self.assert_next_sample(
                    topic=topic,
                    state=DeployableMotionState.DEPLOYED,
                    elementsState=[DeployableMotionState.DEPLOYED] * 4,
                )

            expected_safety_data = {field: 0 for field in SAFETY_INTERLOCKS_FIELDS}
            await self.assert_next_sample(
                topic=self.remote.evt_safetyInterlocks, **expected_safety_data
            )

            # After the initial state the CSC asks to be commander.
            await self.assert_next_sample(
                topic=self.remote.evt_commander, commander=mtmount.Source.CSC
            )

            # Test xSystemState events after the CSC has enabled systems
            enabled_system_topic_names = {
                f"tel_{prefix}SystemState"
                for prefix in (
                    "azimuth",
                    "elevation",
                    "cameraCableWrap",
                    "azimuthCableWrap",
                    "oilSupplySystem",
                    "topEndChiller",
                )
            }
            for state_info in self.csc.system_state_dict.values():
                if state_info.topic.attr_name not in enabled_system_topic_names:
                    continue
                # The topic in state_info is a ControllerEvent;
                # we want the associated event in the remote
                system_state_kwargs = dict()
                topic = getattr(self.remote, state_info.topic.attr_name)
                expected_power_state = PowerState.ON
                system_state_kwargs["powerState"] = expected_power_state
                nskip = 0
                if state_info.num_elements_power_state > 1:
                    system_state_kwargs["elementsPowerState"] = [
                        expected_power_state
                    ] * state_info.num_elements_power_state
                if state_info.num_motion_controller_state > 1:
                    nskip += 1
                    system_state_kwargs["motionControllerState"] = [
                        expected_power_state
                    ] * state_info.num_motion_controller_state
                for i in range(nskip):
                    await topic.next(flush=False, timeout=STD_TIMEOUT)
                await self.assert_next_sample(topic, **system_state_kwargs)

            # Test initial telemetry
            data = await self.assert_next_sample(
                topic=self.remote.tel_azimuth,
                flush=False,
                actualPosition=0,
                actualVelocity=0,
                actualTorque=0,
                demandVelocity=0,
            )
            assert data.actualPosition == pytest.approx(
                mtmount.mock.INITIAL_POSITION[System.AZIMUTH]
            )
            assert data.demandPosition == pytest.approx(
                mtmount.mock.INITIAL_POSITION[System.AZIMUTH]
            )

            data = await self.assert_next_sample(
                topic=self.remote.tel_elevation,
                flush=False,
                actualVelocity=0,
                actualTorque=0,
                demandVelocity=0,
            )
            assert data.demandPosition == pytest.approx(
                mtmount.mock.INITIAL_POSITION[System.ELEVATION]
            )
            assert data.actualPosition == pytest.approx(
                mtmount.mock.INITIAL_POSITION[System.ELEVATION]
            )

            data = await self.assert_next_sample(
                topic=self.remote.tel_cameraCableWrap,
                flush=False,
                actualVelocity=0,
            )
            assert data.actualPosition == pytest.approx(
                mtmount.mock.INITIAL_POSITION[System.CAMERA_CABLE_WRAP]
            )

            # Disable the CSC and check xSystemState
            # for devices the CSC should power down
            await self.remote.cmd_disable.start(timeout=STD_TIMEOUT)

            disabled_system_topic_names = {
                f"{prefix}SystemState"
                for prefix in (
                    "azimuth",
                    "elevation",
                    "cameraCableWrap",
                    "azimuthCableWrap",
                )
            }
            for state_info in self.csc.system_state_dict.values():
                if state_info.topic.attr_name not in disabled_system_topic_names:
                    continue
                # The topic in state_info is a ControllerEvent;
                # we want the associated event in the remote
                system_state_kwargs = dict()
                topic = getattr(self.remote, state_info.topic.attr_name)
                expected_power_state = PowerState.OFF
                system_state_kwargs["powerState"] = expected_power_state
                nskip = 0
                if state_info.num_elements_power_state > 1:
                    system_state_kwargs["elementsPowerState"] = [
                        expected_power_state
                    ] * state_info.num_elements_power_state
                if state_info.num_motion_controller_state > 1:
                    nskip += 1
                    system_state_kwargs["motionControllerState"] = [
                        expected_power_state
                    ] * state_info.num_motion_controller_state
                for i in range(nskip):
                    await topic.next(flush=False, timeout=STD_TIMEOUT)
                await self.assert_next_sample(topic, **system_state_kwargs)

    async def test_standard_state_transitions(self):
        async with self.make_csc(initial_state=salobj.State.STANDBY):
            await self.check_standard_state_transitions(
                enabled_commands=(
                    "closeMirrorCovers",
                    "openMirrorCovers",
                    "disableCameraCableWrapFollowing",
                    "enableCameraCableWrapFollowing",
                    "homeBothAxes",
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

    @contextlib.asynccontextmanager
    async def fake_rotation_loop(self, rotator, position=0, velocity=0, interval=0.2):
        """Publish regular MTRotator rotation telemetry messages.

        Parameters
        ----------
        rotator : `Remote`
            MTRotator remote
        position : `float`, optional
            Position (deg)
        velocity : `float`, optional
            Velocity (deg/sec)
        interval : `float`, optional
            Interval between rotator updates (second)
        """

        async def _implement_loop(rotator, position, velocity, interval):
            while True:
                print("put fake rotation")
                await self.put_fake_rotation(
                    rotator=rotator, position=position, velocity=velocity
                )
                await asyncio.sleep(interval)

        loop_task = asyncio.create_task(
            _implement_loop(
                rotator=rotator, position=position, velocity=velocity, interval=interval
            )
        )
        try:
            yield
        finally:
            loop_task.cancel()

    async def put_fake_rotation(self, rotator, position=0, velocity=0, tai=None):
        """Publish one MTRotator rotation telemetry message.

        Parameters
        ----------
        rotator : `Remote`
            MTRotator remote
        position : `float`, optional
            Position (deg)
        velocity : `float`, optional
            Velocity (deg/sec)
        tai : `float` or `None`, optional
            Date as TAI unix seconds; current time if None
        """
        if tai is None:
            tai = utils.current_tai()
        await rotator.tel_rotation.set_write(
            demandPosition=0,
            demandVelocity=0,
            demandAcceleration=0,
            actualPosition=0,
            actualVelocity=0,
            timestamp=tai,
        )

    async def test_camera_cable_wrap_tracking(self):
        # Start the CSC in DISABLED state
        # so we can get the rotator remote running
        # before the camera cable wrap loop needs data from the rotator.
        # (I tried constructing the rotator before the CSC
        # but for some reason the CSC doesn't see data from the rotator
        # when I do that).
        async with self.make_csc(initial_state=salobj.State.DISABLED):
            await self.assert_next_sample(
                topic=self.remote.evt_cameraCableWrapFollowing, enabled=False
            )
            ccw_device = self.mock_controller.device_dict[System.CAMERA_CABLE_WRAP]
            ccw_actuator = ccw_device.actuator
            assert not ccw_device.power_on
            assert not ccw_device.enabled

            async with salobj.Controller(name="MTRotator") as rotator:
                await self.remote.cmd_enable.start(timeout=STD_TIMEOUT)
                assert ccw_device.power_on
                assert ccw_device.enabled

                await self.assert_next_sample(
                    topic=self.remote.evt_cameraCableWrapFollowing, enabled=True
                )
                assert ccw_device.tracking_enabled
                self.mock_controller.set_command_queue(
                    queue_heartbeat_commands=False, maxsize=0
                )

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
                    await self.put_fake_rotation(
                        rotator=rotator, position=position, velocity=velocity, tai=tai
                    )
                    command = await self.next_lowlevel_command()
                    delay = utils.current_tai() - tai
                    assert (
                        command.command_code
                        == mtmount.CommandCode.CAMERA_CABLE_WRAP_TRACK_TARGET
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

    async def test_command_failed(self):
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            mock_device = self.mock_controller.device_dict[System.MIRROR_COVERS]
            mock_device.fail_next_command = True

            # Open the mirror covers.
            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                await self.remote.cmd_closeMirrorCovers.start(
                    timeout=MIRROR_COVER_TIMEOUT
                )

    async def test_command_superseded(self):
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            # Slow the device way down so there's plenty of time
            # to supersede the move.
            mock_device = self.mock_controller.device_dict[System.MIRROR_COVERS]
            mock_device.actuator.speed /= 10

            # Start opening the mirror covers, then stop all motion.
            task = asyncio.create_task(
                self.remote.cmd_openMirrorCovers.start(timeout=MIRROR_COVER_TIMEOUT)
            )
            await asyncio.sleep(0.1)
            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
            # Note that SAL has no "superseded" CMD_x code,
            # so the CSC uses CMD_FAILED.
            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                await task

    async def test_unmocked_events(self):
        """Test low-level events not output by the mock controller
        (other than output the STATE_INFO command).

        These include:

        * deployablePlatformsMotionState
        * elevationLockingPinMotionState
        * safetyInterlocks
        * limits
        """
        async with self.make_csc(initial_state=salobj.State.DISABLED):
            await self.assert_next_sample(
                topic=self.remote.evt_deployablePlatformsMotionState,
                state=DeployableMotionState.RETRACTED,
                elementsState=[DeployableMotionState.RETRACTED] * 2,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_elevationLockingPinMotionState,
                state=ElevationLockingPinMotionState.UNLOCKED,
                elementsState=[ElevationLockingPinMotionState.UNLOCKED] * 2,
            )
            for topic in (
                self.remote.evt_elevationLimits,
                self.remote.evt_azimuthLimits,
                self.remote.evt_cameraCableWrapLimits,
            ):
                await self.assert_next_sample(topic=topic, limits=0)

            initial_safety_data = {field: 0 for field in SAFETY_INTERLOCKS_FIELDS}
            await self.assert_next_sample(
                topic=self.remote.evt_safetyInterlocks, **initial_safety_data
            )

            # Go in reverse order because the first entry is RETRACTED,
            # which is the current value
            for state in reversed(DeployableMotionState):
                await self.mock_controller.write_reply(
                    mtmount.mock.make_reply_dict(
                        id=mtmount.ReplyId.DEPLOYABLE_PLATFORMS_MOTION_STATE,
                        state=state,
                        elementsState=[state] * 2,
                    )
                )
                await self.assert_next_sample(
                    topic=self.remote.evt_deployablePlatformsMotionState,
                    state=state,
                    elementsState=[state] * 2,
                )

            # Go in reverse order in case the initial position is
            # the first entry (it will never be the last entry).
            for state in reversed(ElevationLockingPinMotionState):
                await self.mock_controller.write_reply(
                    mtmount.mock.make_reply_dict(
                        id=mtmount.ReplyId.ELEVATION_LOCKING_PIN_MOTION_STATE,
                        state=state,
                        elementsState=[state] * 2,
                    )
                )
                await self.assert_next_sample(
                    topic=self.remote.evt_elevationLockingPinMotionState,
                    state=state,
                    elementsState=[state] * 2,
                )

            for system, topic in (
                (System.ELEVATION, self.remote.evt_elevationLimits),
                (System.AZIMUTH, self.remote.evt_azimuthLimits),
                (
                    System.CAMERA_CABLE_WRAP,
                    self.remote.evt_cameraCableWrapLimits,
                ),
            ):
                value = system.value + 10  # arbitary positive value
                await self.mock_controller.write_reply(
                    mtmount.mock.make_reply_dict(
                        id=mtmount.ReplyId.LIMITS,
                        system=system,
                        limits=[value],
                    )
                )
                await self.assert_next_sample(topic=topic, limits=value)

            for i, field in enumerate(SAFETY_INTERLOCKS_FIELDS):
                value = i + 6  # arbitrary nonzero value
                safety_data = initial_safety_data.copy()
                safety_data[field] = value
                await self.mock_controller.write_reply(
                    mtmount.mock.make_reply_dict(
                        id=mtmount.ReplyId.SAFETY_INTERLOCKS,
                        **safety_data,
                    )
                )
                await self.assert_next_sample(
                    topic=self.remote.evt_safetyInterlocks, **safety_data
                )

    async def test_mirror_covers(self):
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            self.mock_controller.set_command_queue(
                queue_heartbeat_commands=False, maxsize=0
            )
            mirror_covers_device = self.mock_controller.device_dict[
                System.MIRROR_COVERS
            ]
            mirror_cover_locks_device = self.mock_controller.device_dict[
                System.MIRROR_COVER_LOCKS
            ]

            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoversMotionState,
                state=DeployableMotionState.DEPLOYED,
                elementsState=[DeployableMotionState.DEPLOYED] * 4,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoverLocksMotionState,
                state=DeployableMotionState.DEPLOYED,
                elementsState=[DeployableMotionState.DEPLOYED] * 4,
            )
            assert mirror_covers_device.motion_state() == DeployableMotionState.DEPLOYED
            assert (
                mirror_cover_locks_device.motion_state()
                == DeployableMotionState.DEPLOYED
            )

            # Open (retract) the mirror covers.
            t0 = time.monotonic()
            await self.remote.cmd_openMirrorCovers.start(timeout=MIRROR_COVER_TIMEOUT)
            dt = time.monotonic() - t0
            print(f"opening the mirror covers took {dt:0.2f} sec")
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoversMotionState,
                state=DeployableMotionState.RETRACTING,
                elementsState=[DeployableMotionState.RETRACTING] * 4,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoverLocksMotionState,
                state=DeployableMotionState.RETRACTING,
                elementsState=[DeployableMotionState.RETRACTING] * 4,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoversMotionState,
                state=DeployableMotionState.RETRACTED,
                elementsState=[DeployableMotionState.RETRACTED] * 4,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoverLocksMotionState,
                state=DeployableMotionState.RETRACTED,
                elementsState=[DeployableMotionState.RETRACTED] * 4,
            )
            assert (
                mirror_covers_device.motion_state() == DeployableMotionState.RETRACTED
            )
            assert (
                mirror_cover_locks_device.motion_state()
                == DeployableMotionState.RETRACTED
            )

            # Open the mirror covers again; this should be quick.
            t0 = time.monotonic()
            await self.remote.cmd_openMirrorCovers.start(timeout=STD_TIMEOUT)
            dt = time.monotonic() - t0
            print(f"opening the mirror covers again took {dt:0.2f} sec")
            assert (
                mirror_covers_device.motion_state() == DeployableMotionState.RETRACTED
            )
            assert (
                mirror_cover_locks_device.motion_state()
                == DeployableMotionState.RETRACTED
            )

            # Close (deploy) the mirror covers.
            t0 = time.monotonic()
            await self.remote.cmd_closeMirrorCovers.start(timeout=MIRROR_COVER_TIMEOUT)
            dt = time.monotonic() - t0
            print(f"closing the mirror covers took {dt:0.2f} sec")
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoversMotionState,
                state=DeployableMotionState.DEPLOYING,
                elementsState=[DeployableMotionState.DEPLOYING] * 4,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoverLocksMotionState,
                state=DeployableMotionState.DEPLOYING,
                elementsState=[DeployableMotionState.DEPLOYING] * 4,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoversMotionState,
                state=DeployableMotionState.DEPLOYED,
                elementsState=[DeployableMotionState.DEPLOYED] * 4,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoverLocksMotionState,
                state=DeployableMotionState.DEPLOYED,
                elementsState=[DeployableMotionState.DEPLOYED] * 4,
            )
            assert mirror_covers_device.motion_state() == DeployableMotionState.DEPLOYED
            assert (
                mirror_cover_locks_device.motion_state()
                == DeployableMotionState.DEPLOYED
            )

            # Close the mirror covers again;
            # the locks are retracted and engaged so it takes some time.
            t0 = time.monotonic()
            await self.remote.cmd_closeMirrorCovers.start(timeout=MIRROR_COVER_TIMEOUT)
            dt = time.monotonic() - t0
            print(f"closing the mirror covers again took {dt:0.2f} sec")
            assert mirror_covers_device.motion_state() == DeployableMotionState.DEPLOYED
            assert (
                mirror_cover_locks_device.motion_state()
                == DeployableMotionState.DEPLOYED
            )

    async def test_move_to_target(self):
        async with self.make_csc(initial_state=salobj.State.ENABLED), salobj.Controller(
            name="MTRotator"
        ) as rotator, self.fake_rotation_loop(rotator=rotator):
            await self.assert_next_sample(
                self.remote.evt_cameraCableWrapFollowing, enabled=False
            )
            await self.assert_next_sample(
                self.remote.evt_cameraCableWrapFollowing, enabled=True
            )
            await self.assert_axes_in_position(elevation=False, azimuth=False)
            await self.assert_target_cleared()
            await self.assert_next_sample(
                topic=self.remote.evt_azimuthToppleBlock, reverse=False, forward=False
            )
            await self.assert_next_sample(
                topic=self.remote.evt_elevationMotionState,
                state=AxisMotionState.STOPPED,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_azimuthMotionState,
                state=AxisMotionState.STOPPED,
            )

            mock_azimuth = self.mock_controller.device_dict[System.AZIMUTH]
            mock_elevation = self.mock_controller.device_dict[System.ELEVATION]

            tai = utils.current_tai()
            azimuth_pvt = mock_azimuth.actuator.path.at(tai)
            elevation_pvt = mock_elevation.actuator.path.at(tai)
            assert elevation_pvt.velocity == pytest.approx(0)
            assert azimuth_pvt.velocity == pytest.approx(0)

            # Move the axes to a specified position.
            # Use a move long enough that CCW following will time out
            # if tracking commands are blocked (DM-30990)
            # Make the elevation move a bit longer than azimuth,
            # to simplify computing the move duration.
            target_azimuth = azimuth_pvt.position + 4
            target_elevation = elevation_pvt.position + 5
            print(
                f"start test_moveToTarget(azimuth={target_azimuth:0.2f}, "
                f"elevation={target_elevation:0.2f})"
            )
            task = asyncio.create_task(
                self.remote.cmd_moveToTarget.set_start(
                    azimuth=target_azimuth,
                    elevation=target_elevation,
                    timeout=STD_TIMEOUT,
                )
            )
            # For point to point moves the target event is set twice:
            # once for elevation and once for azimuth.
            # The first event will have exactly one of elevation or azimuth
            # still set to NaN (note: ^ is bitwise xor, but it works as
            # logical xor for bools)
            data = await self.remote.evt_target.next(flush=False, timeout=STD_TIMEOUT)
            assert math.isnan(data.elevation) ^ math.isnan(data.azimuth)
            data = await self.remote.evt_target.next(flush=False, timeout=STD_TIMEOUT)
            assert not math.isnan(data.elevation) or math.isnan(data.azimuth)
            assert data.elevation == pytest.approx(target_elevation)
            assert data.azimuth == pytest.approx(target_azimuth)
            assert not task.done()

            await self.assert_next_sample(
                topic=self.remote.evt_elevationMotionState,
                state=AxisMotionState.MOVING_POINT_TO_POINT,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_azimuthMotionState,
                state=AxisMotionState.MOVING_POINT_TO_POINT,
            )

            # Check that the target event is received well before
            # the move finishes.
            duration = mock_azimuth.end_tai - utils.current_tai()
            print(f"axis move duration={duration:0.2f} sec")
            assert duration > 1

            await self.assert_next_sample(
                topic=self.remote.evt_elevationMotionState,
                state=AxisMotionState.STOPPED,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_azimuthMotionState,
                state=AxisMotionState.STOPPED,
            )
            await self.assert_axes_in_position(elevation=True, azimuth=True)
            await task
            tai = utils.current_tai()
            elevation_pvt = mock_elevation.actuator.path.at(tai)
            azimuth_pvt = mock_azimuth.actuator.path.at(tai)
            assert tai >= mock_elevation.actuator.path[-1].tai
            assert azimuth_pvt.position == pytest.approx(target_azimuth)
            assert elevation_pvt.position == pytest.approx(target_elevation)
            assert azimuth_pvt.velocity == pytest.approx(0)
            assert elevation_pvt.velocity == pytest.approx(0)

            await self.assert_next_sample(
                topic=self.remote.evt_azimuthToppleBlock, reverse=False, forward=True
            )

            # Check that the CCW is still following the rotator.
            data = self.remote.evt_cameraCableWrapFollowing.get()
            assert data.enabled

            # Check that out-of-bounds moves are rejected,
            # while leaving the mock devices enabled.
            for (azimuth, az_ok), (elevation, el_ok) in itertools.product(
                (
                    (mock_azimuth.cmd_limits.min_position - 0.001, False),
                    (0, True),
                    (mock_azimuth.cmd_limits.max_position + 0.001, False),
                ),
                (
                    (mock_elevation.cmd_limits.min_position - 0.001, False),
                    (45, True),
                    (mock_elevation.cmd_limits.max_position + 0.001, False),
                ),
            ):
                if az_ok and el_ok:
                    continue
                with salobj.assertRaisesAckError():
                    await self.remote.cmd_moveToTarget.set_start(
                        azimuth=azimuth,
                        elevation=elevation,
                        timeout=STD_TIMEOUT,
                    )
            for device in mock_azimuth, mock_elevation:
                assert device.power_on
                assert device.enabled

            # Check that the CCW is still following the rotator.
            data = self.remote.evt_cameraCableWrapFollowing.get()
            assert data.enabled

            # Check that putting the CSC into STANDBY state sends the axes
            # out of position (possibly one axis at a time)
            await self.remote.cmd_disable.start(timeout=STD_TIMEOUT)
            await self.remote.cmd_standby.start(timeout=STD_TIMEOUT)
            await self.assert_axes_in_position(elevation=False, azimuth=False)

    async def test_telemetry_reconnection(self):
        async with self.make_csc(initial_state=salobj.State.STANDBY):
            await self.assert_next_summary_state(salobj.State.STANDBY)
            with pytest.raises(asyncio.TimeoutError):
                await self.remote.tel_cameraCableWrap.next(
                    flush=True, timeout=NOTELEMETRY_TIMEOUT
                )

            await self.remote.cmd_start.start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(salobj.State.DISABLED)
            await self.remote.tel_cameraCableWrap.next(flush=True, timeout=STD_TIMEOUT)

            await self.remote.cmd_standby.start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(salobj.State.STANDBY)
            with pytest.raises(asyncio.TimeoutError):
                await self.remote.tel_cameraCableWrap.next(
                    flush=True, timeout=NOTELEMETRY_TIMEOUT
                )

            await self.remote.cmd_start.start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(salobj.State.DISABLED)
            await self.remote.tel_cameraCableWrap.next(flush=True, timeout=STD_TIMEOUT)

    async def test_tracking(self):
        async with self.make_csc(initial_state=salobj.State.ENABLED), salobj.Controller(
            name="MTRotator"
        ) as rotator, self.fake_rotation_loop(rotator=rotator):
            await self.assert_next_sample(
                self.remote.evt_cameraCableWrapFollowing, enabled=False
            )
            await self.assert_next_sample(
                self.remote.evt_cameraCableWrapFollowing, enabled=True
            )
            await self.assert_axes_in_position(elevation=False, azimuth=False)
            await self.assert_target_cleared()

            mock_azimuth = self.mock_controller.device_dict[System.AZIMUTH]
            mock_elevation = self.mock_controller.device_dict[System.ELEVATION]
            assert not mock_azimuth.tracking_enabled
            assert not mock_elevation.tracking_enabled

            tai = utils.current_tai()
            initial_azimuth = mock_azimuth.actuator.path.at(tai).position
            initial_elevation = mock_elevation.actuator.path.at(tai).position

            # Check that tracking is rejected if not enabled
            kwargs = self.make_track_target_kwargs(
                azimuth=initial_azimuth,
                elevation=initial_elevation,
                taiTime=tai,
            )
            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                await self.remote.cmd_trackTarget.set_start(
                    **kwargs, timeout=STD_TIMEOUT
                )

            # Cannot enable tracking until the axes are homed
            with pytest.raises(salobj.AckError):
                await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)

            # Home the axes, enable tracking and check mock axis controllers
            await self.remote.cmd_homeBothAxes.start(timeout=STD_TIMEOUT)
            await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
            assert mock_azimuth.tracking_enabled
            assert mock_elevation.tracking_enabled

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
            await self.assert_axes_in_position(
                elevation=True,
                azimuth=True,
                timeout=estimated_slew_time + STD_TIMEOUT,
            )
            dt_slew = time.monotonic() - t0
            tracking_task.cancel()
            print(f"Time to finish slew={dt_slew:0.2f} seconds")

            # Tracking should still be enabled
            for device in mock_azimuth, mock_elevation:
                assert device.power_on
                assert device.enabled
                assert device.tracking_enabled

            # Test that out-of-range tracking commands are rejected,
            # while leaving the mock devices enabled and tracking.
            for (
                (azimuth, az_ok),
                (elevation, el_ok),
                (azimuth_velocity, az_vel_ok),
                (elevation_velocity, el_vel_ok),
            ) in itertools.product(
                (
                    (mock_azimuth.cmd_limits.min_position - 0.001, False),
                    (0, True),
                    (mock_azimuth.cmd_limits.max_position + 0.001, False),
                ),
                (
                    (mock_elevation.cmd_limits.min_position - 0.001, False),
                    (45, True),
                    (mock_elevation.cmd_limits.max_position + 0.001, False),
                ),
                (
                    (-mock_azimuth.cmd_limits.max_velocity - 0.001, False),
                    (0, True),
                    (mock_azimuth.cmd_limits.max_velocity + 0.001, False),
                ),
                (
                    (-mock_elevation.cmd_limits.max_velocity - 0.001, False),
                    (0, True),
                    (mock_elevation.cmd_limits.max_velocity + 0.001, False),
                ),
            ):
                if az_ok and el_ok and az_vel_ok and el_vel_ok:
                    continue
                bad_kwargs = self.make_track_target_kwargs(
                    azimuth=azimuth,
                    elevation=elevation,
                    azimuthVelocity=azimuth_velocity,
                    elevationVelocity=elevation_velocity,
                )
                with salobj.assertRaisesAckError():
                    await self.remote.cmd_trackTarget.set_start(
                        **bad_kwargs, timeout=STD_TIMEOUT
                    )
            for device in mock_azimuth, mock_elevation:
                assert device.power_on
                assert device.enabled
                assert device.tracking_enabled

            # Check that the CCW is still following the rotator.
            data = self.remote.evt_cameraCableWrapFollowing.get()
            assert data.enabled

            # Disable tracking and check axis controllers
            await self.remote.cmd_stopTracking.start(timeout=STD_TIMEOUT)
            assert not mock_azimuth.tracking_enabled
            assert not mock_elevation.tracking_enabled

            # Check that both axes are no longer in position.
            await self.assert_axes_in_position(elevation=False, azimuth=False)

            # Check that the CCW is still following the rotator.
            data = self.remote.evt_cameraCableWrapFollowing.get()
            assert data.enabled

    async def track_target_loop(
        self, azimuth, elevation, azimuth_velocity, elevation_velocity
    ):
        """Provide a stream of trackTarget commands until cancelled."""
        # Slew and track until both axes are in position
        mock_azimuth = self.mock_controller.device_dict[System.AZIMUTH]
        mock_elevation = self.mock_controller.device_dict[System.ELEVATION]
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
            ccw_controller_settings_data = (
                await self.remote.evt_cameraCableWrapControllerSettings.next(
                    flush=False, timeout=STD_TIMEOUT
                )
            )
            ccw_device = self.mock_controller.device_dict[System.CAMERA_CABLE_WRAP]
            ccw_cmd_limits = ccw_device.cmd_limits
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
                self.mock_controller.set_command_queue(
                    queue_heartbeat_commands=False, maxsize=0
                )

                assert self.mock_controller.command_queue.empty()

                # Test regular tracking mode. CCW and MTRotator start in sync.

                # seconds until the CCW demand hits the position limit;
                # make it large enough that we get a few target events
                # before we reach the position limit.
                trunc_time = 0.5
                if do_max_limit:
                    position_limit = (
                        ccw_controller_settings_data.maxCmdPosition
                        - self.csc.limits_margin
                    )
                    velocity_limit = (
                        ccw_controller_settings_data.maxCmdVelocity
                        - self.csc.limits_margin
                    )
                else:
                    position_limit = (
                        ccw_controller_settings_data.minCmdPosition
                        + self.csc.limits_margin
                    )
                    velocity_limit = (
                        -ccw_controller_settings_data.maxCmdVelocity
                        + self.csc.limits_margin
                    )
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
                        == mtmount.CommandCode.CAMERA_CABLE_WRAP_TRACK_TARGET
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
                        assert ccw_target.velocity <= ccw_cmd_limits.max_velocity
                        assert ccw_target.position <= position_limit
                    else:
                        assert ccw_target.velocity >= -ccw_cmd_limits.max_velocity
                        assert ccw_target.position >= position_limit
                    print(
                        f"CCW target position={ccw_target.position}, velocity={ccw_target.velocity}"
                    )
                    if ccw_target.position == pytest.approx(position_limit):
                        break

                    await asyncio.sleep(0.1)
                    previous_tai = tai
