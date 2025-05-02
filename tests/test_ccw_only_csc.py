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
import logging
import math
import pathlib
import unittest

import pytest
from lsst.ts import mtmount, salobj, utils
from lsst.ts.xml.enums.MTMount import (
    AxisMotionState,
    DeployableMotionState,
    ElevationLockingPinMotionState,
    PowerState,
    System,
    ThermalCommandState,
)
from numpy.testing import assert_array_equal

STD_TIMEOUT = 60  # standard command timeout (sec)

# Timeout for opening or closing mirror covers (sec).
MIRROR_COVER_TIMEOUT = STD_TIMEOUT + 2

# Timeout for reading telemetry that should not appear (sec).
NOTELEMETRY_TIMEOUT = 2

# Timeout for the CSC to react to a fault report
# from the low-level controller (sec).
FAULT_TIMEOUT = 2

# Timeout for constructing several remotes and controllers (sec).
LONG_TIMEOUT = 60

# Timeout for something that should happen quickly (sec).
SHORT_TIMEOUT = 5

TEST_CONFIG_DIR = pathlib.Path(__file__).parents[1] / "tests" / "data" / "config"

# Desired advance time for tracking commands (sec)
TRACK_ADVANCE_TIME = 0.15

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


class CcwOnlyCscTestCase(salobj.BaseCscTestCase, unittest.IsolatedAsyncioTestCase):
    def basic_make_csc(
        self, initial_state, config_dir, simulation_mode, internal_mock_controller
    ):
        if simulation_mode != 0 and not internal_mock_controller:
            mock_command_port = self.mock_controller.command_server.port
            mock_telemetry_port = self.mock_controller.telemetry_server.port
        else:
            mock_command_port = None
            mock_telemetry_port = None
        csc = mtmount.MTMountCcwOnlyCsc(
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
        log_level=None,
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
        log_level : `int` or `None`, optional
            Logging level, such as `logging.INFO`.
            If `None` then do not set the log level, leaving the default
            behavior of `SalInfo`: increase the log level to INFO.
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
            log_level=log_level,
        ):
            yield

    @contextlib.contextmanager
    def pretend_to_be_disabling_devices(self):
        """Set self.csc into a state where it thinks it is disabling devices.

        This should cause most commands to be rejected.
        """
        self.csc.disable_devices_task = asyncio.Future()
        try:
            yield
        finally:
            self.csc.disable_devices_task.cancel()

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

    async def test_axis_fault(self):
        """Test that azimuth, elevation, or camera cable wrap fault sends
        the CSC to fault.
        """
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, internal_mock_controller=False
        ):
            await self.assert_next_summary_state(salobj.State.ENABLED)
            for system in (
                System.MIRROR_COVERS,
                System.MIRROR_COVER_LOCKS,
            ):
                device = self.mock_controller.device_dict[system]
                device.power_on = False
                device.alarm_on = True
            # Give the CSC time to notice the change and NOT react to it.
            await asyncio.sleep(FAULT_TIMEOUT)
            assert self.csc.summary_state == salobj.State.ENABLED

        for system in (
            System.AZIMUTH,
            System.ELEVATION,
            System.CAMERA_CABLE_WRAP,
        ):
            with self.subTest(system=system):
                salobj.set_test_topic_subname()
                async with self.make_csc(
                    initial_state=salobj.State.ENABLED, internal_mock_controller=False
                ):
                    await self.assert_next_summary_state(salobj.State.ENABLED)
                    device = self.mock_controller.device_dict[system]
                    device.power_on = False
                    device.alarm_on = True
                    await self.assert_next_summary_state(salobj.State.FAULT)
                    assert self.csc.client.connected
                    assert not self.csc.should_be_commander

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
            assert not self.csc.client.connected
            assert not self.csc.should_be_commander

    async def test_initial_state(self):
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, internal_mock_controller=False
        ):
            await self.assert_next_sample(
                topic=self.remote.evt_softwareVersions,
                cscVersion=mtmount.__version__,
                subsystemVersions="",
            )
            await self.assert_next_sample(
                topic=self.remote.evt_connected, connected=False
            )
            await self.assert_next_sample(
                topic=self.remote.evt_connected, connected=True
            )
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
                minL1LimitEnabled=True,
                maxL1LimitEnabled=True,
                minL2LimitEnabled=True,
                maxL2LimitEnabled=True,
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
            await self.assert_next_sample(self.remote.evt_azimuthHomed, homed=False)
            await self.assert_next_sample(self.remote.evt_elevationHomed, homed=False)

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
                    "auxiliaryCabinetsThermal",
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
                        system_state_kwargs["trackAmbient"] = False
                    else:
                        system_state_kwargs["trackAmbient"] = [
                            False
                        ] * state_info.num_thermal
                for i in range(nskip):
                    await topic.next(flush=False, timeout=STD_TIMEOUT)

            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoversMotionState,
                state=DeployableMotionState.DEPLOYED,
                elementsState=[DeployableMotionState.DEPLOYED] * 4,
            )

            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoverLocksMotionState,
                state=DeployableMotionState.RETRACTED,
                elementsState=[DeployableMotionState.RETRACTED] * 4,
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
                    "applySettingsSet",
                    "closeMirrorCovers",
                    "disableCameraCableWrapFollowing",
                    "enableCameraCableWrapFollowing",
                    "homeBothAxes",
                    "moveToTarget",
                    "openMirrorCovers",
                    "park",
                    "restoreDefaultSettings",
                    "setThermal",
                    "startTracking",
                    "stop",
                    "stopTracking",
                    "trackTarget",
                    "unpark",
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
            taiTime = utils.current_tai() + TRACK_ADVANCE_TIME
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
    async def fake_rotation_loop(self, rotator, position=0, velocity=0, interval=0.1):
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

                # Test regular tracking mode. Camera cable wrap and MTRotator
                # start in sync.
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

                    # Check camera cable wrap telemetry; use a crude comparison
                    # because a new camera cable wrap tracking command will
                    # alter the path.
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

                # Check that enableCameraCableWrapFollowing while disabling
                # devices is rejected.
                with self.pretend_to_be_disabling_devices():
                    with salobj.assertRaisesAckError():
                        await self.remote.cmd_enableCameraCableWrapFollowing.start(
                            timeout=STD_TIMEOUT
                        )

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

    async def test_lose_command(self):
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            await self.assert_next_summary_state(salobj.State.ENABLED)
            await self.assert_next_sample(
                topic=self.remote.evt_commander, commander=mtmount.Source.NONE
            )
            await self.assert_next_sample(
                topic=self.remote.evt_commander, commander=mtmount.Source.CSC
            )
            # Change the commander to something other than CSC.
            # The CSC should transition to FAULT state.
            self.mock_controller.commander = mtmount.Source.EUI
            await self.mock_controller.write_commander()
            await self.assert_next_sample(
                topic=self.remote.evt_commander, commander=mtmount.Source.EUI
            )
            await self.assert_next_summary_state(salobj.State.FAULT)
            assert self.csc.client.connected
            assert not self.csc.should_be_commander

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
                state=DeployableMotionState.RETRACTED,
                elementsState=[DeployableMotionState.RETRACTED] * 4,
            )
            assert mirror_covers_device.motion_state() == DeployableMotionState.DEPLOYED
            assert (
                mirror_cover_locks_device.motion_state()
                == DeployableMotionState.RETRACTED
            )

            await self.remote.cmd_openMirrorCovers.start(timeout=MIRROR_COVER_TIMEOUT)

            assert (
                mirror_covers_device.motion_state() == DeployableMotionState.RETRACTED
            )
            assert (
                mirror_cover_locks_device.motion_state()
                == DeployableMotionState.DEPLOYED
            )

            await self.remote.cmd_closeMirrorCovers.start(timeout=MIRROR_COVER_TIMEOUT)
            assert mirror_covers_device.motion_state() == DeployableMotionState.DEPLOYED
            assert (
                mirror_cover_locks_device.motion_state()
                == DeployableMotionState.RETRACTED
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

            # Move the axes to a specified position. Use a move long enough
            # that camera cable wrap following will time out
            # if tracking commands are blocked (DM-30990).
            # Make the elevation move a bit longer than azimuth,
            # to simplify computing the move duration.
            target_azimuth = azimuth_pvt.position + 4
            target_elevation = elevation_pvt.position + 5
            print(
                f"start test_moveToTarget(azimuth={target_azimuth:0.2f}, "
                f"elevation={target_elevation:0.2f})"
            )
            with pytest.raises(salobj.AckError):
                await self.remote.cmd_moveToTarget.set_start(
                    azimuth=target_azimuth,
                    elevation=target_elevation,
                    timeout=STD_TIMEOUT,
                )

    async def test_set_thermal(self):
        system_ids_no_power_command = frozenset(
            (System.AUXILIARY_CABINETS_THERMAL, System.MAIN_CABINET_THERMAL)
        )

        def get_state_event_nelt(system_id):
            """Get the remote event that reports the state of a particular
            system and the number of thermal elements
            """
            state_info = self.csc.system_state_dict[system_id]
            attr_name = state_info.topic.attr_name
            return getattr(self.remote, attr_name), state_info.num_thermal

        def expand_value(value, nelt):
            if nelt == 1:
                return value
            else:
                return [value] * nelt

        async with self.make_csc(initial_state=salobj.State.ENABLED):
            thermal_devices = {
                system_id: self.mock_controller.device_dict[system_id]
                for system_id in self.mock_controller.chiller_state_nelts
            }
            for system_id, device in thermal_devices.items():
                assert device.track_setpoint
                assert device.setpoint == 0
                assert device.ambient_setpoint == 0
                assert device.temperature == pytest.approx(
                    0, abs=device.temperature_slop
                )
                state_topic, nelt = get_state_event_nelt(system_id)
                expected_power_state = (
                    PowerState.ON
                    if system_id in system_ids_no_power_command
                    else PowerState.OFF
                )
                # This topic is output twice: once with setTemperature=nan,
                # before the CSC gets that data,
                # then with the setTemperature=0, the initial value
                # used by mock thermal devices.
                data = await self.assert_next_sample(
                    topic=state_topic,
                    powerState=expected_power_state,
                    trackAmbient=expand_value(False, nelt),
                )
                assert_array_equal(data.setTemperature, expand_value(math.nan, nelt))
                await self.assert_next_sample(
                    topic=state_topic,
                    powerState=expected_power_state,
                    trackAmbient=expand_value(False, nelt),
                    setTemperature=expand_value(0, nelt),
                )

            with pytest.raises(salobj.AckError):
                for state in (ThermalCommandState.OFF, ThermalCommandState.ON):
                    await self.remote.cmd_setThermal.set_start(
                        topEndChillerState=state, timeout=STD_TIMEOUT
                    )

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

    async def test_telemetry_timeout(self):
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            await self.assert_next_summary_state(salobj.State.ENABLED)
            await self.assert_next_sample(
                self.remote.evt_telemetryConnected, connected=False
            )
            await self.assert_next_sample(
                self.remote.evt_telemetryConnected, connected=True
            )
            # Kill the low-level telemetry publishing loop
            # and wait for things to go sour.
            self.mock_controller.telemetry_loop_task.cancel()
            await self.assert_next_sample(
                self.remote.evt_telemetryConnected, connected=False
            )
            await self.assert_next_summary_state(salobj.State.FAULT)
            assert self.csc.client.connected
            assert not self.csc.should_be_commander

    async def test_tracking(self):
        async with salobj.Controller(
            name="MTRotator"
        ) as rotator, self.fake_rotation_loop(rotator=rotator), self.make_csc(
            initial_state=salobj.State.ENABLED, log_level=15
        ):
            await self.assert_next_sample(
                self.remote.evt_cameraCableWrapFollowing, enabled=False
            )
            await self.assert_next_sample(
                self.remote.evt_cameraCableWrapFollowing, enabled=True
            )
            await self.assert_next_sample(self.remote.evt_azimuthHomed, homed=False)
            await self.assert_next_sample(self.remote.evt_elevationHomed, homed=False)
            await self.assert_axes_in_position(elevation=False, azimuth=False)
            await self.assert_target_cleared()

            mock_azimuth = self.mock_controller.device_dict[System.AZIMUTH]
            mock_elevation = self.mock_controller.device_dict[System.ELEVATION]
            assert not mock_azimuth.tracking_enabled
            assert not mock_elevation.tracking_enabled

            tai = utils.current_tai() + TRACK_ADVANCE_TIME
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

            # Check that homeBothAxes while disabling devices is rejected.
            with self.pretend_to_be_disabling_devices():
                with salobj.assertRaisesAckError():
                    await self.remote.cmd_homeBothAxes.start(timeout=STD_TIMEOUT)

            with pytest.raises(salobj.AckError):
                await self.remote.cmd_homeBothAxes.start(timeout=STD_TIMEOUT)

    async def track_target_loop(
        self, azimuth, elevation, azimuth_velocity, elevation_velocity
    ):
        """Provide a stream of trackTarget commands until cancelled."""
        mock_azimuth = self.mock_controller.device_dict[System.AZIMUTH]
        mock_elevation = self.mock_controller.device_dict[System.ELEVATION]
        assert mock_azimuth.tracking_enabled
        assert mock_elevation.tracking_enabled
        azimuth_actuator = mock_azimuth.actuator
        elevation_actuator = mock_elevation.actuator

        initial_tai = utils.current_tai()
        previous_tai = 0
        self.remote.evt_target.flush()
        while True:
            await asyncio.sleep(0.1)
            tai = utils.current_tai() + TRACK_ADVANCE_TIME
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

                # Test regular tracking mode. Camera cable wrap
                # and MTRotator start in sync.

                # Seconds until the camera cable wrap demand reaches the
                # position limit; make it large enough that we get
                # a few target events before we reach the position limit.
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
                        f"Camera cable wrap target position={ccw_target.position}, "
                        f"velocity={ccw_target.velocity}"
                    )
                    if ccw_target.position == pytest.approx(position_limit):
                        break

                    await asyncio.sleep(0.1)
                    previous_tai = tai
