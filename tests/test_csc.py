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
import math
import time
import unittest

import numpy.testing

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
        config_dir=None,
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
            self.assertTrue(math.isnan(value))

    async def test_bin_script(self):
        await self.check_bin_script(
            name="MTMount",
            index=None,
            exe_name="run_mtmount.py",
        )

    def test_class_attributes(self):
        self.assertEqual(tuple(mtmount.MTMountCsc.valid_simulation_modes), (0, 1))
        self.assertEqual(mtmount.MTMountCsc.version, mtmount.__version__)

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

            settings = self.mock_controller.available_settings
            await self.assert_next_sample(
                topic=self.remote.evt_availableSettings,
                names=", ".join(item["name"] for item in settings),
                createdDates=", ".join(item["createdDate"].iso for item in settings),
                modifiedDates=", ".join(item["modifiedDate"].iso for item in settings),
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
                elementState=[DeployableMotionState.RETRACTED] * 2,
            )

            await self.assert_next_sample(
                topic=self.remote.evt_elevationLockingPinMotionState,
                state=ElevationLockingPinMotionState.UNLOCKED,
                elementState=[ElevationLockingPinMotionState.UNLOCKED] * 2,
            )

            # Test xSystemState events
            # Set of topic name prefixes for systems that are initially on
            expected_on_topic_name = {
                f"{prefix}SystemState"
                for prefix in (
                    "azimuthDrivesThermal",
                    "elevationDrivesThermal",
                    "az0101CabinetThermal",
                    "modbusTemperatureControllers",
                    "mainCabinet",
                    "mainAxesPowerSupply",
                )
            }
            for topic_info in self.csc.system_state_dict.values():
                # The topic in topic_info is a ControllerEvent;
                # we want the associated event in the remote
                system_state_kwargs = dict()
                topic = getattr(self.remote, topic_info.topic.attr_name)
                if topic.name in expected_on_topic_name:
                    expected_power_state = PowerState.ON
                else:
                    expected_power_state = PowerState.OFF
                nskip = 0
                system_state_kwargs["powerState"] = expected_power_state
                if topic_info.num_elements_power_state > 1:
                    system_state_kwargs["elementsPowerState"] = [
                        expected_power_state
                    ] * topic_info.num_elements_power_state
                if topic_info.num_motion_controller_state > 1:
                    nskip += 1
                    system_state_kwargs["motionControllerState"] = [
                        expected_power_state
                    ] * topic_info.num_motion_controller_state
                if topic_info.num_thermal > 0:
                    nskip += 1
                    if topic_info.num_thermal == 1:
                        system_state_kwargs["trackAmbient"] = True
                    else:
                        system_state_kwargs["trackAmbient"] = [
                            True
                        ] * topic_info.num_thermal
                for i in range(nskip):
                    await topic.next(flush=False, timeout=STD_TIMEOUT)
                data = await self.assert_next_sample(topic, **system_state_kwargs)
                if topic_info.num_thermal == 1:
                    self.assertAlmostEqual(
                        data.setTemperature, self.mock_controller.ambient_temperature
                    )
                elif topic_info.num_thermal > 1:
                    numpy.testing.assert_allclose(
                        data.setTemperature,
                        [self.mock_controller.ambient_temperature]
                        * topic_info.num_thermal,
                    )

            for topic in (
                self.remote.evt_mirrorCoverLocksMotionState,
                self.remote.evt_mirrorCoversMotionState,
            ):
                await self.assert_next_sample(
                    topic=topic,
                    state=DeployableMotionState.DEPLOYED,
                    elementState=[DeployableMotionState.DEPLOYED] * 4,
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
                f"{prefix}SystemState"
                for prefix in (
                    "azimuth",
                    "elevation",
                    "cameraCableWrap",
                    "azimuthCableWrap",
                    "oilSupplySystem",
                    "topEndChiller",
                )
            }
            for topic_info in self.csc.system_state_dict.values():
                if topic_info.topic.name not in enabled_system_topic_names:
                    continue
                # The topic in topic_info is a ControllerEvent;
                # we want the associated event in the remote
                system_state_kwargs = dict()
                topic = getattr(self.remote, topic_info.topic.attr_name)
                expected_power_state = PowerState.ON
                system_state_kwargs["powerState"] = expected_power_state
                nskip = 0
                if topic_info.num_elements_power_state > 1:
                    system_state_kwargs["elementsPowerState"] = [
                        expected_power_state
                    ] * topic_info.num_elements_power_state
                if topic_info.num_motion_controller_state > 1:
                    nskip += 1
                    system_state_kwargs["motionControllerState"] = [
                        expected_power_state
                    ] * topic_info.num_motion_controller_state
                for i in range(nskip):
                    await topic.next(flush=False, timeout=STD_TIMEOUT)
                await self.assert_next_sample(topic, **system_state_kwargs)

            # Test initial telemetry
            data = await self.assert_next_sample(
                topic=self.remote.tel_azimuth,
                flush=False,
                actualPosition=0,
                actualVelocity=0,
                actualAcceleration=0,
                actualTorque=0,
                demandVelocity=0,
            )
            self.assertAlmostEqual(
                data.actualPosition,
                mtmount.mock.INITIAL_POSITION[System.AZIMUTH],
            )
            self.assertAlmostEqual(
                data.demandPosition,
                mtmount.mock.INITIAL_POSITION[System.AZIMUTH],
            )

            data = await self.assert_next_sample(
                topic=self.remote.tel_elevation,
                flush=False,
                actualVelocity=0,
                actualAcceleration=0,
                actualTorque=0,
                demandVelocity=0,
            )
            self.assertAlmostEqual(
                data.demandPosition,
                mtmount.mock.INITIAL_POSITION[System.ELEVATION],
            )
            self.assertAlmostEqual(
                data.actualPosition,
                mtmount.mock.INITIAL_POSITION[System.ELEVATION],
            )

            data = await self.assert_next_sample(
                topic=self.remote.tel_cameraCableWrap,
                flush=False,
                actualVelocity=0,
                actualAcceleration=0,
            )
            self.assertAlmostEqual(
                data.actualPosition,
                mtmount.mock.INITIAL_POSITION[System.CAMERA_CABLE_WRAP],
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
            for topic_info in self.csc.system_state_dict.values():
                if topic_info.topic.name not in disabled_system_topic_names:
                    continue
                # The topic in topic_info is a ControllerEvent;
                # we want the associated event in the remote
                system_state_kwargs = dict()
                topic = getattr(self.remote, topic_info.topic.attr_name)
                expected_power_state = PowerState.OFF
                system_state_kwargs["powerState"] = expected_power_state
                nskip = 0
                if topic_info.num_elements_power_state > 1:
                    system_state_kwargs["elementsPowerState"] = [
                        expected_power_state
                    ] * topic_info.num_elements_power_state
                if topic_info.num_motion_controller_state > 1:
                    nskip += 1
                    system_state_kwargs["motionControllerState"] = [
                        expected_power_state
                    ] * topic_info.num_motion_controller_state
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
                self.put_fake_rotation(
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

    def put_fake_rotation(self, rotator, position=0, velocity=0, tai=None):
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
            tai = salobj.current_tai()
        rotator.tel_rotation.set_put(
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
            self.assertFalse(ccw_device.power_on)
            self.assertFalse(ccw_device.enabled)

            async with salobj.Controller(name="MTRotator") as rotator:
                await self.remote.cmd_enable.start(timeout=STD_TIMEOUT)
                self.assertTrue(ccw_device.power_on)
                self.assertTrue(ccw_device.enabled)

                await self.assert_next_sample(
                    topic=self.remote.evt_cameraCableWrapFollowing, enabled=True
                )
                self.assertTrue(ccw_device.tracking_enabled)
                self.mock_controller.set_command_queue(maxsize=0)

                self.assertTrue(self.mock_controller.command_queue.empty())

                # Test regular tracking mode. CCW and MTRotator start in sync.
                position0 = 0.0
                velocity = 0.1
                tai0 = salobj.current_tai()
                num_rotator_samples = 10
                previous_tai = 0
                for i in range(num_rotator_samples):
                    tai = salobj.current_tai()
                    # Work around non-monotonic clocks, which are
                    # sometimes seen when running Docker on macOS.
                    if tai < previous_tai:
                        tai = previous_tai + 0.001
                    dt = tai - tai0
                    position = position0 + velocity * dt
                    self.put_fake_rotation(
                        rotator=rotator, position=position, velocity=velocity, tai=tai
                    )
                    command = await self.next_lowlevel_command()
                    delay = salobj.current_tai() - tai
                    self.assertEqual(
                        command.command_code,
                        mtmount.CommandCode.CAMERA_CABLE_WRAP_TRACK,
                    )
                    desired_command_tai = (
                        tai + self.csc.config.camera_cable_wrap_advance_time
                    )
                    self.assertLessEqual(command.tai - desired_command_tai, delay)

                    # Check camera cable wrap telemetry;
                    # use a crude comparison because a new CCW tracking
                    # command will alter the path.
                    tel_ccw_data = await self.remote.tel_cameraCableWrap.next(
                        flush=True, timeout=STD_TIMEOUT
                    )
                    actual_segment = ccw_actuator.path.at(tel_ccw_data.timestamp)
                    self.assertAlmostEqual(
                        tel_ccw_data.actualPosition, actual_segment.position, delta=0.1
                    )
                    self.assertAlmostEqual(
                        tel_ccw_data.actualVelocity, actual_segment.velocity, delta=0.1
                    )
                    self.assertAlmostEqual(
                        tel_ccw_data.actualAcceleration,
                        actual_segment.acceleration,
                        delta=0.1,
                    )

                    await asyncio.sleep(0.1)
                    previous_tai = tai

                # Stop the camera cable wrap from following the rotator.
                await self.remote.cmd_disableCameraCableWrapFollowing.start(
                    timeout=STD_TIMEOUT
                )
                command = await self.next_lowlevel_command()
                self.assertEqual(
                    command.command_code,
                    mtmount.CommandCode.CAMERA_CABLE_WRAP_STOP,
                )
                self.assertTrue(ccw_device.enabled)
                self.assertFalse(ccw_device.tracking_enabled)
                self.assertTrue(self.mock_controller.command_queue.empty())

                # Check that rotator targets are ignored after
                # tracking is disabled.
                for i in range(2):
                    rotator.tel_rotation.set_put(
                        demandPosition=45,
                        demandVelocity=1,
                        demandAcceleration=0,
                        actualPosition=45,
                        actualVelocity=1,
                        timestamp=salobj.current_tai(),
                    )
                    await asyncio.sleep(0.1)
                self.assertTrue(self.mock_controller.command_queue.empty())

                # Restart the camera cable wrap following the rotator.
                await self.remote.cmd_enableCameraCableWrapFollowing.start(
                    timeout=STD_TIMEOUT
                )
                self.assertTrue(ccw_device.tracking_enabled)

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
                elementState=[DeployableMotionState.RETRACTED] * 2,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_elevationLockingPinMotionState,
                state=ElevationLockingPinMotionState.UNLOCKED,
                elementState=[ElevationLockingPinMotionState.UNLOCKED] * 2,
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
                        elementState=[state] * 2,
                    )
                )
                await self.assert_next_sample(
                    topic=self.remote.evt_deployablePlatformsMotionState,
                    state=state,
                    elementState=[state] * 2,
                )

            # Go in reverse order in case the initial position is
            # the first entry (it will never be the last entry).
            for state in reversed(ElevationLockingPinMotionState):
                await self.mock_controller.write_reply(
                    mtmount.mock.make_reply_dict(
                        id=mtmount.ReplyId.ELEVATION_LOCKING_PIN_MOTION_STATE,
                        state=state,
                        elementState=[state] * 2,
                    )
                )
                await self.assert_next_sample(
                    topic=self.remote.evt_elevationLockingPinMotionState,
                    state=state,
                    elementState=[state] * 2,
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
                        limits=value,
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
            self.mock_controller.set_command_queue(maxsize=0)
            mirror_covers_device = self.mock_controller.device_dict[
                System.MIRROR_COVERS
            ]
            mirror_cover_locks_device = self.mock_controller.device_dict[
                System.MIRROR_COVER_LOCKS
            ]

            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoversMotionState,
                state=DeployableMotionState.DEPLOYED,
                elementState=[DeployableMotionState.DEPLOYED] * 4,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoverLocksMotionState,
                state=DeployableMotionState.DEPLOYED,
                elementState=[DeployableMotionState.DEPLOYED] * 4,
            )
            self.assertEqual(
                mirror_covers_device.motion_state(), DeployableMotionState.DEPLOYED
            )
            self.assertEqual(
                mirror_cover_locks_device.motion_state(), DeployableMotionState.DEPLOYED
            )

            # Open (retract) the mirror covers.
            t0 = time.monotonic()
            await self.remote.cmd_openMirrorCovers.start(timeout=MIRROR_COVER_TIMEOUT)
            dt = time.monotonic() - t0
            print(f"opening the mirror covers took {dt:0.2f} sec")
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoversMotionState,
                state=DeployableMotionState.RETRACTING,
                elementState=[DeployableMotionState.RETRACTING] * 4,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoverLocksMotionState,
                state=DeployableMotionState.RETRACTING,
                elementState=[DeployableMotionState.RETRACTING] * 4,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoversMotionState,
                state=DeployableMotionState.RETRACTED,
                elementState=[DeployableMotionState.RETRACTED] * 4,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoverLocksMotionState,
                state=DeployableMotionState.RETRACTED,
                elementState=[DeployableMotionState.RETRACTED] * 4,
            )
            self.assertEqual(
                mirror_covers_device.motion_state(), DeployableMotionState.RETRACTED
            )
            self.assertEqual(
                mirror_cover_locks_device.motion_state(),
                DeployableMotionState.RETRACTED,
            )

            # Open the mirror covers again; this should be quick.
            t0 = time.monotonic()
            await self.remote.cmd_openMirrorCovers.start(timeout=STD_TIMEOUT)
            dt = time.monotonic() - t0
            print(f"opening the mirror covers again took {dt:0.2f} sec")
            self.assertEqual(
                mirror_covers_device.motion_state(), DeployableMotionState.RETRACTED
            )
            self.assertEqual(
                mirror_cover_locks_device.motion_state(),
                DeployableMotionState.RETRACTED,
            )

            # Close (deploy) the mirror covers.
            t0 = time.monotonic()
            await self.remote.cmd_closeMirrorCovers.start(timeout=MIRROR_COVER_TIMEOUT)
            dt = time.monotonic() - t0
            print(f"closing the mirror covers took {dt:0.2f} sec")
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoversMotionState,
                state=DeployableMotionState.DEPLOYING,
                elementState=[DeployableMotionState.DEPLOYING] * 4,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoverLocksMotionState,
                state=DeployableMotionState.DEPLOYING,
                elementState=[DeployableMotionState.DEPLOYING] * 4,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoversMotionState,
                state=DeployableMotionState.DEPLOYED,
                elementState=[DeployableMotionState.DEPLOYED] * 4,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_mirrorCoverLocksMotionState,
                state=DeployableMotionState.DEPLOYED,
                elementState=[DeployableMotionState.DEPLOYED] * 4,
            )
            self.assertEqual(
                mirror_covers_device.motion_state(), DeployableMotionState.DEPLOYED
            )
            self.assertEqual(
                mirror_cover_locks_device.motion_state(),
                DeployableMotionState.DEPLOYED,
            )

            # Close the mirror covers again;
            # the locks are retracted and engaged so it takes some time.
            t0 = time.monotonic()
            await self.remote.cmd_closeMirrorCovers.start(timeout=MIRROR_COVER_TIMEOUT)
            dt = time.monotonic() - t0
            print(f"closing the mirror covers again took {dt:0.2f} sec")
            self.assertEqual(
                mirror_covers_device.motion_state(), DeployableMotionState.DEPLOYED
            )
            self.assertEqual(
                mirror_cover_locks_device.motion_state(),
                DeployableMotionState.DEPLOYED,
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

            tai = salobj.current_tai()
            azimuth_pvt = mock_azimuth.actuator.path.at(tai)
            elevation_pvt = mock_elevation.actuator.path.at(tai)
            self.assertAlmostEqual(elevation_pvt.velocity, 0)
            self.assertAlmostEqual(azimuth_pvt.velocity, 0)

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
            self.assertTrue(math.isnan(data.elevation) ^ math.isnan(data.azimuth))
            data = await self.remote.evt_target.next(flush=False, timeout=STD_TIMEOUT)
            self.assertFalse(math.isnan(data.elevation) or math.isnan(data.azimuth))
            self.assertAlmostEqual(data.elevation, target_elevation)
            self.assertAlmostEqual(data.azimuth, target_azimuth)
            self.assertFalse(task.done())

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
            duration = mock_azimuth.end_tai - salobj.current_tai()
            print(f"axis move duration={duration:0.2f} sec")
            self.assertGreater(duration, 1)

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
            tai = salobj.current_tai()
            elevation_pvt = mock_elevation.actuator.path.at(tai)
            azimuth_pvt = mock_azimuth.actuator.path.at(tai)
            self.assertAlmostEqual(azimuth_pvt.position, target_azimuth)
            self.assertGreaterEqual(tai, mock_elevation.actuator.path[-1].tai)
            self.assertAlmostEqual(elevation_pvt.position, target_elevation)
            self.assertAlmostEqual(azimuth_pvt.velocity, 0)
            self.assertAlmostEqual(elevation_pvt.velocity, 0)

            await self.assert_next_sample(
                topic=self.remote.evt_azimuthToppleBlock, reverse=False, forward=True
            )

            # Check that the CCW is still following the rotator.
            data = self.remote.evt_cameraCableWrapFollowing.get()
            self.assertTrue(data.enabled)

            # Check that putting the CSC into STANDBY state sends the axes
            # out of position (possibly one axis at a time)
            await self.remote.cmd_disable.start(timeout=STD_TIMEOUT)
            await self.remote.cmd_standby.start(timeout=STD_TIMEOUT)
            await self.assert_axes_in_position(elevation=False, azimuth=False)

    async def test_telemetry_reconnection(self):
        async with self.make_csc(initial_state=salobj.State.STANDBY):
            await self.assert_next_summary_state(salobj.State.STANDBY)
            with self.assertRaises(asyncio.TimeoutError):
                await self.remote.tel_cameraCableWrap.next(
                    flush=True, timeout=NOTELEMETRY_TIMEOUT
                )

            await self.remote.cmd_start.start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(salobj.State.DISABLED)
            await self.remote.tel_cameraCableWrap.next(flush=True, timeout=STD_TIMEOUT)

            await self.remote.cmd_standby.start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(salobj.State.STANDBY)
            with self.assertRaises(asyncio.TimeoutError):
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
            self.assertFalse(mock_azimuth.tracking_enabled)
            self.assertFalse(mock_elevation.tracking_enabled)

            tai = salobj.current_tai()
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
            await self.assert_axes_in_position(
                elevation=True,
                azimuth=True,
                timeout=estimated_slew_time + STD_TIMEOUT,
            )
            dt_slew = time.monotonic() - t0
            tracking_task.cancel()
            print(f"Time to finish slew={dt_slew:0.2f} seconds")

            # Disable tracking and check axis controllers
            await self.remote.cmd_stopTracking.start(timeout=STD_TIMEOUT)
            self.assertFalse(mock_azimuth.tracking_enabled)
            self.assertFalse(mock_elevation.tracking_enabled)

            # Check that both axes are no longer in position.
            await self.assert_axes_in_position(elevation=False, azimuth=False)

            # Check that the CCW is still following the rotator.
            data = self.remote.evt_cameraCableWrapFollowing.get()
            self.assertTrue(data.enabled)

    async def track_target_loop(
        self, azimuth, elevation, azimuth_velocity, elevation_velocity
    ):
        """Provide a stream of trackTarget commands until cancelled."""
        # Slew and track until both axes are in position
        mock_azimuth = self.mock_controller.device_dict[System.AZIMUTH]
        mock_elevation = self.mock_controller.device_dict[System.ELEVATION]
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
            tel_el_data = await self.remote.tel_elevation.next(
                flush=True, timeout=STD_TIMEOUT
            )
            el_actual = elevation_actuator.path.at(tel_el_data.timestamp)
            el_target = elevation_actuator.target.at(tel_el_data.timestamp)
            self.assertAlmostEqual(tel_el_data.demandPosition, el_target.position)
            self.assertAlmostEqual(tel_el_data.demandVelocity, el_target.velocity)
            self.assertAlmostEqual(tel_el_data.actualPosition, el_actual.position)
            self.assertAlmostEqual(tel_el_data.actualVelocity, el_actual.velocity)

            tel_az_data = await self.remote.tel_azimuth.next(
                flush=True, timeout=STD_TIMEOUT
            )
            az_actual = azimuth_actuator.path.at(tel_az_data.timestamp)
            az_target = azimuth_actuator.target.at(tel_az_data.timestamp)
            self.assertAlmostEqual(tel_az_data.demandPosition, az_target.position)
            self.assertAlmostEqual(tel_az_data.demandVelocity, az_target.velocity)
            self.assertAlmostEqual(tel_az_data.actualPosition, az_actual.position)
            self.assertAlmostEqual(tel_az_data.actualVelocity, az_actual.velocity)

            previous_tai = tai
