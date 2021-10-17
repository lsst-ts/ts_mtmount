# This file is part of ts_mtmount.
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
import logging
import unittest

import pytest

from lsst.ts import utils
from lsst.ts import mtmount
from lsst.ts.idl.enums.MTMount import AxisMotionState, DeployableMotionState, System

STD_TIMEOUT = 2  # Timeout for short operations (sec)
# Padding for the time limit returned by device do_methods
TIMEOUT_PADDING = 1


class TrivialMockController:
    def __init__(self):
        self.command_dict = {}
        self.log = logging.getLogger()


class MockDevicesTestCase(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.controller = TrivialMockController()
        axis_devices = [
            mtmount.mock.AxisDevice(
                controller=self.controller,
                system_id=system_id,
                start_position=mtmount.mock.INITIAL_POSITION[system_id],
            )
            for system_id in mtmount.LimitsDict
        ]
        devices = axis_devices + [
            mtmount.mock.MainAxesPowerSupplyDevice(controller=self.controller),
            mtmount.mock.MirrorCoverLocksDevice(controller=self.controller),
            mtmount.mock.MirrorCoversDevice(controller=self.controller),
            mtmount.mock.OilSupplySystemDevice(controller=self.controller),
            mtmount.mock.TopEndChillerDevice(controller=self.controller),
        ]
        self.device_dict = {device.system_id: device for device in devices}

    def get_command_class(self, command_code_name):
        command_code = getattr(mtmount.CommandCode, command_code_name.upper())
        return mtmount.commands.CommandDict[command_code]

    async def run_command(
        self, command, min_timeout=None, should_be_superseded=False, should_fail=False
    ):
        """Run a command that should succeed and wait for replies.

        Parameters
        ----------
        command : `mtmount.commands.Command`
            Command to execute.
        min_timeout : `float` or `None`, optional
            Minimum timeout. Must be specified if the command method
            returns a timeout, None if not.
        should_be_superseded : `bool`, optional
            Set True if the command should start OK but then be superseded.
        should_fail : `bool`, optional
            True if the command should start OK but then fail.
        """
        print(f"run_command({command})")
        do_method = self.controller.command_dict[command.command_code]
        try:
            timeout_task = do_method(command)
        except Exception as e:
            print(f"command failed: {e!r}")
            raise

        if timeout_task is None:
            timeout = None
            task = None
            assert min_timeout is None, f"min_timeout={min_timeout} but no timeout seen"
            return

        timeout, task = timeout_task
        if min_timeout is None:
            self.fail(f"min_timeout=None but timeout={timeout}")
        assert timeout >= min_timeout

        try:
            await asyncio.wait_for(task, timeout=timeout + STD_TIMEOUT)
            if should_fail:
                self.fail(f"Command {command} succeeded but should_fail true")
        except (mtmount.CommandSupersededException, asyncio.CancelledError) as e:
            if not should_be_superseded:
                self.fail(
                    f"Command {command} superseded, but should_be_superseded false: {e!r}"
                )
        except Exception as e:
            if not should_fail:
                self.fail(f"Command {command} failed, but should_fail false: {e!r}")

    async def test_base_commands(self):
        for system_id, device in self.device_dict.items():
            with self.subTest(system_id=device.system_id.name):
                if system_id in (
                    System.MIRROR_COVERS,
                    System.MIRROR_COVER_LOCKS,
                ):
                    drive = -1
                else:
                    drive = None
                await self.check_base_commands(device=device, drive=drive)

    async def test_mirror_cover_locks(self):
        device = self.device_dict[System.MIRROR_COVER_LOCKS]

        await self.check_deployable_device(
            device=device,
            power_on_command=mtmount.commands.MirrorCoverLocksPower(drive=-1, on=True),
            deploy_command=mtmount.commands.MirrorCoverLocksMoveAll(
                drive=-1, deploy=True
            ),
            retract_command=mtmount.commands.MirrorCoverLocksMoveAll(
                drive=-1, deploy=False
            ),
            stop_command=mtmount.commands.MirrorCoverLocksStop(drive=-1),
            start_deployed=True,
            move_min_timeout=0.5,
            power_on_min_timeout=None,
        )

    async def test_mirror_covers(self):
        device = self.device_dict[System.MIRROR_COVERS]

        await self.check_deployable_device(
            device=device,
            power_on_command=mtmount.commands.MirrorCoversPower(drive=-1, on=True),
            deploy_command=mtmount.commands.MirrorCoversDeploy(drive=-1),
            retract_command=mtmount.commands.MirrorCoversRetract(drive=-1),
            stop_command=mtmount.commands.MirrorCoversStop(drive=-1),
            start_deployed=True,
            move_min_timeout=0.5,
            power_on_min_timeout=None,
        )

    async def test_oil_supply_system(self):
        device = self.device_dict[System.OIL_SUPPLY_SYSTEM]

        # Test the OilSupplySystemPower command
        assert not device.power_on
        assert not device.cooling_on
        assert not device.oil_on
        assert not device.main_pump_on

        await self.run_command(
            command=mtmount.commands.OilSupplySystemPower(on=True), min_timeout=900
        )
        assert device.power_on
        assert device.cooling_on
        assert device.oil_on
        assert device.main_pump_on

        await self.run_command(
            command=mtmount.commands.OilSupplySystemPower(on=False), min_timeout=0
        )
        assert not device.power_on
        assert not device.cooling_on
        assert not device.oil_on
        assert not device.main_pump_on

        # Test the subsystem power commands.
        subsystem_command_dict = {
            "cooling": mtmount.commands.OilSupplySystemPowerCooling,
            "oil": mtmount.commands.OilSupplySystemPowerOil,
            "main_pump": mtmount.commands.OilSupplySystemPowerMainPump,
        }

        def get_value_dict():
            """Get the current value of all subsystem attributes.

            The returned data is a dict of name: value,
            where the name entries are the keys of subsystem_command_dict.
            """
            return {
                name: getattr(device, f"{name}_on") for name in subsystem_command_dict
            }

        for name, command_class in subsystem_command_dict.items():
            expected_values = get_value_dict()

            # Turn this subsystem on
            expected_values[name] = True
            min_timeout = (
                900
                if command_class is mtmount.commands.OilSupplySystemPowerCooling
                else None
            )
            await self.run_command(command_class(on=True), min_timeout=min_timeout)
            values = get_value_dict()
            assert values == expected_values

            # Turn this subsystem off
            expected_values[name] = False
            min_timeout = (
                0
                if command_class is mtmount.commands.OilSupplySystemPowerCooling
                else None
            )
            await self.run_command(command_class(on=False), min_timeout=min_timeout)
            values = get_value_dict()
            assert values == expected_values

    async def test_top_end_chiller(self):
        device = self.device_dict[System.TOP_END_CHILLER]
        initial_track_ambient = device.track_ambient
        initial_temperature = device.temperature
        temperature1 = 5.1  # An arbitrary value

        # track_ambient should fail if power is off
        # and the state should remain unchanged.
        assert not device.power_on
        assert device.temperature != temperature1
        with pytest.raises(RuntimeError):
            await self.run_command(
                mtmount.commands.TopEndChillerTrackAmbient(
                    on=True, temperature=temperature1
                )
            )
        with pytest.raises(RuntimeError):
            await self.run_command(
                mtmount.commands.TopEndChillerTrackAmbient(
                    on=False, temperature=temperature1
                )
            )
        assert not device.power_on
        assert device.track_ambient == initial_track_ambient
        assert device.temperature == initial_temperature

        await self.run_command(mtmount.commands.TopEndChillerPower(on=True))
        assert device.power_on
        assert device.track_ambient == initial_track_ambient
        assert device.temperature == initial_temperature

        await self.run_command(
            mtmount.commands.TopEndChillerTrackAmbient(
                on=True, temperature=temperature1
            )
        )
        assert device.power_on
        assert device.track_ambient
        assert device.temperature == pytest.approx(temperature1)

        temperature2 = 0.12  # A different arbitrary value
        await self.run_command(
            mtmount.commands.TopEndChillerTrackAmbient(
                on=False, temperature=temperature2
            )
        )
        assert device.power_on
        assert not device.track_ambient
        assert device.temperature == pytest.approx(temperature2)

    async def test_axis_devices(self):
        for system_id in (
            System.AZIMUTH,
            System.ELEVATION,
            System.CAMERA_CABLE_WRAP,
        ):
            with self.subTest(system_id=system_id.name):
                device = self.device_dict[system_id]
                await self.check_axis_device(device)

    async def test_command_failure(self):
        device = self.device_dict[System.MIRROR_COVERS]
        await self.run_command(mtmount.commands.MirrorCoversPower(drive=-1, on=True))
        assert device.power_on
        device.fail_next_command = True
        await self.run_command(
            mtmount.commands.MirrorCoversDeploy(drive=-1),
            min_timeout=0,
            should_fail=True,
        )

    async def check_axis_device(self, device):
        is_elaz = device.system_id in (
            System.AZIMUTH,
            System.ELEVATION,
        )

        assert not device.power_on
        assert not device.enabled
        assert not device.has_target
        assert not device.moving_point_to_point

        short_command_names = [
            "drive_enable",
            "drive_reset",
            "enable_tracking",
            "move",
            "power",
            "stop",
            "track",
            "reset_alarm",
        ]
        if is_elaz:
            short_command_names.append("home")
        dev_prefix = f"{device.system_id.name}_"
        command_codes = {
            name: getattr(mtmount.CommandCode, dev_prefix + name.upper())
            for name in short_command_names
        }
        command_classes = {
            name: mtmount.commands.CommandDict[cmd_id]
            for name, cmd_id in command_codes.items()
        }
        slow_command_codes = {command_codes[name] for name in ("move", "track")}

        # A batch of useful commands.
        drive_enable_off_command = command_classes["drive_enable"](drive=-1, on=False)
        drive_enable_on_command = command_classes["drive_enable"](drive=-1, on=True)
        drive_reset_command = command_classes["drive_reset"](drive=-1)
        if is_elaz:
            # Elevation and azimuth have a home command,
            # and tracking cannot be paused (so the enable tracking command
            # has no "on" parameter)
            home_command = command_classes["home"]()
            enable_tracking_on_command = command_classes["enable_tracking"]()
            pause_tracking_command = None
        else:
            # The camera cable wrap has no home command,
            # and tracking can be paused.
            home_command = None
            enable_tracking_on_command = command_classes["enable_tracking"](on=True)
            pause_tracking_command = command_classes["enable_tracking"](on=False)
        power_off_command = command_classes["power"](on=False)
        power_on_command = command_classes["power"](on=True)
        stop_command = command_classes["stop"]()
        reset_alarm_command = command_classes["reset_alarm"]()
        move_command_class = command_classes["move"]
        track_command_class = command_classes["track"]

        # drive_reset fails if off
        assert not device.power_on
        assert not device.enabled
        assert not device.tracking_enabled
        with pytest.raises(RuntimeError):
            await self.run_command(drive_reset_command)
        assert not device.power_on
        assert not device.enabled
        assert not device.tracking_enabled

        # Power on the device; this should not enable the drive.
        await self.run_command(power_on_command)
        assert device.power_on
        assert device.enabled
        assert not device.tracking_enabled
        assert not device.has_target

        # Disable the drive
        await self.run_command(drive_enable_off_command)
        assert device.power_on
        assert not device.enabled
        assert not device.tracking_enabled
        assert not device.has_target

        # Most commands should fail if drive not enabled.
        fail_if_not_enabled_commands = [
            enable_tracking_on_command,
            stop_command,
            home_command,
            move_command_class(position=device.actuator.min_position + 1),
            track_command_class(
                position=device.actuator.min_position + 1,
                velocity=0,
                tai=utils.current_tai(),
            ),
        ]
        if is_elaz:
            fail_if_not_enabled_commands.append(home_command)
        else:
            fail_if_not_enabled_commands.append(pause_tracking_command)
        for command in fail_if_not_enabled_commands:
            if command is None:
                continue
            with self.subTest(command=str(command)):
                min_timeout = 0 if command.command_code in slow_command_codes else None
                with pytest.raises(RuntimeError):
                    await self.run_command(command, min_timeout=min_timeout)
                assert device.power_on
                assert not device.enabled
                assert not device.tracking_enabled
                assert not device.has_target

        # Enable the drives
        await self.run_command(drive_enable_on_command)
        assert device.power_on
        assert device.enabled
        assert not device.tracking_enabled
        assert not device.has_target

        # Do a point to point move
        assert device.motion_state() == AxisMotionState.STOPPED
        start_segment = device.actuator.path.at(utils.current_tai())
        assert start_segment.velocity == 0
        start_position = start_segment.position
        end_position = start_position + 2
        move_command = move_command_class(position=end_position)
        task = asyncio.create_task(self.run_command(move_command, min_timeout=0.5))

        await asyncio.sleep(0.1)  # give command time to start
        assert device.motion_state() == AxisMotionState.MOVING_POINT_TO_POINT
        assert device.actuator.target.position == pytest.approx(end_position)
        assert device.actuator.target.velocity == 0
        segment = device.actuator.path.at(utils.current_tai())
        assert abs(segment.velocity) > 0.01
        assert device.has_target
        assert device.moving_point_to_point
        assert device.point_to_point_target == pytest.approx(end_position)

        await task
        assert device.motion_state() == AxisMotionState.STOPPED
        end_segment = device.actuator.path.at(utils.current_tai())
        assert end_segment.velocity == pytest.approx(0)
        assert end_segment.position == pytest.approx(end_position)
        assert device.has_target
        assert device.point_to_point_target == pytest.approx(end_position)

        # Start homing or a big point to point move, then stop the axes
        # (Homing is a point to point move, and it's slow).
        if is_elaz:
            slow_move_command = home_command
            end_position = device.home_position
        else:
            start_position = start_segment.position
            end_position = start_position + 10
            slow_move_command = move_command_class(position=end_position)
        task = asyncio.create_task(
            self.run_command(
                slow_move_command, min_timeout=0.5, should_be_superseded=True
            )
        )

        await asyncio.sleep(0.1)
        assert device.point_to_point_target == pytest.approx(end_position)
        assert device.motion_state() == AxisMotionState.MOVING_POINT_TO_POINT

        await self.run_command(stop_command)
        await task
        stop_end_tai = device.actuator.path[-1].tai
        stop_duration = stop_end_tai - utils.current_tai()
        # Check STOPPING state; the specified tai can be any time
        # earlier than the end time of the stop.
        assert device.motion_state(stop_end_tai - 0.1) == AxisMotionState.STOPPING
        await asyncio.sleep(stop_duration)
        assert not device.has_target
        assert device.point_to_point_target == pytest.approx(end_position)
        assert device.motion_state() == AxisMotionState.STOPPED

        # The tracking command should fail when not in tracking mode.
        track_command = track_command_class(
            position=device.actuator.min_position + 1,
            velocity=0,
            tai=utils.current_tai(),
        )
        with pytest.raises(RuntimeError):
            await self.run_command(track_command)

        # Turn on tracking mode and confirm that non-tracking mode
        # commands are rejected.
        assert not device.tracking_enabled
        assert device.power_on
        assert device.enabled
        await self.run_command(enable_tracking_on_command)
        assert device.power_on
        assert device.enabled
        assert device.tracking_enabled
        assert not device.tracking_paused
        assert not device.has_target
        assert not device.moving_point_to_point
        assert device.motion_state() == AxisMotionState.STOPPED

        non_tracking_mode_commands = [
            home_command,
            move_command,
        ]
        for command in non_tracking_mode_commands:
            if command is None:
                continue
            with pytest.raises(RuntimeError):
                await self.run_command(command, min_timeout=0)

        # Issue a few tracking commands; confirm that the actuator path
        # is updated accordingly.
        tai0 = utils.current_tai()
        previous_tai = tai0
        position0 = device.actuator.path[-1].position + 1
        for i in range(3):
            tai = utils.current_tai()
            if tai < previous_tai:
                tai = previous_tai + 0.01
            previous_tai = tai
            dt = tai - tai0
            velocity = 0.1 + 0.001 * i
            position = position0 + velocity * dt
            track_command = track_command_class(
                position=position,
                velocity=velocity,
                tai=tai,
            )
            await self.run_command(track_command)
            assert device.actuator.target.position == pytest.approx(position)
            assert device.actuator.target.velocity == pytest.approx(velocity)
            assert device.actuator.target.tai == pytest.approx(tai, abs=1e-5)
            assert device.has_target
            assert device.motion_state(tai) == AxisMotionState.TRACKING
            await asyncio.sleep(0.1)

        # Wait for tracking to time out; add some time to deal with
        # clock errors in macOS Docker.
        await asyncio.sleep(mtmount.mock.MAX_TRACKING_DELAY + 0.2)
        assert device.alarm_on
        assert not device.power_on
        assert not device.enabled
        assert not device.tracking_enabled
        assert not device.tracking_paused
        assert not device.has_target
        assert device.motion_state(tai) == AxisMotionState.STOPPED

        # Re-enable tracking and supply one tracking update
        await self.run_command(reset_alarm_command)
        await self.run_command(power_on_command)
        await self.run_command(enable_tracking_on_command)
        assert device.power_on
        assert device.enabled
        assert device.tracking_enabled
        assert not device.tracking_paused
        assert not device.has_target

        await self.run_command(
            track_command_class(
                position=device.actuator.path[-1].position,
                velocity=1,
                tai=utils.current_tai(),
            )
        )
        assert device.has_target
        assert device.motion_state(tai) == AxisMotionState.TRACKING

        # If camera cable wrap, pause tracking and check state
        if not is_elaz:
            # Pause tracking
            await self.run_command(pause_tracking_command)
            assert device.power_on
            assert device.enabled
            assert device.tracking_enabled
            assert device.tracking_paused
            assert not device.has_target
            # Note: there may eventually be an AxisMotionState for paused
            # tracking but Tekniker does not report it yet.
            assert (
                device.motion_state(tai=device.actuator.path[-1].tai - 0.01)
                == AxisMotionState.STOPPING
            )
            assert (
                device.motion_state(tai=device.actuator.path[-1].tai + 0.01)
                == AxisMotionState.TRACKING_PAUSED
            )

            # Make sure the tracking timer does not fire
            await asyncio.sleep(mtmount.mock.MAX_TRACKING_DELAY + 0.2)
            assert device.power_on
            assert device.enabled
            assert device.tracking_enabled
            assert device.tracking_paused

            # Un-pause tracking
            await self.run_command(enable_tracking_on_command)
            assert device.power_on
            assert device.enabled
            assert device.tracking_enabled
            assert not device.tracking_paused
            assert not device.has_target
            await self.run_command(
                track_command_class(
                    position=device.actuator.path[-1].position,
                    velocity=1,
                    tai=utils.current_tai(),
                )
            )
            assert device.motion_state(tai) == AxisMotionState.TRACKING

        # Check that stop disables tracking
        await self.run_command(stop_command)
        assert device.power_on
        assert device.enabled
        assert not device.tracking_enabled
        assert not device.tracking_paused
        assert not device.has_target
        stop_end_tai = device.actuator.path[-1].tai
        assert device.motion_state(stop_end_tai - 0.01) == AxisMotionState.STOPPING
        assert device.motion_state(stop_end_tai + 0.01) == AxisMotionState.STOPPED

        # Check that drive_disable disables tracking
        # but does not turn off power.
        await self.run_command(enable_tracking_on_command)
        assert device.power_on
        assert device.enabled
        assert device.tracking_enabled
        assert not device.has_target

        await self.run_command(drive_enable_off_command)
        assert device.power_on
        assert not device.enabled
        assert not device.tracking_enabled
        assert not device.has_target

        # Check that drive_reset disables the drive and tracking
        await self.run_command(drive_enable_on_command)
        assert device.power_on
        assert device.enabled
        assert not device.tracking_enabled
        assert not device.has_target

        await self.run_command(enable_tracking_on_command)
        assert device.power_on
        assert device.enabled
        assert device.tracking_enabled
        assert not device.has_target

        await self.run_command(drive_reset_command)
        assert device.power_on
        assert not device.enabled
        assert not device.tracking_enabled
        assert not device.has_target

        # Check that power off disables everything
        await self.run_command(power_on_command)
        assert device.power_on
        assert device.enabled
        assert not device.tracking_enabled
        assert not device.has_target

        await self.run_command(enable_tracking_on_command)
        assert device.power_on
        assert device.enabled
        assert device.tracking_enabled
        assert not device.has_target

        await self.run_command(power_off_command)
        assert not device.power_on
        assert not device.enabled
        assert not device.tracking_enabled
        assert not device.has_target

    async def check_base_commands(self, device, drive=None):
        """Test the power and reset_alarm commands common to all devices.

        Parameters
        ----------
        device : `mtmount.mock.BaseDevice`
            Mock device to test.
        drive : `int` or `None`
            Value for ``drive`` argument of power commands.
            If `None` then the ``drive`` argument is not provided.
        """
        assert not device.power_on
        assert not device.alarm_on

        device_prefix = device.system_id.name
        power_command_class = self.get_command_class(f"{device_prefix}_POWER")
        reset_alarm_command_class = self.get_command_class(
            f"{device_prefix}_RESET_ALARM"
        )
        if drive is not None:
            power_kwargs = dict(drive=drive)
        else:
            power_kwargs = {}
        power_command_on = power_command_class(on=True, **power_kwargs)
        power_command_off = power_command_class(on=False, **power_kwargs)
        reset_alarm_command = reset_alarm_command_class()

        min_on_timeout = {
            System.MAIN_AXES_POWER_SUPPLY: 5,
            System.OIL_SUPPLY_SYSTEM: 900,
        }.get(device.system_id, None)
        min_off_timeout = {
            System.MAIN_AXES_POWER_SUPPLY: 0,
            System.OIL_SUPPLY_SYSTEM: 0,
        }.get(device.system_id, None)

        await self.run_command(power_command_on, min_timeout=min_on_timeout)
        assert device.power_on
        assert not device.alarm_on

        device.alarm_on = False
        await self.run_command(reset_alarm_command)
        assert device.power_on
        assert not device.alarm_on

        await self.run_command(power_command_off, min_timeout=min_off_timeout)
        assert not device.power_on
        assert not device.alarm_on

    async def check_deployable_device(
        self,
        device,
        power_on_command,
        deploy_command,
        retract_command,
        stop_command,
        start_deployed,
        move_min_timeout,
        power_on_min_timeout,
    ):
        def assert_motion_state(motion_state, tai=None):
            if tai is None:
                tai = utils.current_tai()
            position = device.actuator.position(tai)
            velocity = device.actuator.velocity(tai)
            actual_motion_state = device.motion_state(tai)
            assert actual_motion_state == motion_state

            if motion_state == DeployableMotionState.DEPLOYED:
                assert position == pytest.approx(device.deployed_position)
                assert velocity == 0
            elif motion_state == DeployableMotionState.RETRACTED:
                assert position == pytest.approx(device.retracted_position)
                assert velocity == 0
            elif motion_state == DeployableMotionState.LOST:
                assert position != pytest.approx(device.deployed_position)
                assert position != pytest.approx(device.retracted_position)
                assert velocity == 0
            elif motion_state == DeployableMotionState.DEPLOYING:
                assert velocity != 0
                if device.deployed_position > device.retracted_position:
                    assert velocity > 0
                else:
                    assert velocity < 0
            elif motion_state == DeployableMotionState.RETRACTING:
                assert velocity != 0
                if device.deployed_position > device.retracted_position:
                    assert velocity < 0
                else:
                    assert velocity > 0
            else:
                self.fail(f"Unrecognized motion_state={motion_state}")

        assert_motion_state(
            DeployableMotionState.DEPLOYED
            if start_deployed
            else DeployableMotionState.RETRACTED
        )

        # Test that moves fail if not powered on.
        # This failure happens before the command starts running,
        # so the should_fail argument is not relevant.
        with pytest.raises(RuntimeError):
            await self.run_command(
                command=deploy_command,
                min_timeout=move_min_timeout,
            )
        with pytest.raises(RuntimeError):
            await self.run_command(
                command=retract_command,
                min_timeout=move_min_timeout,
            )

        await self.run_command(
            command=power_on_command, min_timeout=power_on_min_timeout
        )

        # Deploy the device, if not already deployed.
        if not start_deployed:
            await self.run_command(
                command=deploy_command,
                min_timeout=move_min_timeout,
            )
            assert_motion_state(DeployableMotionState.DEPLOYED)

        # Stop should have no effect while halted
        await self.run_command(command=stop_command, min_timeout=None)
        assert_motion_state(DeployableMotionState.DEPLOYED)

        # Start retracting and check motion state
        task = asyncio.create_task(
            self.run_command(command=retract_command, min_timeout=move_min_timeout)
        )
        await asyncio.sleep(0.1)  # Let the move begin
        assert_motion_state(DeployableMotionState.RETRACTING)

        await task
        assert_motion_state(DeployableMotionState.RETRACTED)

        # Start deploying and stop partway
        task = asyncio.create_task(
            self.run_command(
                command=deploy_command,
                min_timeout=move_min_timeout,
                should_be_superseded=True,
            )
        )
        await asyncio.sleep(0.1)  # Let the move begin
        assert_motion_state(DeployableMotionState.DEPLOYING)
        await self.run_command(command=stop_command, min_timeout=None)
        assert_motion_state(DeployableMotionState.LOST)

        await task
