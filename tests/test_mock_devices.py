# This file is part of ts_MTMount.
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

from lsst.ts import salobj
from lsst.ts import MTMount
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
            MTMount.mock.AxisDevice(
                controller=self.controller,
                system_id=system_id,
                start_position=MTMount.mock.INITIAL_POSITION[system_id],
            )
            for system_id in MTMount.LimitsDict
        ]
        devices = axis_devices + [
            MTMount.mock.MainAxesPowerSupplyDevice(controller=self.controller),
            MTMount.mock.MirrorCoverLocksDevice(controller=self.controller),
            MTMount.mock.MirrorCoversDevice(controller=self.controller),
            MTMount.mock.OilSupplySystemDevice(controller=self.controller),
            MTMount.mock.TopEndChillerDevice(controller=self.controller),
        ]
        self.device_dict = {device.system_id: device for device in devices}

    def get_command_class(self, command_code_name):
        command_code = getattr(MTMount.CommandCode, command_code_name.upper())
        return MTMount.commands.CommandDict[command_code]

    async def run_command(
        self, command, min_timeout=None, should_be_superseded=False, should_fail=False
    ):
        """Run a command that should succeed and wait for replies.

        Parameters
        ----------
        command : `MTMount.commands.Command`
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
            self.assertIsNone(
                min_timeout, f"min_timeout={min_timeout} but no timeout seen"
            )
            return

        timeout, task = timeout_task
        if min_timeout is None:
            self.fail(f"min_timeout=None but timeout={timeout}")
        self.assertGreaterEqual(timeout, min_timeout)

        try:
            await asyncio.wait_for(task, timeout=timeout + STD_TIMEOUT)
            if should_fail:
                self.fail(f"Command {command} succeeded but should_fail true")
        except (MTMount.CommandSupersededException, asyncio.CancelledError) as e:
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
            power_on_command=MTMount.commands.MirrorCoverLocksPower(drive=-1, on=True),
            deploy_command=MTMount.commands.MirrorCoverLocksMoveAll(
                drive=-1, deploy=True
            ),
            retract_command=MTMount.commands.MirrorCoverLocksMoveAll(
                drive=-1, deploy=False
            ),
            stop_command=MTMount.commands.MirrorCoverLocksStop(drive=-1),
            start_deployed=True,
            move_min_timeout=0.5,
            power_on_min_timeout=None,
        )

    async def test_mirror_covers(self):
        device = self.device_dict[System.MIRROR_COVERS]

        await self.check_deployable_device(
            device=device,
            power_on_command=MTMount.commands.MirrorCoversPower(drive=-1, on=True),
            deploy_command=MTMount.commands.MirrorCoversDeploy(drive=-1),
            retract_command=MTMount.commands.MirrorCoversRetract(drive=-1),
            stop_command=MTMount.commands.MirrorCoversStop(drive=-1),
            start_deployed=True,
            move_min_timeout=0.5,
            power_on_min_timeout=None,
        )

    async def test_oil_supply_system(self):
        device = self.device_dict[System.OIL_SUPPLY_SYSTEM]

        # Test the OilSupplySystemPower command
        self.assertFalse(device.power_on)
        self.assertFalse(device.cooling_on)
        self.assertFalse(device.oil_on)
        self.assertFalse(device.main_pump_on)

        await self.run_command(
            command=MTMount.commands.OilSupplySystemPower(on=True), min_timeout=900
        )
        self.assertTrue(device.power_on)
        self.assertTrue(device.cooling_on)
        self.assertTrue(device.oil_on)
        self.assertTrue(device.main_pump_on)

        await self.run_command(
            command=MTMount.commands.OilSupplySystemPower(on=False), min_timeout=0
        )
        self.assertFalse(device.power_on)
        self.assertFalse(device.cooling_on)
        self.assertFalse(device.oil_on)
        self.assertFalse(device.main_pump_on)

        # Test the subsystem power commands.
        subsystem_command_dict = {
            "cooling": MTMount.commands.OilSupplySystemPowerCooling,
            "oil": MTMount.commands.OilSupplySystemPowerOil,
            "main_pump": MTMount.commands.OilSupplySystemPowerMainPump,
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
                if command_class is MTMount.commands.OilSupplySystemPowerCooling
                else None
            )
            await self.run_command(command_class(on=True), min_timeout=min_timeout)
            values = get_value_dict()
            self.assertEqual(values, expected_values)

            # Turn this subsystem off
            expected_values[name] = False
            min_timeout = (
                0
                if command_class is MTMount.commands.OilSupplySystemPowerCooling
                else None
            )
            await self.run_command(command_class(on=False), min_timeout=min_timeout)
            values = get_value_dict()
            self.assertEqual(values, expected_values)

    async def test_top_end_chiller(self):
        device = self.device_dict[System.TOP_END_CHILLER]
        initial_track_ambient = device.track_ambient
        initial_temperature = device.temperature
        temperature1 = 5.1  # An arbitrary value

        # track_ambient should fail if power is off
        # and the state should remain unchanged.
        self.assertFalse(device.power_on)
        self.assertNotEqual(device.temperature, temperature1)
        with self.assertRaises(RuntimeError):
            await self.run_command(
                MTMount.commands.TopEndChillerTrackAmbient(
                    on=True, temperature=temperature1
                )
            )
        with self.assertRaises(RuntimeError):
            await self.run_command(
                MTMount.commands.TopEndChillerTrackAmbient(
                    on=False, temperature=temperature1
                )
            )
        self.assertFalse(device.power_on)
        self.assertEqual(device.track_ambient, initial_track_ambient)
        self.assertEqual(device.temperature, initial_temperature)

        await self.run_command(MTMount.commands.TopEndChillerPower(on=True))
        self.assertTrue(device.power_on)
        self.assertEqual(device.track_ambient, initial_track_ambient)
        self.assertEqual(device.temperature, initial_temperature)

        await self.run_command(
            MTMount.commands.TopEndChillerTrackAmbient(
                on=True, temperature=temperature1
            )
        )
        self.assertTrue(device.power_on)
        self.assertTrue(device.track_ambient)
        self.assertAlmostEqual(device.temperature, temperature1)

        temperature2 = 0.12  # A different arbitrary value
        await self.run_command(
            MTMount.commands.TopEndChillerTrackAmbient(
                on=False, temperature=temperature2
            )
        )
        self.assertTrue(device.power_on)
        self.assertFalse(device.track_ambient)
        self.assertAlmostEqual(device.temperature, temperature2)

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
        await self.run_command(MTMount.commands.MirrorCoversPower(drive=-1, on=True))
        self.assertTrue(device.power_on)
        device.fail_next_command = True
        await self.run_command(
            MTMount.commands.MirrorCoversDeploy(drive=-1),
            min_timeout=0,
            should_fail=True,
        )

    async def check_axis_device(self, device):
        is_elaz = device.system_id in (
            System.AZIMUTH,
            System.ELEVATION,
        )

        self.assertFalse(device.power_on)
        self.assertFalse(device.enabled)
        self.assertFalse(device.has_target)
        self.assertFalse(device.moving_point_to_point)

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
            name: getattr(MTMount.CommandCode, dev_prefix + name.upper())
            for name in short_command_names
        }
        command_classes = {
            name: MTMount.commands.CommandDict[cmd_id]
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
        self.assertFalse(device.power_on)
        self.assertFalse(device.enabled)
        self.assertFalse(device.tracking_enabled)
        with self.assertRaises(RuntimeError):
            await self.run_command(drive_reset_command)
        self.assertFalse(device.power_on)
        self.assertFalse(device.enabled)
        self.assertFalse(device.tracking_enabled)

        # Power on the device; this should not enable the drive.
        await self.run_command(power_on_command)
        self.assertTrue(device.power_on)
        self.assertTrue(device.enabled)
        self.assertFalse(device.tracking_enabled)
        self.assertFalse(device.has_target)

        # Disable the drive
        await self.run_command(drive_enable_off_command)
        self.assertTrue(device.power_on)
        self.assertFalse(device.enabled)
        self.assertFalse(device.tracking_enabled)
        self.assertFalse(device.has_target)

        # Most commands should fail if drive not enabled.
        fail_if_not_enabled_commands = [
            enable_tracking_on_command,
            stop_command,
            home_command,
            move_command_class(position=device.actuator.min_position + 1),
            track_command_class(
                position=device.actuator.min_position + 1,
                velocity=0,
                tai=salobj.current_tai(),
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
                with self.assertRaises(RuntimeError):
                    await self.run_command(command, min_timeout=min_timeout)
                self.assertTrue(device.power_on)
                self.assertFalse(device.enabled)
                self.assertFalse(device.tracking_enabled)
                self.assertFalse(device.has_target)

        # Enable the drives
        await self.run_command(drive_enable_on_command)
        self.assertTrue(device.power_on)
        self.assertTrue(device.enabled)
        self.assertFalse(device.tracking_enabled)
        self.assertFalse(device.has_target)

        # Do a point to point move
        self.assertEqual(device.motion_state(), AxisMotionState.STOPPED)
        start_segment = device.actuator.path.at(salobj.current_tai())
        self.assertEqual(start_segment.velocity, 0)
        start_position = start_segment.position
        end_position = start_position + 2
        move_command = move_command_class(position=end_position)
        task = asyncio.create_task(self.run_command(move_command, min_timeout=0.5))

        await asyncio.sleep(0.1)  # give command time to start
        self.assertEqual(device.motion_state(), AxisMotionState.MOVING_POINT_TO_POINT)
        self.assertAlmostEqual(device.actuator.target.position, end_position)
        self.assertEqual(device.actuator.target.velocity, 0)
        segment = device.actuator.path.at(salobj.current_tai())
        self.assertGreater(abs(segment.velocity), 0.01)
        self.assertTrue(device.has_target)
        self.assertTrue(device.moving_point_to_point)
        self.assertAlmostEqual(device.point_to_point_target, end_position)

        await task
        self.assertEqual(device.motion_state(), AxisMotionState.STOPPED)
        end_segment = device.actuator.path.at(salobj.current_tai())
        self.assertAlmostEqual(end_segment.velocity, 0)
        self.assertAlmostEqual(end_segment.position, end_position)
        self.assertTrue(device.has_target)
        self.assertAlmostEqual(device.point_to_point_target, end_position)

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
        self.assertAlmostEqual(device.point_to_point_target, end_position)
        self.assertEqual(device.motion_state(), AxisMotionState.MOVING_POINT_TO_POINT)

        await self.run_command(stop_command)
        await task
        stop_end_tai = device.actuator.path[-1].tai
        stop_duration = stop_end_tai - salobj.current_tai()
        # Check STOPPING state; the specified tai can be any time
        # earlier than the end time of the stop.
        self.assertEqual(
            device.motion_state(stop_end_tai - 0.1), AxisMotionState.STOPPING
        )
        await asyncio.sleep(stop_duration)
        self.assertFalse(device.has_target)
        self.assertAlmostEqual(device.point_to_point_target, end_position)
        self.assertEqual(device.motion_state(), AxisMotionState.STOPPED)

        # The tracking command should fail when not in tracking mode.
        track_command = track_command_class(
            position=device.actuator.min_position + 1,
            velocity=0,
            tai=salobj.current_tai(),
        )
        with self.assertRaises(RuntimeError):
            await self.run_command(track_command)

        # Turn on tracking mode and confirm that non-tracking mode
        # commands are rejected.
        self.assertFalse(device.tracking_enabled)
        self.assertTrue(device.power_on)
        self.assertTrue(device.enabled)
        await self.run_command(enable_tracking_on_command)
        self.assertTrue(device.power_on)
        self.assertTrue(device.enabled)
        self.assertTrue(device.tracking_enabled)
        self.assertFalse(device.tracking_paused)
        self.assertFalse(device.has_target)
        self.assertFalse(device.moving_point_to_point)
        self.assertEqual(device.motion_state(), AxisMotionState.STOPPED)

        non_tracking_mode_commands = [
            home_command,
            move_command,
        ]
        for command in non_tracking_mode_commands:
            if command is None:
                continue
            with self.assertRaises(RuntimeError):
                await self.run_command(command, min_timeout=0)

        # Issue a few tracking commands; confirm that the actuator path
        # is updated accordingly.
        tai0 = salobj.current_tai()
        previous_tai = tai0
        position0 = device.actuator.path[-1].position + 1
        for i in range(3):
            tai = salobj.current_tai()
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
            self.assertAlmostEqual(device.actuator.target.position, position)
            self.assertAlmostEqual(device.actuator.target.velocity, velocity)
            self.assertAlmostEqual(device.actuator.target.tai, tai, delta=1e-5)
            self.assertTrue(device.has_target)
            self.assertEqual(device.motion_state(tai), AxisMotionState.TRACKING)
            await asyncio.sleep(0.1)

        # Wait for tracking to time out; add some time to deal with
        # clock errors in macOS Docker.
        await asyncio.sleep(MTMount.mock.MAX_TRACKING_DELAY + 0.2)
        self.assertTrue(device.alarm_on)
        self.assertFalse(device.power_on)
        self.assertFalse(device.enabled)
        self.assertFalse(device.tracking_enabled)
        self.assertFalse(device.tracking_paused)
        self.assertFalse(device.has_target)
        self.assertEqual(device.motion_state(tai), AxisMotionState.STOPPED)

        # Re-enable tracking and supply one tracking update
        await self.run_command(reset_alarm_command)
        await self.run_command(power_on_command)
        await self.run_command(enable_tracking_on_command)
        self.assertTrue(device.power_on)
        self.assertTrue(device.enabled)
        self.assertTrue(device.tracking_enabled)
        self.assertFalse(device.tracking_paused)
        self.assertFalse(device.has_target)

        await self.run_command(
            track_command_class(
                position=device.actuator.path[-1].position,
                velocity=1,
                tai=salobj.current_tai(),
            )
        )
        self.assertTrue(device.has_target)
        self.assertEqual(device.motion_state(tai), AxisMotionState.TRACKING)

        # If camera cable wrap, pause tracking and check state
        if not is_elaz:
            # Pause tracking
            await self.run_command(pause_tracking_command)
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertTrue(device.tracking_enabled)
            self.assertTrue(device.tracking_paused)
            self.assertFalse(device.has_target)
            # Note: there may eventually be an AxisMotionState for paused
            # tracking but Tekniker does not report it yet.
            self.assertEqual(
                device.motion_state(tai=device.actuator.path[-1].tai - 0.01),
                AxisMotionState.STOPPING,
            )
            self.assertEqual(
                device.motion_state(tai=device.actuator.path[-1].tai + 0.01),
                AxisMotionState.TRACKING_PAUSED,
            )

            # Make sure the tracking timer does not fire
            await asyncio.sleep(MTMount.mock.MAX_TRACKING_DELAY + 0.2)
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertTrue(device.tracking_enabled)
            self.assertTrue(device.tracking_paused)

            # Un-pause tracking
            await self.run_command(enable_tracking_on_command)
            self.assertTrue(device.power_on)
            self.assertTrue(device.enabled)
            self.assertTrue(device.tracking_enabled)
            self.assertFalse(device.tracking_paused)
            self.assertFalse(device.has_target)
            await self.run_command(
                track_command_class(
                    position=device.actuator.path[-1].position,
                    velocity=1,
                    tai=salobj.current_tai(),
                )
            )
            self.assertEqual(device.motion_state(tai), AxisMotionState.TRACKING)

        # Check that stop disables tracking
        await self.run_command(stop_command)
        self.assertTrue(device.power_on)
        self.assertTrue(device.enabled)
        self.assertFalse(device.tracking_enabled)
        self.assertFalse(device.tracking_paused)
        self.assertFalse(device.has_target)
        stop_end_tai = device.actuator.path[-1].tai
        self.assertEqual(
            device.motion_state(stop_end_tai - 0.01), AxisMotionState.STOPPING
        )
        self.assertEqual(
            device.motion_state(stop_end_tai + 0.01), AxisMotionState.STOPPED
        )

        # Check that drive_disable disables tracking
        # but does not turn off power.
        await self.run_command(enable_tracking_on_command)
        self.assertTrue(device.power_on)
        self.assertTrue(device.enabled)
        self.assertTrue(device.tracking_enabled)
        self.assertFalse(device.has_target)

        await self.run_command(drive_enable_off_command)
        self.assertTrue(device.power_on)
        self.assertFalse(device.enabled)
        self.assertFalse(device.tracking_enabled)
        self.assertFalse(device.has_target)

        # Check that drive_reset disables the drive and tracking
        await self.run_command(drive_enable_on_command)
        self.assertTrue(device.power_on)
        self.assertTrue(device.enabled)
        self.assertFalse(device.tracking_enabled)
        self.assertFalse(device.has_target)

        await self.run_command(enable_tracking_on_command)
        self.assertTrue(device.power_on)
        self.assertTrue(device.enabled)
        self.assertTrue(device.tracking_enabled)
        self.assertFalse(device.has_target)

        await self.run_command(drive_reset_command)
        self.assertTrue(device.power_on)
        self.assertFalse(device.enabled)
        self.assertFalse(device.tracking_enabled)
        self.assertFalse(device.has_target)

        # Check that power off disables everything
        await self.run_command(power_on_command)
        self.assertTrue(device.power_on)
        self.assertTrue(device.enabled)
        self.assertFalse(device.tracking_enabled)
        self.assertFalse(device.has_target)

        await self.run_command(enable_tracking_on_command)
        self.assertTrue(device.power_on)
        self.assertTrue(device.enabled)
        self.assertTrue(device.tracking_enabled)
        self.assertFalse(device.has_target)

        await self.run_command(power_off_command)
        self.assertFalse(device.power_on)
        self.assertFalse(device.enabled)
        self.assertFalse(device.tracking_enabled)
        self.assertFalse(device.has_target)

    async def check_base_commands(self, device, drive=None):
        """Test the power and reset_alarm commands common to all devices.

        Parameters
        ----------
        device : `MTMount.mock.BaseDevice`
            Mock device to test.
        drive : `int` or `None`
            Value for ``drive`` argument of power commands.
            If `None` then the ``drive`` argument is not provided.
        """
        self.assertFalse(device.power_on)
        self.assertFalse(device.alarm_on)

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
        self.assertTrue(device.power_on)
        self.assertFalse(device.alarm_on)

        device.alarm_on = False
        await self.run_command(reset_alarm_command)
        self.assertTrue(device.power_on)
        self.assertFalse(device.alarm_on)

        await self.run_command(power_command_off, min_timeout=min_off_timeout)
        self.assertFalse(device.power_on)
        self.assertFalse(device.alarm_on)

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
                tai = salobj.current_tai()
            position = device.actuator.position(tai)
            velocity = device.actuator.velocity(tai)
            actual_motion_state = device.motion_state(tai)
            self.assertEqual(actual_motion_state, motion_state)

            if motion_state == DeployableMotionState.DEPLOYED:
                self.assertAlmostEqual(position, device.deployed_position)
                self.assertEqual(velocity, 0)
            elif motion_state == DeployableMotionState.RETRACTED:
                self.assertAlmostEqual(position, device.retracted_position)
                self.assertEqual(velocity, 0)
            elif motion_state == DeployableMotionState.LOST:
                self.assertNotAlmostEqual(position, device.deployed_position)
                self.assertNotAlmostEqual(position, device.retracted_position)
                self.assertEqual(velocity, 0)
            elif motion_state == DeployableMotionState.DEPLOYING:
                self.assertNotEqual(velocity, 0)
                if device.deployed_position > device.retracted_position:
                    self.assertGreater(velocity, 0)
                else:
                    self.assertLess(velocity, 0)
            elif motion_state == DeployableMotionState.RETRACTING:
                self.assertNotEqual(velocity, 0)
                if device.deployed_position > device.retracted_position:
                    self.assertLess(velocity, 0)
                else:
                    self.assertGreater(velocity, 0)
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
        with self.assertRaises(RuntimeError):
            await self.run_command(
                command=deploy_command,
                min_timeout=move_min_timeout,
            )
        with self.assertRaises(RuntimeError):
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


if __name__ == "__main__":
    unittest.main()
