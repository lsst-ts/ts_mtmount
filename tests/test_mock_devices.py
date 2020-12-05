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
import logging
import unittest

import asynctest

from lsst.ts import salobj
from lsst.ts import MTMount

STD_TIMEOUT = 2  # Timeout for short operations (sec)
# Padding for the time limit returned by device do_methods
TIMEOUT_PADDING = 1


class TrivialMockController:
    def __init__(self):
        self.command_dict = {}
        self.log = logging.getLogger()


class MockDevicesTestCase(asynctest.TestCase):
    def setUp(self):
        self.controller = TrivialMockController()
        axis_devices = [
            MTMount.mock.AxisDevice(controller=self.controller, device_id=device_id)
            for device_id in MTMount.LimitsDict
        ]
        devices = axis_devices + [
            MTMount.mock.MainPowerSupplyDevice(controller=self.controller),
            MTMount.mock.MirrorCoverLocksDevice(controller=self.controller),
            MTMount.mock.MirrorCoversDevice(controller=self.controller),
            MTMount.mock.OilSupplySystemDevice(controller=self.controller),
            MTMount.mock.TopEndChillerDevice(controller=self.controller),
        ]
        self.device_dict = {device.device_id: device for device in devices}

    def get_command_class(self, command_code_name):
        command_code = getattr(MTMount.CommandCode, command_code_name.upper())
        return MTMount.commands.CommandDict[command_code]

    async def run_command(self, command, min_timeout=None, should_noack=False):
        """Run a command that should succeed and wait for replies.

        Parameters
        ----------
        command : `MTMount.commands.Command`
            Command to execute.
        min_timeout : `float` or `None`, optional
            Minimum timeout. Must be specified if the command method
            returns a timeout, None if not.
        should_noack : `bool`, optional
            True if the command should start OK,
            but then fail or be superseded.
        """
        print(f"run_command({command})")
        do_method = self.controller.command_dict[command.command_code]
        try:
            timeout_task = do_method(command)
        except Exception as e:
            print(f"command failed: {e}")
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
            await task
            if should_noack:
                self.fail(f"Command {command} succeeded but should_noack true")
        except Exception as e:
            if not should_noack:
                self.fail(f"Command {command} failed but should_noack false: {e}")

    async def test_base_commands(self):
        for device_id, device in self.device_dict.items():
            with self.subTest(device_id=device.device_id.name):
                if device_id in (
                    MTMount.DeviceId.MIRROR_COVERS,
                    MTMount.DeviceId.MIRROR_COVER_LOCKS,
                ):
                    drive = -1
                else:
                    drive = None
                await self.check_base_commands(device=device, drive=drive)

    async def test_mirror_cover_locks(self):
        device = self.device_dict[MTMount.DeviceId.MIRROR_COVER_LOCKS]

        await self.check_point_to_point_device(
            device=device,
            power_on_command=MTMount.commands.MirrorCoverLocksPower(drive=-1, on=True),
            goto_min_command=MTMount.commands.MirrorCoverLocksMoveAll(
                drive=-1, deploy=False
            ),
            goto_max_command=MTMount.commands.MirrorCoverLocksMoveAll(
                drive=-1, deploy=True
            ),
            stop_command=MTMount.commands.MirrorCoverLocksStop(drive=-1),
            start_at_min=True,
            move_min_timeout=0.5,
            power_on_min_timeout=None,
        )

    async def test_mirror_covers(self):
        device = self.device_dict[MTMount.DeviceId.MIRROR_COVERS]

        await self.check_point_to_point_device(
            device=device,
            power_on_command=MTMount.commands.MirrorCoversPower(drive=-1, on=True),
            goto_min_command=MTMount.commands.MirrorCoversDeploy(drive=-1),
            goto_max_command=MTMount.commands.MirrorCoversRetract(drive=-1),
            stop_command=MTMount.commands.MirrorCoversStop(drive=-1),
            start_at_min=True,
            move_min_timeout=0.5,
            power_on_min_timeout=None,
        )

    async def test_oil_supply_system(self):
        device = self.device_dict[MTMount.DeviceId.OIL_SUPPLY_SYSTEM]

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
        device = self.device_dict[MTMount.DeviceId.TOP_END_CHILLER]
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
        for device_id in (
            MTMount.DeviceId.AZIMUTH_AXIS,
            MTMount.DeviceId.ELEVATION_AXIS,
            MTMount.DeviceId.CAMERA_CABLE_WRAP,
        ):
            with self.subTest(device_id=device_id.name):
                device = self.device_dict[device_id]
                await self.check_axis_device(device)

    async def check_axis_device(self, device):
        has_home_command = device.device_id in (
            MTMount.DeviceId.AZIMUTH_AXIS,
            MTMount.DeviceId.ELEVATION_AXIS,
        )

        self.assertFalse(device.power_on)
        self.assertFalse(device.enabled)
        self.assertFalse(device.has_target)

        short_command_names = [
            "drive_enable",
            "drive_reset",
            "enable_tracking",
            "move",
            "power",
            "stop",
            "track",
        ]
        if has_home_command:
            short_command_names.append("home")
        dev_prefix = f"{device.device_id.name}_"
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
        enable_tracking_off_command = command_classes["enable_tracking"](on=False)
        enable_tracking_on_command = command_classes["enable_tracking"](on=True)
        home_command = command_classes["home"]() if has_home_command else None
        power_off_command = command_classes["power"](on=False)
        power_on_command = command_classes["power"](on=True)
        stop_command = command_classes["stop"]()

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
            enable_tracking_off_command,
            enable_tracking_on_command,
            stop_command,
            home_command,
            command_classes["move"](position=device.actuator.min_position + 1),
            command_classes["track"](
                position=device.actuator.min_position + 1,
                velocity=0,
                tai=salobj.current_tai(),
            ),
        ]
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
        start_segment = device.actuator.path.at(salobj.current_tai())
        self.assertEqual(start_segment.velocity, 0)
        start_position = start_segment.position
        end_position = start_position + 2
        move_command = command_classes["move"](position=end_position)
        task = asyncio.create_task(self.run_command(move_command, min_timeout=0.5))

        await asyncio.sleep(0.1)  # give command time to start
        self.assertAlmostEqual(device.actuator.target.position, end_position)
        self.assertEqual(device.actuator.target.velocity, 0)
        segment = device.actuator.path.at(salobj.current_tai())
        self.assertGreater(abs(segment.velocity), 0.01)
        self.assertTrue(device.has_target)

        await task
        end_segment = device.actuator.path.at(salobj.current_tai())
        self.assertAlmostEqual(end_segment.velocity, 0)
        self.assertAlmostEqual(end_segment.position, end_position)
        self.assertTrue(device.has_target)

        # Start homing or a big point to point move, then stop the axes
        # (Homing is a point to point move, and it's slow).
        if has_home_command:
            slow_move_command = home_command
        else:
            start_position = start_segment.position
            end_position = start_position + 10
            slow_move_command = command_classes["move"](position=end_position)
        task = asyncio.create_task(
            self.run_command(slow_move_command, min_timeout=0.5, should_noack=True)
        )

        await asyncio.sleep(0.1)

        await self.run_command(stop_command)
        await task
        stop_duration = device.actuator.path[-1].tai - salobj.current_tai()
        await asyncio.sleep(stop_duration)
        self.assertFalse(device.has_target)

        # The tracking command should fail when not in tracking mode.
        track_command = command_classes["track"](
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
        self.assertFalse(device.has_target)

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
            track_command = command_classes["track"](
                position=position, velocity=velocity, tai=tai,
            )
            await self.run_command(track_command)
            self.assertAlmostEqual(device.actuator.target.position, position)
            self.assertAlmostEqual(device.actuator.target.velocity, velocity)
            self.assertAlmostEqual(device.actuator.target.tai, tai, delta=1e-5)
            self.assertTrue(device.has_target)
            await asyncio.sleep(0.1)

        # Disable tracking and check state
        await self.run_command(enable_tracking_off_command)
        self.assertTrue(device.power_on)
        self.assertTrue(device.enabled)
        self.assertFalse(device.tracking_enabled)
        self.assertFalse(device.has_target)

        # Check that stop disable tracking
        await self.run_command(enable_tracking_on_command)
        self.assertTrue(device.power_on)
        self.assertTrue(device.enabled)
        self.assertTrue(device.tracking_enabled)
        self.assertFalse(device.has_target)

        await self.run_command(stop_command)
        self.assertTrue(device.power_on)
        self.assertTrue(device.enabled)
        self.assertFalse(device.tracking_enabled)
        self.assertFalse(device.has_target)

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

        device_prefix = device.device_id.name
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
            MTMount.DeviceId.MAIN_POWER_SUPPLY: 5,
            MTMount.DeviceId.OIL_SUPPLY_SYSTEM: 900,
        }.get(device.device_id, None)
        min_off_timeout = {
            MTMount.DeviceId.MAIN_POWER_SUPPLY: 0,
            MTMount.DeviceId.OIL_SUPPLY_SYSTEM: 0,
        }.get(device.device_id, None)

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

    async def check_point_to_point_device(
        self,
        device,
        power_on_command,
        goto_min_command,
        goto_max_command,
        stop_command,
        start_at_min,
        move_min_timeout,
        power_on_min_timeout,
    ):
        def assert_at_end(at_min):
            """Assert the device is its min or max position.
            """
            if at_min:
                self.assertAlmostEqual(
                    device.actuator.position(), device.actuator.min_position
                )
                self.assertAlmostEqual(
                    device.actuator.end_position, device.actuator.min_position
                )
            else:
                self.assertAlmostEqual(
                    device.actuator.position(), device.actuator.max_position
                )
                self.assertAlmostEqual(
                    device.actuator.end_position, device.actuator.max_position
                )
            self.assertFalse(device.actuator.moving())

        assert_at_end(at_min=start_at_min)

        # Test that moves fail if not powered on
        with self.assertRaises(RuntimeError):
            await self.run_command(
                command=goto_min_command,
                min_timeout=move_min_timeout,
                should_noack=True,
            )
        with self.assertRaises(RuntimeError):
            await self.run_command(
                command=goto_max_command, min_timeout=move_min_timeout
            )

        await self.run_command(
            command=power_on_command, min_timeout=power_on_min_timeout
        )

        # Move to min position
        await self.run_command(
            command=goto_min_command,
            min_timeout=0 if start_at_min else move_min_timeout,
        )
        assert_at_end(at_min=True)

        # Stop should have no effect while halted
        await self.run_command(command=stop_command, min_timeout=None)
        assert_at_end(at_min=True)

        # Start a move to max and check that it is as expected
        task = asyncio.create_task(
            self.run_command(command=goto_max_command, min_timeout=move_min_timeout)
        )
        await asyncio.sleep(0.1)  # Let the move begin
        self.assertGreaterEqual(
            device.actuator.position(), device.actuator.min_position
        )
        self.assertAlmostEqual(
            device.actuator.end_position, device.actuator.max_position
        )
        self.assertTrue(device.actuator.moving())

        await task
        assert_at_end(at_min=False)

        # Move back to min but stop partway
        task = asyncio.create_task(
            self.run_command(
                command=goto_min_command,
                min_timeout=move_min_timeout,
                should_noack=True,
            )
        )
        await asyncio.sleep(0.1)  # Let the move begin
        self.assertLess(device.actuator.position(), device.actuator.max_position)
        self.assertAlmostEqual(
            device.actuator.end_position, device.actuator.min_position
        )
        self.assertTrue(device.actuator.moving())
        await self.run_command(command=stop_command, min_timeout=None)
        self.assertLess(device.actuator.position(), device.actuator.max_position)
        self.assertFalse(device.actuator.moving())

        await task


if __name__ == "__main__":
    unittest.main()
