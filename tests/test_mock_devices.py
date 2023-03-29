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
from lsst.ts import mtmount, utils
from lsst.ts.idl.enums.MTMount import AxisMotionState, DeployableMotionState, System

STD_TIMEOUT = 2  # Timeout for short operations (sec)
# Padding for the time limit returned by device do_methods
TIMEOUT_PADDING = 1


class TrivialMockController:
    def __init__(self):
        self.command_dict = {}
        self.log = logging.getLogger()
        axis_devices = [
            mtmount.mock.AxisDevice(
                controller=self,
                system_id=system_id,
                start_position=mtmount.mock.INITIAL_POSITION[system_id],
            )
            for system_id in mtmount.mock.CmdLimitsDict
        ]
        thermal_devices = [
            mtmount.mock.ThermalDevice(controller=self, system_id=system_id)
            for system_id in mtmount.mock.ThermalDevice.supported_system_ids
        ]
        devices = (
            axis_devices
            + thermal_devices
            + [
                mtmount.mock.MainAxesPowerSupplyDevice(controller=self),
                mtmount.mock.MainCabinetThermalDevice(controller=self),
                mtmount.mock.MirrorCoverLocksDevice(controller=self),
                mtmount.mock.MirrorCoversDevice(controller=self),
                mtmount.mock.AuxiliaryCabinetsThermalDevice(controller=self),
                mtmount.mock.OilSupplySystemDevice(controller=self),
                mtmount.mock.TopEndChillerDevice(controller=self),
            ]
        )
        self.device_dict = {device.system_id: device for device in devices}
        self.ambient_temperature = -1.23

    @property
    def main_axes_power_supply(self):
        """Return the main axes power supply mock device."""
        return self.device_dict[System.MAIN_AXES_POWER_SUPPLY]


class MockDevicesTestCase(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        # Use asyncSetUp instead of setUp, so there is a running event loop.
        self.controller = TrivialMockController()

        self.system_ids_no_power_command = frozenset(
            (System.AUXILIARY_CABINETS_THERMAL, System.MAIN_CABINET_THERMAL)
        )

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
        for system_id, device in self.controller.device_dict.items():
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
        device = self.controller.device_dict[System.MIRROR_COVER_LOCKS]
        self.check_device_repr(device)

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
        device = self.controller.device_dict[System.MIRROR_COVERS]
        self.check_device_repr(device)

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

    async def test_oil_supply_system_auto(self):
        """Test the OSS main power command, which requires auto mode."""
        system_id = System.OIL_SUPPLY_SYSTEM
        device = self.controller.device_dict[system_id]
        device_prefix = device.system_id.name
        self.check_device_repr(device)

        assert not device.power_on
        assert not device.cooling_on
        assert not device.circulation_pump_on
        assert not device.main_pump_on
        assert not device.auto_mode

        # You must be in auto mode to use the OilSupplySystemPower
        with pytest.raises(RuntimeError):
            await self.run_command(
                command=mtmount.commands.OilSupplySystemPower(on=False), min_timeout=0
            )
        with pytest.raises(RuntimeError):
            await self.run_command(
                command=mtmount.commands.OilSupplySystemPower(on=True),
                min_timeout=900,
            )
        assert not device.power_on
        assert not device.cooling_on
        assert not device.circulation_pump_on
        assert not device.main_pump_on
        assert not device.auto_mode

        await self.run_command(
            command=mtmount.commands.OilSupplySystemSetMode(auto=True)
        )
        assert not device.power_on
        assert not device.cooling_on
        assert not device.circulation_pump_on
        assert not device.main_pump_on
        assert device.auto_mode

        await self.run_command(
            command=mtmount.commands.OilSupplySystemPower(on=True), min_timeout=900
        )
        assert device.power_on
        assert device.cooling_on
        assert device.circulation_pump_on
        assert device.main_pump_on
        assert device.auto_mode

        # Test cabinet thermal control while in auto mode and power is on
        make_power_command = self.get_command_class(device_prefix + "_POWER")
        set_setpoint_command_class = self.get_command_class(
            device_prefix + "_CABINETS_THERMAL_SETPOINT"
        )

        def make_track_setpoint_command(setpoint):
            return set_setpoint_command_class(setpoint=setpoint)

        await self.check_thermal_device(
            system_id=system_id,
            make_power_command=make_power_command,
            make_track_setpoint_command=make_track_setpoint_command,
            make_track_ambient_command=None,
        )

        await self.run_command(
            command=mtmount.commands.OilSupplySystemPower(on=False), min_timeout=0
        )
        assert not device.power_on
        assert not device.cooling_on
        assert not device.circulation_pump_on
        assert not device.main_pump_on
        assert device.auto_mode

        await self.run_command(
            command=mtmount.commands.OilSupplySystemSetMode(auto=False)
        )
        assert not device.power_on
        assert not device.cooling_on
        assert not device.circulation_pump_on
        assert not device.main_pump_on
        assert not device.auto_mode

    async def test_oil_supply_system_manual(self):
        """Test the OSS subsystem power commands, which require manual mode."""
        device = self.controller.device_dict[System.OIL_SUPPLY_SYSTEM]
        self.check_device_repr(device)

        # Specify the commands are in the order in which the subsystems
        # must be turned on.
        subsystem_command_dict = {
            "cooling": mtmount.commands.OilSupplySystemPowerCooling,
            "circulation_pump": mtmount.commands.OilSupplySystemPowerCirculationPump,
            "main_pump": mtmount.commands.OilSupplySystemPowerMainPump,
        }

        # All of these commands are rejected if in auto mode
        await self.run_command(
            command=mtmount.commands.OilSupplySystemSetMode(auto=True)
        )
        assert device.auto_mode
        for command in subsystem_command_dict.values():
            with pytest.raises(RuntimeError):
                await self.run_command(command(on=False))
            with pytest.raises(RuntimeError):
                await self.run_command(command(on=True))

        assert not device.power_on
        assert not device.cooling_on
        assert not device.circulation_pump_on
        assert not device.main_pump_on
        assert device.auto_mode

        await self.run_command(
            command=mtmount.commands.OilSupplySystemSetMode(auto=False)
        )
        assert not device.power_on
        assert not device.cooling_on
        assert not device.circulation_pump_on
        assert not device.main_pump_on
        assert not device.auto_mode

        # Cannot turn on circulation or main pump if cooling off
        for command in (
            mtmount.commands.OilSupplySystemPowerCirculationPump,
            mtmount.commands.OilSupplySystemPowerMainPump,
        ):
            with pytest.raises(RuntimeError):
                await self.run_command(command=command(on=True))

        # Turn on cooling
        await self.run_command(
            command=mtmount.commands.OilSupplySystemPowerCooling(on=True),
            min_timeout=900,
        )
        assert device.cooling_on
        assert not device.circulation_pump_on
        assert not device.main_pump_on
        assert not device.auto_mode

        # Cannot turn on main pump if circulation pump off
        with pytest.raises(RuntimeError):
            await self.run_command(
                command=mtmount.commands.OilSupplySystemPowerMainPump(on=True)
            )

        # Turn on circulation pump
        await self.run_command(
            command=mtmount.commands.OilSupplySystemPowerCirculationPump(on=True)
        )
        assert device.cooling_on
        assert device.circulation_pump_on
        assert not device.main_pump_on
        assert not device.auto_mode

        # Cannot turn off cooling if circulation pump is on
        with pytest.raises(RuntimeError):
            await self.run_command(
                command=mtmount.commands.OilSupplySystemPowerCooling(on=False)
            )

        # Turn on main pump
        await self.run_command(
            command=mtmount.commands.OilSupplySystemPowerMainPump(on=True)
        )
        assert device.cooling_on
        assert device.circulation_pump_on
        assert device.main_pump_on
        assert device.power_on
        assert not device.auto_mode

        # Cannot turn off chiller or circulation pump with main pump on
        for command in (
            mtmount.commands.OilSupplySystemPowerCooling,
            mtmount.commands.OilSupplySystemPowerCirculationPump,
        ):
            with pytest.raises(RuntimeError):
                await self.run_command(command=command(on=False))

        # Now turn the subsystems off one at a time, in t he proper order.
        # We have already tested forbidden commands in all the resulting
        # states, so don't test that again.

        # Turn off main pump
        await self.run_command(
            command=mtmount.commands.OilSupplySystemPowerMainPump(on=False)
        )
        assert device.cooling_on
        assert device.circulation_pump_on
        assert not device.main_pump_on
        assert not device.auto_mode

        # Turn off circulation pump
        await self.run_command(
            command=mtmount.commands.OilSupplySystemPowerCirculationPump(on=False)
        )
        assert device.cooling_on
        assert not device.circulation_pump_on
        assert not device.main_pump_on
        assert not device.auto_mode

        # Turn off chiller
        await self.run_command(
            command=mtmount.commands.OilSupplySystemPowerCooling(on=False),
            min_timeout=0,
        )
        assert not device.cooling_on
        assert not device.circulation_pump_on
        assert not device.main_pump_on
        assert not device.auto_mode

    async def test_thermal_devices(self):
        """Test ThermalDevice, a specific kind of thermal controller."""
        assert mtmount.mock.ThermalDevice.supported_system_ids == {
            System.AZIMUTH_DRIVES_THERMAL,
            System.CABINET_0101_THERMAL,
            System.ELEVATION_DRIVES_THERMAL,
        }
        for system_id in mtmount.mock.ThermalDevice.supported_system_ids:
            device = self.controller.device_dict[system_id]
            device_prefix = device.system_id.name

            make_power_command = self.get_command_class(device_prefix + "_POWER")

            set_mode_command_class = self.get_command_class(
                device_prefix + "_CONTROL_MODE"
            )

            def make_track_setpoint_command(setpoint):
                return set_mode_command_class(
                    mode=mtmount.ThermalMode.TRACK_SETPOINT, setpoint=setpoint
                )

            def make_track_ambient_command(setpoint):
                return set_mode_command_class(
                    mode=mtmount.ThermalMode.TRACK_AMBIENT, setpoint=setpoint
                )

            await self.check_thermal_device(
                system_id=system_id,
                make_power_command=make_power_command,
                make_track_ambient_command=make_track_ambient_command,
                make_track_setpoint_command=make_track_setpoint_command,
            )

    async def check_thermal_device(
        self,
        system_id,
        make_power_command,
        make_track_ambient_command,
        make_track_setpoint_command,
    ):
        """Check any kind of mock thermal device.

        Parameters
        ----------
        system_id : `System`
            The system ID of the thermal controller; a System enum.
        make_power_command : function | None
            A function that takes on argument "on", a boolean,
            and returns a command to turn the device on or off.
            Specify None if the system doesn't support being turned on and off.
        make_track_ambient_command : function | None
            A function that takes one argument "setpoint", a float,
            and returns a command to make the system track ambient,
            ignoring the setpoint argument.
            Specify None if the system cannot track ambient.
        make_track_setpoint_command : function
            A function that takes one argument "setpoint", a float,
            and returns a command to make the system track that setpoint.
            This cannot be None.
        """
        device = self.controller.device_dict[system_id]
        power_min_timeout = None
        if system_id in {System.OIL_SUPPLY_SYSTEM}:
            power_min_timeout = 0

        if make_power_command is not None:
            await self.run_command(
                make_power_command(on=True), min_timeout=power_min_timeout
            )
            assert device.power_on
            assert not device.alarm_on
            await self.run_command(
                make_power_command(on=False), min_timeout=power_min_timeout
            )
            assert not device.power_on
            assert not device.alarm_on
            await self.run_command(
                make_power_command(on=True), min_timeout=power_min_timeout
            )
            assert device.power_on
            assert not device.alarm_on

        # In setpoint mode temperature should track setpoint.
        for setpoint in (-55.5, 0, 23.4):
            await self.run_command(make_track_setpoint_command(setpoint=setpoint))
            assert device.power_on
            assert not device.alarm_on
            assert device.track_setpoint
            assert not device.track_ambient
            assert device.setpoint == setpoint
            assert device.temperature == pytest.approx(
                setpoint, abs=device.temperature_slop
            )

        # In ambient mode temperature should match current controller ambient.
        if make_track_ambient_command is not None:
            for setpoint in (-55.5, 0, 23.4):
                await self.run_command(make_track_ambient_command(setpoint=setpoint))
                assert device.power_on
                assert not device.alarm_on
                assert not device.track_setpoint
                assert device.track_ambient
                assert device.temperature == pytest.approx(
                    self.controller.ambient_temperature, abs=device.temperature_slop
                )

        # Commands should fail if off.
        if make_power_command is not None:
            await self.run_command(
                make_power_command(on=False), min_timeout=power_min_timeout
            )
            assert not device.power_on
            assert not device.alarm_on
            with pytest.raises(RuntimeError):
                await self.run_command(make_track_setpoint_command(setpoint=0))
            if make_track_ambient_command is not None:
                with pytest.raises(RuntimeError):
                    await self.run_command(make_track_ambient_command(setpoint=0))

            await self.run_command(
                make_power_command(on=True), min_timeout=power_min_timeout
            )
            assert device.power_on
            assert not device.alarm_on

        # Commands should fail in fault.
        device.alarm_on = True
        assert device.alarm_on
        assert not device.power_on
        with pytest.raises(RuntimeError):
            await self.run_command(make_track_setpoint_command(setpoint=0))
        if make_track_ambient_command is not None:
            with pytest.raises(RuntimeError):
                await self.run_command(make_track_ambient_command(setpoint=0))

        # Reset alarm state turn power back on.
        device.alarm_on = False
        if make_power_command is not None:
            await self.run_command(
                make_power_command(on=True), min_timeout=power_min_timeout
            )
        assert device.power_on
        assert not device.alarm_on

    async def test_auxiliary_cabinets_thermal(self):
        system_id = System.AUXILIARY_CABINETS_THERMAL
        device = self.controller.device_dict[system_id]
        assert device.power_on
        assert not device.alarm_on

        for on in (True, False):
            await self.run_command(
                mtmount.commands.AuxiliaryCabinetsThermalFanPower(on=on)
            )
            assert device.fans_on == on

        def make_track_setpoint_command(setpoint):
            return mtmount.commands.AuxiliaryCabinetsThermalSetpoint(setpoint=setpoint)

        await self.check_thermal_device(
            system_id=System.AUXILIARY_CABINETS_THERMAL,
            make_power_command=None,
            make_track_setpoint_command=make_track_setpoint_command,
            make_track_ambient_command=None,
        )

    async def test_main_cabinet_thermal(self):
        system_id = System.MAIN_CABINET_THERMAL

        track_ambient_command_class = mtmount.commands.MainCabinetThermalTrackAmbient

        def make_track_setpoint_command(setpoint):
            return track_ambient_command_class(track_ambient=False, setpoint=setpoint)

        def make_track_ambient_command(setpoint):
            return track_ambient_command_class(track_ambient=True, setpoint=setpoint)

        await self.check_thermal_device(
            system_id=system_id,
            make_power_command=None,
            make_track_ambient_command=make_track_ambient_command,
            make_track_setpoint_command=make_track_setpoint_command,
        )

    async def test_top_end_chiller(self):
        system_id = System.TOP_END_CHILLER
        device = self.controller.device_dict[system_id]
        self.check_device_repr(device)

        track_ambient_command_class = mtmount.commands.TopEndChillerTrackAmbient

        def make_track_setpoint_command(setpoint):
            return track_ambient_command_class(track_ambient=False, setpoint=setpoint)

        def make_track_ambient_command(setpoint):
            return track_ambient_command_class(track_ambient=True, setpoint=setpoint)

        await self.check_thermal_device(
            system_id=system_id,
            make_power_command=mtmount.commands.TopEndChillerPower,
            make_track_ambient_command=make_track_ambient_command,
            make_track_setpoint_command=make_track_setpoint_command,
        )

    async def test_axis_devices(self):
        for system_id in (
            System.AZIMUTH,
            System.ELEVATION,
            System.CAMERA_CABLE_WRAP,
        ):
            with self.subTest(system_id=system_id.name):
                device = self.controller.device_dict[system_id]
                await self.check_axis_device(device)

    async def test_command_failure(self):
        device = self.controller.device_dict[System.MIRROR_COVERS]
        await self.run_command(mtmount.commands.MirrorCoversPower(drive=-1, on=True))
        assert device.power_on
        device.fail_next_command = True
        await self.run_command(
            mtmount.commands.MirrorCoversDeploy(drive=-1),
            min_timeout=0,
            should_fail=True,
        )

    async def check_axis_device(self, device):
        # Set main axes power supply off
        # This test uses TrivialMockController so do not try to send
        # the usual command.
        self.controller.main_axes_power_supply.power_on = False
        assert not self.controller.main_axes_power_supply.power_on

        self.check_device_repr(device)

        assert device.is_azel == (
            device.system_id in {System.ELEVATION, System.AZIMUTH}
        )
        assert not device.power_on
        assert not device.enabled
        assert not device.has_target
        assert not device.moving_point_to_point

        assert device.cmd_limits == mtmount.mock.CmdLimitsDict[device.system_id]

        short_command_names = [
            "drive_enable",
            "drive_reset",
            "enable_tracking",
            "move",
            "power",
            "stop",
            "track_target",
            "reset_alarm",
        ]
        if device.is_azel:
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
        slow_command_codes = {command_codes[name] for name in ("move", "track_target")}

        # A batch of useful commands.
        drive_enable_off_command = command_classes["drive_enable"](drive=-1, on=False)
        drive_enable_on_command = command_classes["drive_enable"](drive=-1, on=True)
        drive_reset_command = command_classes["drive_reset"](drive=-1)
        enable_tracking_on_command = command_classes["enable_tracking"](on=True)
        if device.is_azel:
            # Elevation and azimuth have a home command,
            # and tracking cannot be paused (so the enable tracking command
            # has no "on" parameter)
            home_command = command_classes["home"]()
            pause_tracking_command = None
        else:
            # The camera cable wrap has no home command,
            # and tracking can be paused.
            home_command = None
            pause_tracking_command = command_classes["enable_tracking"](on=False)
        power_off_command = command_classes["power"](on=False)
        power_on_command = command_classes["power"](on=True)
        stop_command = command_classes["stop"]()
        reset_alarm_command = command_classes["reset_alarm"]()
        move_command_class = command_classes["move"]
        track_command_class = command_classes["track_target"]

        # Many commands should fail for elevation and azimuth
        # if the main power supply is off
        if device.is_azel:
            for command in (
                enable_tracking_on_command,
                stop_command,
                home_command,
                move_command_class(position=device.cmd_limits.min_position + 0.1),
                pause_tracking_command,
                power_on_command,
                track_command_class(
                    position=device.cmd_limits.min_position + 0.1,
                    velocity=0,
                    tai=utils.current_tai(),
                ),
            ):
                if command is None:
                    continue
                with self.subTest(command=str(command)):
                    min_timeout = (
                        0 if command.command_code in slow_command_codes else None
                    )
                    with pytest.raises(RuntimeError):
                        await self.run_command(command, min_timeout=min_timeout)
                    assert not device.power_on
                    assert not device.enabled
                    assert not device.tracking_enabled
                    assert not device.has_target
                    assert not device.homed

        # drive_reset fails if off
        assert not device.power_on
        assert not device.enabled
        assert not device.tracking_enabled
        with pytest.raises(RuntimeError):
            await self.run_command(drive_reset_command)
        assert not device.power_on
        assert not device.enabled
        assert not device.tracking_enabled
        assert not device.homed

        # Set main axes power supply on if azimuth or elevation.
        # This test uses TrivialMockController so do not try to send
        # the usual command.
        if device.is_azel:
            self.controller.main_axes_power_supply.power_on = True
            assert self.controller.main_axes_power_supply.power_on

        # Power on the device; this should not enable the drive.
        await self.run_command(power_on_command)
        assert device.power_on
        assert device.enabled
        assert not device.tracking_enabled
        assert not device.has_target
        assert not device.homed

        # Disable the drive
        await self.run_command(drive_enable_off_command)
        assert device.power_on
        assert not device.enabled
        assert not device.tracking_enabled
        assert not device.has_target
        assert not device.homed

        # Most commands should fail if drive not enabled.
        for command in (
            enable_tracking_on_command,
            stop_command,
            home_command,
            move_command_class(position=device.cmd_limits.min_position + 0.1),
            pause_tracking_command,
            track_command_class(
                position=device.cmd_limits.min_position + 0.1,
                velocity=0,
                tai=utils.current_tai(),
            ),
        ):
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
                assert not device.homed

        # Enable the drives
        await self.run_command(drive_enable_on_command)
        assert device.power_on
        assert device.enabled
        assert not device.tracking_enabled
        assert not device.has_target

        # Enabling tracking should fail for elevation and azimuth until
        # these axes are homed.
        if device.is_azel:
            with pytest.raises(RuntimeError):
                await self.run_command(enable_tracking_on_command, min_timeout=0)
            assert device.power_on
            assert device.enabled
            assert not device.tracking_enabled
            assert not device.has_target
            assert not device.homed

            await self.run_command(home_command, min_timeout=0)
            assert device.power_on
            assert device.enabled
            assert not device.tracking_enabled
            assert device.has_target
            assert device.homed

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

        # Check that homing is rejected while moving
        if device.is_azel:
            with pytest.raises(RuntimeError):
                await self.run_command(home_command, min_timeout=0)

        await task
        assert device.motion_state() == AxisMotionState.STOPPED
        end_segment = device.actuator.path.at(utils.current_tai())
        assert end_segment.velocity == pytest.approx(0)
        assert end_segment.position == pytest.approx(end_position)
        assert device.has_target
        assert device.point_to_point_target == pytest.approx(end_position)
        assert not device.tracking_enabled

        # Test that out-of-range point to point moves are rejected,
        # leaving the device enabled.
        for bad_position in (
            device.cmd_limits.min_position - 0.001,
            device.cmd_limits.max_position + 0.001,
        ):
            move_command = move_command_class(position=bad_position)
            with pytest.raises(ValueError):
                await self.run_command(move_command, min_timeout=0.5)
        assert device.power_on
        assert device.enabled
        assert device.has_target
        assert not device.tracking_enabled

        # Start a big point to point move, then stop the axes
        end_position = device.cmd_limits.max_position
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
            position=device.cmd_limits.min_position + 0.1,
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

        # Check that out-of-range tracking commands are rejected,
        # leaving the device enabled and in tracking mode.
        for bad_position in (
            device.cmd_limits.min_position - 0.001,
            device.cmd_limits.max_position + 0.001,
        ):
            tai = previous_tai + 0.001
            previous_tai = tai
            with pytest.raises(ValueError):
                track_command = track_command_class(
                    position=bad_position,
                    velocity=velocity,
                    tai=tai,
                )
                await self.run_command(track_command)
        for bad_velocity in (
            -device.cmd_limits.max_velocity - 0.001,
            device.cmd_limits.max_velocity + 0.001,
        ):
            tai = previous_tai + 0.001
            previous_tai = tai
            track_command = track_command_class(
                position=position,
                velocity=bad_velocity,
                tai=tai,
            )
            with pytest.raises(ValueError):
                await self.run_command(track_command)
        assert not device.alarm_on
        assert device.power_on
        assert device.enabled
        assert device.motion_state(tai) == AxisMotionState.TRACKING

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
        if not device.is_azel:
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
        device_has_power_command = (
            device.system_id not in self.system_ids_no_power_command
        )

        self.controller.main_axes_power_supply.power_on = False

        assert not device.alarm_on
        if device_has_power_command:
            assert not device.power_on
        else:
            assert device.power_on

        device_prefix = device.system_id.name
        if device_has_power_command:
            power_command_class = self.get_command_class(device_prefix + "_POWER")
            if drive is not None:
                power_kwargs = dict(drive=drive)
            else:
                power_kwargs = {}
            power_command_on = power_command_class(on=True, **power_kwargs)
            power_command_off = power_command_class(on=False, **power_kwargs)
        else:
            power_command_on = None
            power_command_off = None

        reset_alarm_command_class = self.get_command_class(
            device_prefix + "_RESET_ALARM"
        )
        reset_alarm_command = reset_alarm_command_class()

        is_azel = device.system_id in {System.ELEVATION, System.AZIMUTH}

        min_on_timeout = {
            System.MAIN_AXES_POWER_SUPPLY: 5,
            System.OIL_SUPPLY_SYSTEM: 900,
        }.get(device.system_id, None)
        min_off_timeout = {
            System.MAIN_AXES_POWER_SUPPLY: 0,
            System.OIL_SUPPLY_SYSTEM: 0,
        }.get(device.system_id, None)

        assert not device.alarm_on
        device.alarm_on = True
        await self.run_command(reset_alarm_command)
        if device_has_power_command:
            assert not device.power_on
        else:
            assert device.power_on

        if is_azel:
            # The reset alarm command for azimuth and elevation
            # is a no-op if main axes power off
            assert device.alarm_on
            self.controller.main_axes_power_supply.power_on = True
            await self.run_command(reset_alarm_command)
            assert not device.alarm_on
            self.controller.main_axes_power_supply.power_on = False
        else:
            assert not device.alarm_on

        if is_azel:
            # Cannot turn on device unless main power is on
            with pytest.raises(RuntimeError):
                await self.run_command(power_command_on, min_timeout=min_on_timeout)
            self.controller.main_axes_power_supply.power_on = True

        if device.system_id == System.OIL_SUPPLY_SYSTEM:
            # The oil supply system must be in auto mode to turn it on
            # using the standard power command.
            await self.run_command(
                command=mtmount.commands.OilSupplySystemSetMode(auto=True)
            )
            assert device.auto_mode
        if device_has_power_command:
            await self.run_command(power_command_on, min_timeout=min_on_timeout)
            assert device.power_on
            assert not device.alarm_on

        # Check that we can reset alarms when the device is on.
        await self.run_command(reset_alarm_command)
        assert device.power_on
        assert not device.alarm_on

        if device_has_power_command:
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

    def check_device_repr(self, device):
        """Check the string representation of a mock device."""
        repr_str = repr(device)
        assert type(device).__name__ in repr_str
        assert device.system_id.name in repr_str
