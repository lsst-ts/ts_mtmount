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

__all__ = ["MTMountCommander"]

import asyncio
import dataclasses
import functools

from lsst.ts import salobj

STD_TIMEOUT = 2
TRACK_INTERVAL = 0.1


@dataclasses.dataclass
class RampArgs:
    el_pos0: float
    az_pos0: float
    el_vel: float
    az_vel: float
    duration: float


class MTMountCommander(salobj.CscCommander):
    def __init__(self):
        super().__init__(name="NewMTMount")
        # Remote for telemetry
        self.mtmount_remote = salobj.Remote(
            domain=self.remote.salinfo.domain, name="MTMount"
        )
        self.previous_tel_Azimuth = None
        self.previous_tel_Elevation = None
        self.mtmount_remote.tel_Azimuth.callback = functools.partial(
            self.tel_axis_callback, name="Azimuth"
        )
        self.mtmount_remote.tel_Elevation.callback = functools.partial(
            self.tel_axis_callback, name="Elevation"
        )

        ramp_arg_names = list(RampArgs.__dataclass_fields__)
        self.help_dict["ramp"] = " ".join(ramp_arg_names) + " # track a ramp"
        self.ramp_count = 0

    def tel_axis_callback(self, data, name):
        prev_name = f"previous_tel_{name}"
        actual_pos = round(getattr(data, f"{name}_Angle_Actual"), 2)
        actual_vel = round(getattr(data, f"{name}_Velocity_Actual"), 3)
        set_pos = getattr(data, f"{name}_Angle_Set")
        set_vel = getattr(data, f"{name}_Velocity_Set")
        data_to_save = (actual_pos, actual_vel, set_pos, set_vel)
        if data_to_save == getattr(self, prev_name, None):
            return
        setattr(self, prev_name, data_to_save)
        print(
            f"{name}: set_pos={set_pos}; "
            f"actual_pos={actual_pos}; "
            f"set_vel={set_vel}; "
            f"actual_vel={actual_vel}"
        )

    @property
    def ramp_arg_names(self):
        return

    async def do_ramp(self, args):
        ramp_arg_info = RampArgs.__dataclass_fields__
        if len(args) != len(ramp_arg_info):
            ramp_arg_names = list(ramp_arg_info)
            raise RuntimeError(
                f"requires {len(ramp_arg_info)} args: "
                f"{' '.join(ramp_arg_names)}; "
                f"got {len(args)} args: {' '.join(args)}"
            )

        arg_dict = {
            name: info.type(args[i])
            for i, (name, info) in enumerate(ramp_arg_info.items())
        }
        args = RampArgs(**arg_dict)
        print("args=", args)
        self.ramp_count += 1

        print("Enable tracking")
        await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)
        try:
            print("Start the ramp")
            tai0 = salobj.current_tai()
            while True:
                curr_tai = salobj.current_tai()
                dt = curr_tai - tai0
                if dt > args.duration:
                    break
                el = args.el_pos0 + args.el_vel * dt
                az = args.az_pos0 + args.az_vel * dt
                await self.remote.cmd_trackTarget.set_start(
                    azimuth=az,
                    azimuthVelocity=args.az_vel,
                    elevation=el,
                    elevationVelocity=args.el_vel,
                    taiTime=curr_tai,
                    trackId=self.ramp_count,
                    tracksys="local",
                    radesys="ICRS",
                    timeout=STD_TIMEOUT,
                )
                await asyncio.sleep(TRACK_INTERVAL)
        except asyncio.CancelledError:
            pass
        print("Disable tracking")
        await self.remote.cmd_stopTracking.start(timeout=STD_TIMEOUT)
