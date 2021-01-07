# This file is part of ts_MTMount.
#
# Developed for Vera Rubin Observatory.
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
    def __init__(self, enable):
        super().__init__(
            name="MTMount",
            enable=enable,
            telemetry_fields_to_not_compare=("current", "timestamp"),
        )
        for command_to_ignore in ("abort", "setValue"):
            self.command_dict.pop(command_to_ignore, None)

        ramp_arg_names = list(RampArgs.__dataclass_fields__)
        self.help_dict["ramp"] = " ".join(ramp_arg_names) + " # track a ramp"
        self.ramp_count = 0

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
