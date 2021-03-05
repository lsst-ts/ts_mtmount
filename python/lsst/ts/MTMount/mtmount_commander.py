# This file is part of ts_MTMount.
#
# Developed for Vera C. Rubin Observatory Telescope and Site Systems.
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
from lsst.ts import simactuators

STD_TIMEOUT = 2

TRACK_INTERVAL = 0.1  # interval between tracking updates (seconds)

# How far in advance to set the time field of tracking commands (seconds)
TRACK_ADVANCE_TIME = 0.05


@dataclasses.dataclass
class RampArgs:
    el_start: float
    az_start: float
    el_end: float
    az_end: float
    el_speed: float
    az_speed: float


class MTMountCommander(salobj.CscCommander):
    def __init__(self, enable):
        super().__init__(
            name="MTMount",
            enable=enable,
            telemetry_fields_to_not_compare=("current", "timestamp"),
        )
        for command_to_ignore in ("abort", "setValue"):
            self.command_dict.pop(command_to_ignore, None)

        self.tracking_task = salobj.make_done_future()

        ramp_arg_names = list(RampArgs.__dataclass_fields__)
        self.help_dict["ramp"] = " ".join(ramp_arg_names) + " # track a ramp"
        self.ramp_count = 0

    @property
    def ramp_arg_names(self):
        return

    async def close(self):
        self.tracking_task.cancel()
        await super().close()

    async def do_ramp(self, args):
        if not self.tracking_task.done():
            print("Cancelling existing tracking sequence")
            self.tracking_task.cancel()

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
        self.tracking_task = asyncio.create_task(
            self._ramp(ramp_count=self.ramp_count, ramp_args=args)
        )

    async def do_stop(self, args):
        self.tracking_task.cancel()
        await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)

    async def do_stopTracking(self, args):
        self.tracking_task.cancel()
        await self.remote.cmd_stopTracking.start(timeout=STD_TIMEOUT)

    async def _ramp(self, ramp_count, ramp_args):
        try:
            ramp_generator = simactuators.RampGenerator(
                start_positions=[ramp_args.el_start, ramp_args.az_start],
                end_positions=[ramp_args.el_end, ramp_args.az_end],
                speeds=[ramp_args.el_speed, ramp_args.az_speed],
                advance_time=TRACK_ADVANCE_TIME,
            )
            print(
                f"Tracking a ramp from el={ramp_args.el_start}, az={ramp_args.az_start} "
                f" to el={ramp_args.el_end}, az={ramp_args.az_end} "
                f" at speed el={ramp_args.el_speed}, az={ramp_args.az_speed}; "
                f"this will take {ramp_generator.duration:0.2f} seconds"
            )

            print("Enable tracking")
            await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)

            print("Start the ramp")
            for positions, velocities, tai in ramp_generator():
                await self.remote.cmd_trackTarget.set_start(
                    elevation=positions[0],
                    elevationVelocity=velocities[0],
                    azimuth=positions[1],
                    azimuthVelocity=velocities[1],
                    taiTime=tai,
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
        print("Ramp done")
