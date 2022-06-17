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

__all__ = ["MTMountCommander"]

import asyncio
import dataclasses

from lsst.ts import utils
from lsst.ts import salobj
from lsst.ts import simactuators

STD_TIMEOUT = 2

TRACK_INTERVAL = 0.1  # interval between tracking updates (seconds)

# How far in advance to set the time field of tracking commands (seconds)
TRACK_ADVANCE_TIME = 0.05


@dataclasses.dataclass
class RampArgs:
    az_start: float
    el_start: float
    az_end: float
    el_end: float
    az_speed: float
    el_speed: float


class MTMountCommander(salobj.CscCommander):
    def __init__(self, enable):
        super().__init__(
            name="MTMount",
            enable=enable,
            telemetry_fields_to_not_compare=("current", "timestamp"),
        )
        for command_to_ignore in ("abort", "setValue"):
            self.command_dict.pop(command_to_ignore, None)

        self.tracking_task = utils.make_done_future()

        ramp_arg_names = list(RampArgs.__dataclass_fields__)
        self.help_dict["ramp"] = " ".join(ramp_arg_names) + " # track a ramp"
        self.ramp_count = 0

        # Disable telemetry for now
        # TODO: output it if important values change by "enough"
        for name in self.remote.salinfo.telemetry_names:
            topic_attr_name = f"tel_{name}"
            topic = getattr(self.remote, topic_attr_name)
            setattr(topic, "callback", None)

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
                start_positions=[ramp_args.az_start, ramp_args.el_start],
                end_positions=[ramp_args.az_end, ramp_args.el_end],
                speeds=[ramp_args.az_speed, ramp_args.el_speed],
                advance_time=TRACK_ADVANCE_TIME,
            )
            print(
                f"Tracking a ramp from az={ramp_args.az_start}, el={ramp_args.el_start} "
                f" to az={ramp_args.az_end}, el={ramp_args.el_end} "
                f" at speed az={ramp_args.az_speed}, el={ramp_args.el_speed}; "
                f"this will take {ramp_generator.duration:0.2f} seconds"
            )

            print("Enable tracking")
            await self.remote.cmd_startTracking.start(timeout=STD_TIMEOUT)

            print("Start the ramp")
            isfirst = True
            for positions, velocities, tai in ramp_generator():
                t0 = utils.current_tai()
                await self.remote.cmd_trackTarget.set_start(
                    azimuth=positions[0],
                    azimuthVelocity=velocities[0],
                    elevation=positions[1],
                    elevationVelocity=velocities[1],
                    taiTime=tai,
                    trackId=self.ramp_count,
                    tracksys="local",
                    radesys="ICRS",
                    timeout=STD_TIMEOUT,
                )
                if isfirst:
                    isfirst = True
                    dt = utils.current_tai() - t0
                    print(f"*** track command delay = {dt:0.3f}")
                await asyncio.sleep(TRACK_INTERVAL)
        except asyncio.CancelledError:
            pass
        print("Disable tracking")
        await self.remote.cmd_stopTracking.start(timeout=STD_TIMEOUT)
        print("Ramp done")
