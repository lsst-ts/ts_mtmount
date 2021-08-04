# This file is part of ts_MTMount.
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

__all__ = ["DeployableDevice"]

from lsst.ts import salobj
from lsst.ts.idl.enums.MTMount import DeployableMotionState

from .point_to_point_device import PointToPointDevice


class DeployableDevice(PointToPointDevice):
    """A device that can only be deployed or retracted.

    Includes DeployablePlatform, MirrorCovers and MirrorCoverLocks.

    Parameters
    ----------
    controller : `MockController`
        Mock controller.
    system_id : `lsst.ts.idl.enums.MTMount.System`
        System ID.
    start_deployed : `bool`
        Start deployed (True) or retracted (False)?
    deployed_position : `float`, optional
        Deployed position.
    retracted_position : `float`, optional
        Retracted position.
    move_time : `float`, optional
        Time to deploy or retract (second).

    Notes
    -----
    Supports all commands except MOVE and MOVE_VELOCITY.

    Supports deploy, retract, and move_all commands because
    some devices use deploy and retract (mirror covers),
    and others use move_all (mirror cover locks).

    Unlike the real system, the drive argument must always be -1
    (meaning all drives). That suffices for the CSC.
    """

    epsilon = 0.1
    """Accuracy for the device to be considered fully deployed or retracted.
    """

    def __init__(
        self,
        controller,
        system_id,
        start_deployed,
        deployed_position=100,
        retracted_position=0,
        move_time=1,
    ):
        if deployed_position == retracted_position:
            raise ValueError(
                f"deployed_position must not equal retracted_position = {deployed_position}"
            )
        self.retracted_position = retracted_position
        self.deployed_position = deployed_position
        self._deploying_velocity_sign = (
            1 if deployed_position > retracted_position else -1
        )
        start_position = deployed_position if start_deployed else retracted_position
        super().__init__(
            controller=controller,
            system_id=system_id,
            min_position=min(retracted_position, deployed_position),
            max_position=max(retracted_position, deployed_position),
            start_position=start_position,
            speed=abs(deployed_position - retracted_position) / move_time,
            multi_drive=True,
        )

    def motion_state(self, tai=None):
        """Get the motion state at a specified time.

        Parameters
        ----------
        tai : `float` or `None`, optional
            TAI time at which to make the determination.
            The current TAI if None.

        Returns
        -------
        state : `lsst.ts.idl.enums.MTMount.DeployableMotionState`
            Motion state.
        """
        if tai is None:
            tai = salobj.current_tai()
        velocity = self.actuator.velocity(tai)
        position = self.actuator.position(tai)
        if velocity == 0:
            if abs(position - self.deployed_position) < self.epsilon:
                return DeployableMotionState.DEPLOYED
            elif abs(position - self.retracted_position) < self.epsilon:
                return DeployableMotionState.RETRACTED
            else:
                return DeployableMotionState.LOST
        elif (self._deploying_velocity_sign > 0) == (velocity > 0):
            return DeployableMotionState.DEPLOYING
        else:
            return DeployableMotionState.RETRACTING

    def do_deploy(self, command):
        return self.move(position=self.deployed_position, command=command)

    def do_retract(self, command):
        return self.move(position=self.retracted_position, command=command)

    def do_move(self, command):
        raise NotImplementedError("Not implemented")

    def do_move_velocity(self, command):
        raise NotImplementedError("Not implemented")

    def do_move_all(self, command):
        if command.deploy:
            return self.do_deploy(command)
        else:
            return self.do_retract(command)

    def do_stop(self, command):
        """Stop the actuator."""
        self.supersede_move_command(command)
        self.actuator.stop()
