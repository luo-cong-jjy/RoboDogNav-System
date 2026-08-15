# Copyright 2026 Virdyn Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pure transition policy for the M20 factory ROS 2 motion state machine."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional


IDLE = 0
STAND = 1
SOFT_ESTOP = 2
BOOT_DAMPING = 3
LIE_DOWN = 4
RL_CONTROL = 17


@dataclass(frozen=True)
class DirectMotionStatus:
    """Motion fields reported by ``drdds/msg/MotionInfo``."""

    state: int
    gait: int
    linear_x: float
    linear_y: float
    angular_z: float

    @property
    def stationary(self) -> bool:
        """Return whether gait changes are safe under the vendor contract."""
        return (
            abs(self.linear_x) < 0.03
            and abs(self.linear_y) < 0.03
            and abs(self.angular_z) < 0.05
        )


def decode_motion_info(data: Any) -> DirectMotionStatus:
    """
    Decode the public deep-robotics-msg 1.1 MotionInfoValue ABI.

    The current vendor interface nests state and gait in their corresponding
    value messages. Keeping this conversion in the transport boundary avoids
    leaking DrDDS layout details into the common locomotion state machine.
    """
    return DirectMotionStatus(
        state=int(data.motion_state.state),
        gait=int(data.gait_state.gait),
        linear_x=float(data.vel_x),
        linear_y=float(data.vel_y),
        angular_z=float(data.vel_yaw),
    )


@dataclass(frozen=True)
class DirectTransition:
    """One deterministic state-machine decision."""

    ready: bool = False
    fault: str = ''
    state_command: Optional[int] = None
    gait_command: Optional[int] = None


def select_direct_transition(
    *,
    motion_requested: bool,
    ownership_confirmed: bool,
    e_stop: bool,
    status: Optional[DirectMotionStatus],
    requested_gait: int,
) -> DirectTransition:
    """Choose the next explicit ROS 2 state or gait command."""
    if e_stop:
        return DirectTransition(fault='E_STOP_ASSERTED')
    if not motion_requested:
        return DirectTransition()
    if not ownership_confirmed:
        return DirectTransition(fault='COMMAND_OWNERSHIP_NOT_CONFIRMED')
    if status is None:
        return DirectTransition()
    if status.state == SOFT_ESTOP:
        return DirectTransition(fault='M20_SOFT_ESTOP_LATCHED')
    if status.state in {IDLE, BOOT_DAMPING, LIE_DOWN}:
        return DirectTransition(state_command=STAND)
    if status.state == STAND:
        return DirectTransition(state_command=RL_CONTROL)
    if status.state != RL_CONTROL:
        return DirectTransition(
            fault='UNSUPPORTED_MOTION_STATE_{0}'.format(status.state)
        )
    if status.gait != requested_gait:
        if status.stationary:
            return DirectTransition(gait_command=requested_gait)
        return DirectTransition()
    return DirectTransition(ready=True)
