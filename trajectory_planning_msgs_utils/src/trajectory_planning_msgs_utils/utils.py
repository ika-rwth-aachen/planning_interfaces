# ============================================================================
# MIT License
# 
# Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ============================================================================

import math
from typing import Sequence

from trajectory_planning_msgs.msg import (
    DRIVABLE,
    DRIVABLERWS,
    REFERENCE,
    Trajectory,
)


def wrap_angle(angle: float) -> float:
    """
    Wrap an angle to the range [-π, π].
    """
    wrapped_angle = angle
    while wrapped_angle > math.pi:
        wrapped_angle -= 2 * math.pi
    while wrapped_angle < -math.pi:
        wrapped_angle += 2 * math.pi
    return wrapped_angle


kExceptionUnknownType = "Unknown type ID: "


def get_state_dim(type_id: int) -> int:
    if type_id == DRIVABLE.TYPE_ID:
        return DRIVABLE.STATE_DIM
    if type_id == DRIVABLERWS.TYPE_ID:
        return DRIVABLERWS.STATE_DIM
    if type_id == REFERENCE.TYPE_ID:
        return REFERENCE.STATE_DIM
    raise ValueError(kExceptionUnknownType + str(type_id))


def get_state_dim_from_trajectory(trajectory: Trajectory) -> int:
    return get_state_dim(trajectory.type_id)


def get_states_size(states: Sequence[float]) -> int:
    return len(states)


def get_states_size_from_trajectory(trajectory: Trajectory) -> int:
    return get_states_size(trajectory.states)


def is_floating_division(numerator: int, denominator: int) -> bool:
    return numerator % denominator != 0
