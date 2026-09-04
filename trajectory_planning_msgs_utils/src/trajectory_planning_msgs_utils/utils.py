# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: MIT

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
