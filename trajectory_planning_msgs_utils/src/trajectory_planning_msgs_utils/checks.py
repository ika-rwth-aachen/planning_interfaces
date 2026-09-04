# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: MIT

import math
from typing import Sequence

from trajectory_planning_msgs.msg import Trajectory

from .utils import get_state_dim, get_states_size, is_floating_division


kExceptionInvalidStateSize = "Invalid state size for trajectory with type ID: "
kExceptionInvalidStatesSize = "Invalid states (sample points) size for trajectory with type ID: "


def sanity_check_state_size(state: Sequence[float], type_id: int) -> None:
    state_dim = get_state_dim(type_id)
    state_size = len(state)
    if state_size != state_dim:
        raise ValueError(
            f"{kExceptionInvalidStateSize}{type_id}, {state_size} != {state_dim}"
        )


def sanity_check_states_size(states: Sequence[float], type_id: int) -> None:
    state_dim = get_state_dim(type_id)
    states_size = get_states_size(states)
    if is_floating_division(states_size, state_dim):
        raise ValueError(
            f"{kExceptionInvalidStatesSize}{type_id}, states_size = {states_size}, state_dim = {state_dim}"
        )


def sanity_check_states_size_for_trajectory(trajectory: Trajectory) -> None:
    sanity_check_states_size(trajectory.states, trajectory.type_id)


def sanity_check_angle(angle: float, val_min: float = -math.pi, val_max: float = math.pi) -> None:
    if angle < val_min or angle > val_max:
        raise ValueError(
            f"Angle value: {angle} out of range [{val_min}, {val_max}]"
        )


def sanity_check_trajectory(trajectory: Trajectory) -> None:
    sanity_check_states_size_for_trajectory(trajectory)
    # Additional checks can be added here in the future.
