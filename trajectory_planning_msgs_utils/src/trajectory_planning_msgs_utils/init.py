# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: MIT

from trajectory_planning_msgs.msg import Trajectory

from .state_setters import set_states
from .utils import get_state_dim


STATE_INIT = 0.0


def initialize_states(states, type_id: int, sample_points: int) -> None:
    vector_size = sample_points * get_state_dim(type_id)
    set_states(states, type_id, [STATE_INIT] * vector_size)


def initialize_trajectory(trajectory: Trajectory, type_id: int, sample_points: int) -> None:
    initialize_states(trajectory.states, type_id, sample_points)
    trajectory.type_id = type_id
    trajectory.standstill = True
