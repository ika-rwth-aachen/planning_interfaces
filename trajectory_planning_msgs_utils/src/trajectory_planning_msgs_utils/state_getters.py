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
from typing import List, Sequence

from trajectory_planning_msgs.msg import DRIVABLE, DRIVABLERWS, REFERENCE, Trajectory

from .checks import (
    sanity_check_angle,
    sanity_check_state_size,
    sanity_check_states_size,
    sanity_check_states_size_for_trajectory,
    sanity_check_trajectory,
)
from .state_index import (
    index_a,
    index_beta,
    index_delta_ack,
    index_delta_front,
    index_delta_rear,
    index_s,
    index_t,
    index_theta,
    index_v,
    index_x,
    index_y,
    kExceptionUnknownStateEntry,
)
from .utils import get_state_dim, get_states_size


def get_sample_point_size(states: Sequence[float], type_id: int) -> int:
    sanity_check_states_size(states, type_id)
    states_size = get_states_size(states)
    state_dim = get_state_dim(type_id)
    return states_size // state_dim


def get_sample_point_size_from_trajectory(trajectory: Trajectory) -> int:
    sanity_check_states_size_for_trajectory(trajectory)
    return get_sample_point_size(trajectory.states, trajectory.type_id)


def get_state(states: Sequence[float], type_id: int, index: int) -> List[float]:
    sanity_check_states_size(states, type_id)
    sample_point_size = get_sample_point_size(states, type_id)
    if index < 0 or index >= sample_point_size:
        raise IndexError(f"Index i = {index} out of range ({sample_point_size})")
    state_dim = get_state_dim(type_id)
    start = index * state_dim
    end = start + state_dim
    if end > get_states_size(states):
        raise IndexError(f"Slice [{start}:{end}] exceeds available states")
    return list(states[start:end])


def get_state_from_trajectory(trajectory: Trajectory, index: int) -> List[float]:
    sanity_check_trajectory(trajectory)
    return get_state(trajectory.states, trajectory.type_id, index)


def get_t(state: Sequence[float], type_id: int) -> float:
    sanity_check_state_size(state, type_id)
    return state[index_t(type_id)]


def get_t_from_states(states: Sequence[float], type_id: int, index: int) -> float:
    return get_t(get_state(states, type_id, index), type_id)


def get_t_from_trajectory(trajectory: Trajectory, index: int) -> float:
    sanity_check_trajectory(trajectory)
    return get_t_from_states(trajectory.states, trajectory.type_id, index)


def get_x(state: Sequence[float], type_id: int) -> float:
    sanity_check_state_size(state, type_id)
    return state[index_x(type_id)]


def get_x_from_states(states: Sequence[float], type_id: int, index: int) -> float:
    return get_x(get_state(states, type_id, index), type_id)


def get_x_from_trajectory(trajectory: Trajectory, index: int) -> float:
    sanity_check_trajectory(trajectory)
    return get_x_from_states(trajectory.states, trajectory.type_id, index)


def get_y(state: Sequence[float], type_id: int) -> float:
    sanity_check_state_size(state, type_id)
    return state[index_y(type_id)]


def get_y_from_states(states: Sequence[float], type_id: int, index: int) -> float:
    return get_y(get_state(states, type_id, index), type_id)


def get_y_from_trajectory(trajectory: Trajectory, index: int) -> float:
    sanity_check_trajectory(trajectory)
    return get_y_from_states(trajectory.states, trajectory.type_id, index)


def get_v(state: Sequence[float], type_id: int) -> float:
    sanity_check_state_size(state, type_id)
    return state[index_v(type_id)]


def get_v_from_states(states: Sequence[float], type_id: int, index: int) -> float:
    return get_v(get_state(states, type_id, index), type_id)


def get_v_from_trajectory(trajectory: Trajectory, index: int) -> float:
    sanity_check_trajectory(trajectory)
    return get_v_from_states(trajectory.states, trajectory.type_id, index)


def approximate_theta(state1: Sequence[float], state2: Sequence[float], type_id: int) -> float:
    dx = get_x(state2, type_id) - get_x(state1, type_id)
    dy = get_y(state2, type_id) - get_y(state1, type_id)
    return math.atan2(dy, dx)


def get_theta(state: Sequence[float], type_id: int) -> float:
    sanity_check_state_size(state, type_id)
    theta = state[index_theta(type_id)]
    sanity_check_angle(theta)
    return theta


def get_theta_from_states(states: Sequence[float], type_id: int, index: int) -> float:
    sanity_check_states_size(states, type_id)
    if type_id in {DRIVABLE.TYPE_ID, DRIVABLERWS.TYPE_ID}:
        return get_theta(get_state(states, type_id, index), type_id)

    if type_id == REFERENCE.TYPE_ID:
        n_samples = get_sample_point_size(states, type_id)
        if n_samples < 2:
            raise ValueError("Not enough sample points to approximate theta")

        if index == 0:
            state1 = get_state(states, type_id, index)
            state2 = get_state(states, type_id, index + 1)
        elif index == n_samples - 1:
            state1 = get_state(states, type_id, index - 1)
            state2 = get_state(states, type_id, index)
        else:
            state1 = get_state(states, type_id, index - 1)
            state2 = get_state(states, type_id, index + 1)
        theta = approximate_theta(state1, state2, type_id)
        sanity_check_angle(theta)
        return theta

    raise ValueError(kExceptionUnknownStateEntry + f"{type_id}, theta")


def get_theta_from_trajectory(trajectory: Trajectory, index: int) -> float:
    sanity_check_trajectory(trajectory)
    return get_theta_from_states(trajectory.states, trajectory.type_id, index)


def get_a(state: Sequence[float], type_id: int) -> float:
    sanity_check_state_size(state, type_id)
    return state[index_a(type_id)]


def get_a_from_states(states: Sequence[float], type_id: int, index: int) -> float:
    return get_a(get_state(states, type_id, index), type_id)


def get_a_from_trajectory(trajectory: Trajectory, index: int) -> float:
    sanity_check_trajectory(trajectory)
    return get_a_from_states(trajectory.states, trajectory.type_id, index)


def get_beta(state: Sequence[float], type_id: int) -> float:
    sanity_check_state_size(state, type_id)
    beta = state[index_beta(type_id)]
    sanity_check_angle(beta)
    return beta


def get_beta_from_states(states: Sequence[float], type_id: int, index: int) -> float:
    return get_beta(get_state(states, type_id, index), type_id)


def get_beta_from_trajectory(trajectory: Trajectory, index: int) -> float:
    sanity_check_trajectory(trajectory)
    return get_beta_from_states(trajectory.states, trajectory.type_id, index)


def get_delta_front(state: Sequence[float], type_id: int) -> float:
    sanity_check_state_size(state, type_id)
    delta_front = state[index_delta_front(type_id)]
    sanity_check_angle(delta_front)
    return delta_front


def get_delta_front_from_states(states: Sequence[float], type_id: int, index: int) -> float:
    return get_delta_front(get_state(states, type_id, index), type_id)


def get_delta_front_from_trajectory(trajectory: Trajectory, index: int) -> float:
    sanity_check_trajectory(trajectory)
    return get_delta_front_from_states(trajectory.states, trajectory.type_id, index)


def get_delta_rear(state: Sequence[float], type_id: int) -> float:
    sanity_check_state_size(state, type_id)
    delta_rear = state[index_delta_rear(type_id)]
    sanity_check_angle(delta_rear)
    return delta_rear


def get_delta_rear_from_states(states: Sequence[float], type_id: int, index: int) -> float:
    return get_delta_rear(get_state(states, type_id, index), type_id)


def get_delta_rear_from_trajectory(trajectory: Trajectory, index: int) -> float:
    sanity_check_trajectory(trajectory)
    return get_delta_rear_from_states(trajectory.states, trajectory.type_id, index)


def get_delta_ack(state: Sequence[float], type_id: int) -> float:
    sanity_check_state_size(state, type_id)
    delta_ack = state[index_delta_ack(type_id)]
    sanity_check_angle(delta_ack)
    return delta_ack


def get_delta_ack_from_states(states: Sequence[float], type_id: int, index: int) -> float:
    return get_delta_ack(get_state(states, type_id, index), type_id)


def get_delta_ack_from_trajectory(trajectory: Trajectory, index: int) -> float:
    sanity_check_trajectory(trajectory)
    return get_delta_ack_from_states(trajectory.states, trajectory.type_id, index)


def get_s(state: Sequence[float], type_id: int) -> float:
    sanity_check_state_size(state, type_id)
    return state[index_s(type_id)]


def get_s_from_states(states: Sequence[float], type_id: int, index: int) -> float:
    return get_s(get_state(states, type_id, index), type_id)


def get_s_from_trajectory(trajectory: Trajectory, index: int) -> float:
    sanity_check_trajectory(trajectory)
    return get_s_from_states(trajectory.states, trajectory.type_id, index)


def get_standstill(trajectory: Trajectory) -> bool:
    sanity_check_trajectory(trajectory)
    return bool(trajectory.standstill)
