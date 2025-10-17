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
import warnings
from typing import MutableSequence, Sequence

from trajectory_planning_msgs.msg import Trajectory

from .checks import (
    sanity_check_angle,
    sanity_check_state_size,
    sanity_check_states_size,
    sanity_check_trajectory,
)
from .state_getters import get_sample_point_size
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
)
from .utils import get_state_dim, wrap_angle


def _assign_slice(sequence: MutableSequence[float], start: int, values: Sequence[float]) -> None:
    sequence[start : start + len(values)] = list(values)


def set_state(state: MutableSequence[float], type_id: int, values: Sequence[float]) -> None:
    sanity_check_state_size(values, type_id)
    state[:] = list(values)


def set_state_in_states(
    states: MutableSequence[float], type_id: int, values: Sequence[float], sample_point: int
) -> None:
    sanity_check_state_size(values, type_id)
    sanity_check_states_size(states, type_id)
    state_dim = get_state_dim(type_id)
    sample_point_size = get_sample_point_size(states, type_id)
    if sample_point < 0 or sample_point >= sample_point_size:
        raise IndexError(f"Index i = {sample_point} out of range ({sample_point_size})")
    start = sample_point * state_dim
    _assign_slice(states, start, values)


def set_state_in_trajectory(trajectory: Trajectory, values: Sequence[float], sample_point: int) -> None:
    sanity_check_trajectory(trajectory)
    set_state_in_states(trajectory.states, trajectory.type_id, values, sample_point)


def set_states(states: MutableSequence[float], type_id: int, values: Sequence[float]) -> None:
    sanity_check_states_size(values, type_id)
    states[:] = list(values)


def set_states_in_trajectory(trajectory: Trajectory, type_id: int, values: Sequence[float]) -> None:
    set_states(trajectory.states, type_id, values)


def _set_entry_with_index(state: MutableSequence[float], index: int, value: float) -> None:
    state[index] = value


def _set_angle_entry(state: MutableSequence[float], index: int, value: float) -> None:
    try:
        sanity_check_angle(value)
        state[index] = value
    except ValueError as exc:
        warnings.warn(
            f"{exc}. Wrapping value to [-π, π].",
            RuntimeWarning,
            stacklevel=2,
        )
        state[index] = wrap_angle(value)


def _set_entry_in_states(
    states: MutableSequence[float], type_id: int, index: int, sample_point: int, value: float, angle_entry: bool = False
) -> None:
    sanity_check_states_size(states, type_id)
    state_dim = get_state_dim(type_id)
    sample_point_size = get_sample_point_size(states, type_id)
    if sample_point < 0 or sample_point >= sample_point_size:
        raise IndexError(f"Index i = {sample_point} out of range ({sample_point_size})")
    start = sample_point * state_dim
    absolute_index = start + index
    if angle_entry:
        _set_angle_entry(states, absolute_index, value)
    else:
        _set_entry_with_index(states, absolute_index, value)


def _set_entry_in_trajectory(
    trajectory: Trajectory, index: int, sample_point: int, value: float, angle_entry: bool = False
) -> None:
    sanity_check_trajectory(trajectory)
    _set_entry_in_states(
        trajectory.states,
        trajectory.type_id,
        index,
        sample_point,
        value,
        angle_entry=angle_entry,
    )


def set_t(state: MutableSequence[float], type_id: int, value: float) -> None:
    sanity_check_state_size(state, type_id)
    _set_entry_with_index(state, index_t(type_id), value)


def set_t_in_states(states: MutableSequence[float], type_id: int, value: float, sample_point: int) -> None:
    _set_entry_in_states(states, type_id, index_t(type_id), sample_point, value)


def set_t_in_trajectory(trajectory: Trajectory, value: float, sample_point: int) -> None:
    _set_entry_in_trajectory(trajectory, index_t(trajectory.type_id), sample_point, value)


def set_x(state: MutableSequence[float], type_id: int, value: float) -> None:
    sanity_check_state_size(state, type_id)
    _set_entry_with_index(state, index_x(type_id), value)


def set_x_in_states(states: MutableSequence[float], type_id: int, value: float, sample_point: int) -> None:
    _set_entry_in_states(states, type_id, index_x(type_id), sample_point, value)


def set_x_in_trajectory(trajectory: Trajectory, value: float, sample_point: int) -> None:
    _set_entry_in_trajectory(trajectory, index_x(trajectory.type_id), sample_point, value)


def set_y(state: MutableSequence[float], type_id: int, value: float) -> None:
    sanity_check_state_size(state, type_id)
    _set_entry_with_index(state, index_y(type_id), value)


def set_y_in_states(states: MutableSequence[float], type_id: int, value: float, sample_point: int) -> None:
    _set_entry_in_states(states, type_id, index_y(type_id), sample_point, value)


def set_y_in_trajectory(trajectory: Trajectory, value: float, sample_point: int) -> None:
    _set_entry_in_trajectory(trajectory, index_y(trajectory.type_id), sample_point, value)


def set_v(state: MutableSequence[float], type_id: int, value: float) -> None:
    sanity_check_state_size(state, type_id)
    _set_entry_with_index(state, index_v(type_id), value)


def set_v_in_states(states: MutableSequence[float], type_id: int, value: float, sample_point: int) -> None:
    _set_entry_in_states(states, type_id, index_v(type_id), sample_point, value)


def set_v_in_trajectory(trajectory: Trajectory, value: float, sample_point: int) -> None:
    _set_entry_in_trajectory(trajectory, index_v(trajectory.type_id), sample_point, value)


def set_theta(state: MutableSequence[float], type_id: int, value: float) -> None:
    sanity_check_state_size(state, type_id)
    _set_angle_entry(state, index_theta(type_id), value)


def set_theta_in_states(states: MutableSequence[float], type_id: int, value: float, sample_point: int) -> None:
    _set_entry_in_states(states, type_id, index_theta(type_id), sample_point, value, angle_entry=True)


def set_theta_in_trajectory(trajectory: Trajectory, value: float, sample_point: int) -> None:
    _set_entry_in_trajectory(
        trajectory,
        index_theta(trajectory.type_id),
        sample_point,
        value,
        angle_entry=True,
    )


def set_a(state: MutableSequence[float], type_id: int, value: float) -> None:
    sanity_check_state_size(state, type_id)
    _set_entry_with_index(state, index_a(type_id), value)


def set_a_in_states(states: MutableSequence[float], type_id: int, value: float, sample_point: int) -> None:
    _set_entry_in_states(states, type_id, index_a(type_id), sample_point, value)


def set_a_in_trajectory(trajectory: Trajectory, value: float, sample_point: int) -> None:
    _set_entry_in_trajectory(trajectory, index_a(trajectory.type_id), sample_point, value)


def compute_beta(
    delta_front: float,
    delta_rear: float,
    distance_front_axle: float,
    distance_rear_axle: float,
) -> float:
    wheelbase = distance_front_axle + distance_rear_axle
    return math.atan(
        (distance_rear_axle / wheelbase) * math.tan(delta_front)
        + (distance_front_axle / wheelbase) * math.tan(delta_rear)
    )


def set_beta_from_axles(
    state: MutableSequence[float],
    type_id: int,
    delta_front: float,
    delta_rear: float,
    distance_front_axle: float,
    distance_rear_axle: float,
) -> None:
    sanity_check_state_size(state, type_id)
    beta = compute_beta(delta_front, delta_rear, distance_front_axle, distance_rear_axle)
    _set_angle_entry(state, index_beta(type_id), beta)


def set_beta(state: MutableSequence[float], type_id: int, value: float) -> None:
    sanity_check_state_size(state, type_id)
    _set_angle_entry(state, index_beta(type_id), value)


def set_beta_from_axles_in_states(
    states: MutableSequence[float],
    type_id: int,
    delta_front: float,
    delta_rear: float,
    distance_front_axle: float,
    distance_rear_axle: float,
    sample_point: int,
) -> None:
    beta = compute_beta(delta_front, delta_rear, distance_front_axle, distance_rear_axle)
    _set_entry_in_states(
        states,
        type_id,
        index_beta(type_id),
        sample_point,
        beta,
        angle_entry=True,
    )


def set_beta_in_states(
    states: MutableSequence[float], type_id: int, value: float, sample_point: int
) -> None:
    _set_entry_in_states(
        states,
        type_id,
        index_beta(type_id),
        sample_point,
        value,
        angle_entry=True,
    )


def set_beta_from_axles_in_trajectory(
    trajectory: Trajectory,
    delta_front: float,
    delta_rear: float,
    distance_front_axle: float,
    distance_rear_axle: float,
    sample_point: int,
) -> None:
    sanity_check_trajectory(trajectory)
    set_beta_from_axles_in_states(
        trajectory.states,
        trajectory.type_id,
        delta_front,
        delta_rear,
        distance_front_axle,
        distance_rear_axle,
        sample_point,
    )


def set_beta_in_trajectory(trajectory: Trajectory, value: float, sample_point: int) -> None:
    _set_entry_in_trajectory(
        trajectory,
        index_beta(trajectory.type_id),
        sample_point,
        value,
        angle_entry=True,
    )


def set_delta_front(state: MutableSequence[float], type_id: int, value: float) -> None:
    sanity_check_state_size(state, type_id)
    _set_angle_entry(state, index_delta_front(type_id), value)


def set_delta_front_in_states(
    states: MutableSequence[float], type_id: int, value: float, sample_point: int
) -> None:
    _set_entry_in_states(
        states,
        type_id,
        index_delta_front(type_id),
        sample_point,
        value,
        angle_entry=True,
    )


def set_delta_front_in_trajectory(trajectory: Trajectory, value: float, sample_point: int) -> None:
    _set_entry_in_trajectory(
        trajectory,
        index_delta_front(trajectory.type_id),
        sample_point,
        value,
        angle_entry=True,
    )


def set_delta_rear(state: MutableSequence[float], type_id: int, value: float) -> None:
    sanity_check_state_size(state, type_id)
    _set_angle_entry(state, index_delta_rear(type_id), value)


def set_delta_rear_in_states(
    states: MutableSequence[float], type_id: int, value: float, sample_point: int
) -> None:
    _set_entry_in_states(
        states,
        type_id,
        index_delta_rear(type_id),
        sample_point,
        value,
        angle_entry=True,
    )


def set_delta_rear_in_trajectory(trajectory: Trajectory, value: float, sample_point: int) -> None:
    _set_entry_in_trajectory(
        trajectory,
        index_delta_rear(trajectory.type_id),
        sample_point,
        value,
        angle_entry=True,
    )


def set_delta_ack(state: MutableSequence[float], type_id: int, value: float) -> None:
    sanity_check_state_size(state, type_id)
    _set_angle_entry(state, index_delta_ack(type_id), value)


def set_delta_ack_in_states(
    states: MutableSequence[float], type_id: int, value: float, sample_point: int
) -> None:
    _set_entry_in_states(
        states,
        type_id,
        index_delta_ack(type_id),
        sample_point,
        value,
        angle_entry=True,
    )


def set_delta_ack_in_trajectory(trajectory: Trajectory, value: float, sample_point: int) -> None:
    _set_entry_in_trajectory(
        trajectory,
        index_delta_ack(trajectory.type_id),
        sample_point,
        value,
        angle_entry=True,
    )


def set_s(state: MutableSequence[float], type_id: int, value: float) -> None:
    sanity_check_state_size(state, type_id)
    _set_entry_with_index(state, index_s(type_id), value)


def set_s_in_states(states: MutableSequence[float], type_id: int, value: float, sample_point: int) -> None:
    _set_entry_in_states(states, type_id, index_s(type_id), sample_point, value)


def set_s_in_trajectory(trajectory: Trajectory, value: float, sample_point: int) -> None:
    _set_entry_in_trajectory(trajectory, index_s(trajectory.type_id), sample_point, value)


def set_standstill(trajectory: Trajectory, value: bool) -> None:
    sanity_check_trajectory(trajectory)
    trajectory.standstill = bool(value)
