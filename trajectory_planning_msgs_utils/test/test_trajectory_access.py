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

import pytest

from trajectory_planning_msgs.msg import DRIVABLE, DRIVABLERWS, REFERENCE, Trajectory

from trajectory_planning_msgs_utils.init import initialize_states, initialize_trajectory
from trajectory_planning_msgs_utils.state_getters import (
    get_beta_from_trajectory,
    get_delta_ack_from_trajectory,
    get_delta_front_from_trajectory,
    get_sample_point_size_from_trajectory,
    get_standstill,
    get_t_from_trajectory,
    get_theta_from_trajectory,
    get_x_from_trajectory,
    get_y_from_trajectory,
)
from trajectory_planning_msgs_utils.state_setters import (
    compute_beta,
    set_beta_from_axles_in_trajectory,
    set_delta_ack_in_trajectory,
    set_delta_front_in_trajectory,
    set_s_in_trajectory,
    set_standstill,
    set_t_in_trajectory,
    set_theta_in_trajectory,
    set_x_in_trajectory,
    set_y_in_trajectory,
)
from trajectory_planning_msgs_utils.utils import get_state_dim, wrap_angle

EPS = 1e-12


def test_wrap_angle():
    assert math.isclose(wrap_angle(4 * math.pi), 0.0, rel_tol=EPS, abs_tol=EPS)
    assert math.isclose(wrap_angle(-3 * math.pi / 2), math.pi / 2, rel_tol=EPS, abs_tol=EPS)


def test_initialize_states():
    states = []
    initialize_states(states, DRIVABLE.TYPE_ID, 2)
    assert len(states) == 2 * get_state_dim(DRIVABLE.TYPE_ID)
    assert all(value == 0.0 for value in states)


def test_initialize_trajectory_and_basic_setters():
    trajectory = Trajectory()
    initialize_trajectory(trajectory, DRIVABLE.TYPE_ID, 2)

    assert trajectory.type_id == DRIVABLE.TYPE_ID
    assert get_sample_point_size_from_trajectory(trajectory) == 2
    assert get_standstill(trajectory)

    set_t_in_trajectory(trajectory, 1.0, 0)
    set_x_in_trajectory(trajectory, 2.0, 0)
    set_y_in_trajectory(trajectory, -1.0, 0)
    set_s_in_trajectory(trajectory, 5.0, 0)

    assert math.isclose(get_t_from_trajectory(trajectory, 0), 1.0, rel_tol=EPS)
    assert math.isclose(get_x_from_trajectory(trajectory, 0), 2.0, rel_tol=EPS)
    assert math.isclose(get_y_from_trajectory(trajectory, 0), -1.0, rel_tol=EPS)

    # Angles are wrapped into [-pi, pi]
    set_theta_in_trajectory(trajectory, math.pi + 0.3, 0)
    theta = get_theta_from_trajectory(trajectory, 0)
    assert -math.pi <= theta <= math.pi
    assert math.isclose(theta, wrap_angle(math.pi + 0.3), rel_tol=EPS, abs_tol=EPS)

    set_delta_ack_in_trajectory(trajectory, -4.2, 0)
    delta = get_delta_ack_from_trajectory(trajectory, 0)
    assert -math.pi <= delta <= math.pi

    set_standstill(trajectory, False)
    assert not get_standstill(trajectory)


def test_reference_theta_approximation():
    trajectory = Trajectory()
    initialize_trajectory(trajectory, REFERENCE.TYPE_ID, 3)

    set_x_in_trajectory(trajectory, 0.0, 0)
    set_y_in_trajectory(trajectory, 0.0, 0)

    set_x_in_trajectory(trajectory, 1.0, 1)
    set_y_in_trajectory(trajectory, 0.0, 1)

    set_x_in_trajectory(trajectory, 1.0, 2)
    set_y_in_trajectory(trajectory, 1.0, 2)

    theta_mid = get_theta_from_trajectory(trajectory, 1)
    assert math.isclose(theta_mid, math.pi / 4, rel_tol=1e-9)

    theta_start = get_theta_from_trajectory(trajectory, 0)
    assert math.isclose(theta_start, 0.0, rel_tol=1e-9)

    theta_end = get_theta_from_trajectory(trajectory, 2)
    assert math.isclose(theta_end, math.pi / 2, rel_tol=1e-9)


def test_beta_computation_for_drivablerws():
    trajectory = Trajectory()
    initialize_trajectory(trajectory, DRIVABLERWS.TYPE_ID, 1)

    delta_front = 0.1
    delta_rear = -0.05
    distance_front_axle = 1.2
    distance_rear_axle = 1.3

    set_beta_from_axles_in_trajectory(
        trajectory,
        delta_front,
        delta_rear,
        distance_front_axle,
        distance_rear_axle,
        0,
    )
    beta = get_beta_from_trajectory(trajectory, 0)
    expected = compute_beta(delta_front, delta_rear, distance_front_axle, distance_rear_axle)
    assert math.isclose(beta, expected, rel_tol=1e-9)

    set_delta_front_in_trajectory(trajectory, math.pi / 2, 0)
    delta_front_read = get_delta_front_from_trajectory(trajectory, 0)
    assert -math.pi <= delta_front_read <= math.pi
