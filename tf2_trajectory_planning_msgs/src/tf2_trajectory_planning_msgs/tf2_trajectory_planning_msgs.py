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
from copy import deepcopy
from typing import Iterable, Sequence

from geometry_msgs.msg import Pose, TransformStamped
from trajectory_planning_msgs.msg import DRIVABLE, DRIVABLERWS, REFERENCE, Trajectory
from trajectory_planning_msgs_utils.state_getters import (
    get_sample_point_size_from_trajectory,
    get_state_from_trajectory,
    get_t,
    get_theta,
    get_x,
    get_y,
)
from trajectory_planning_msgs_utils.state_setters import (
    set_state_in_trajectory,
    set_t,
    set_theta,
    set_x,
    set_y,
)

import tf2_geometry_msgs
import tf2_ros


def _register_passthrough_conversions(message_types: Iterable[type]) -> None:
    def _to_msg(msg):
        return msg

    def _from_msg(msg):
        return msg

    for msg_type in message_types:
        tf2_ros.ConvertRegistration().add_to_msg(msg_type, _to_msg)
        tf2_ros.ConvertRegistration().add_from_msg(msg_type, _from_msg)


_register_passthrough_conversions((Trajectory,))


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _stamps_to_time_delta(stamp_a, stamp_b) -> float:
    """
    Compute (stamp_a - stamp_b) in seconds.
    """
    return _stamp_to_seconds(stamp_a) - _stamp_to_seconds(stamp_b)


def _state_has_theta(type_id: int) -> bool:
    return type_id in (DRIVABLE.TYPE_ID, DRIVABLERWS.TYPE_ID)


def _quaternion_from_yaw(yaw: float):
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _yaw_from_quaternion(quaternion) -> float:
    x, y, z, w = quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _do_transform_state(
    state: Sequence[float],
    type_id: int,
    transform: TransformStamped,
    time_offset: float,
) -> Sequence[float]:
    x = get_x(state, type_id)
    y = get_y(state, type_id)
    t = get_t(state, type_id)
    theta = get_theta(state, type_id) if _state_has_theta(type_id) else 0.0

    pose_in = Pose()
    pose_in.position.x = x
    pose_in.position.y = y
    pose_in.position.z = 0.0
    qx, qy, qz, qw = _quaternion_from_yaw(theta)
    pose_in.orientation.x = qx
    pose_in.orientation.y = qy
    pose_in.orientation.z = qz
    pose_in.orientation.w = qw

    pose_out = tf2_geometry_msgs.do_transform_pose(pose_in, transform)

    state_out = list(state)
    set_x(state_out, type_id, pose_out.position.x)
    set_y(state_out, type_id, pose_out.position.y)
    set_t(state_out, type_id, t + time_offset)

    if _state_has_theta(type_id):
        yaw = _yaw_from_quaternion(
            (
                pose_out.orientation.x,
                pose_out.orientation.y,
                pose_out.orientation.z,
                pose_out.orientation.w,
            )
        )
        set_theta(state_out, type_id, yaw)

    return state_out


def do_transform_trajectory(msg: Trajectory, transform: TransformStamped) -> Trajectory:
    """
    Apply a `Transform` or `TransformStamped` to a `Trajectory`.
    """
    msg_out = deepcopy(msg)
    msg_out.header.stamp = transform.header.stamp
    msg_out.header.frame_id = transform.header.frame_id

    time_offset = _stamps_to_time_delta(transform.header.stamp, msg.header.stamp)

    sample_points = get_sample_point_size_from_trajectory(msg)
    for i in range(sample_points):
        state_in = get_state_from_trajectory(msg, i)
        state_out = _do_transform_state(state_in, msg.type_id, transform, time_offset)
        set_state_in_trajectory(msg_out, state_out, i)

    return msg_out


tf2_ros.TransformRegistration().add(Trajectory, do_transform_trajectory)
