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
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, TransformStamped
from trajectory_planning_msgs.msg import DRIVABLE, Trajectory

from tf2_trajectory_planning_msgs import do_transform_trajectory
from trajectory_planning_msgs_utils.init import initialize_trajectory
from trajectory_planning_msgs_utils.state_getters import (
    get_theta_from_trajectory,
    get_t_from_trajectory,
    get_x_from_trajectory,
    get_y_from_trajectory,
)
from trajectory_planning_msgs_utils.state_setters import (
    set_theta_in_trajectory,
    set_t_in_trajectory,
    set_x_in_trajectory,
    set_y_in_trajectory,
)

EPS = 1e-9


def _stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quaternion_from_yaw(yaw: float):
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _yaw_from_quaternion(quaternion) -> float:
    x, y, z, w = quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def test_do_transform_trajectory_drivable():
    trajectory = Trajectory()
    trajectory.header.frame_id = "source"
    trajectory.header.stamp.sec = 1
    trajectory.header.stamp.nanosec = 200_000_000
    trajectory.type_id = DRIVABLE.TYPE_ID

    initialize_trajectory(trajectory, DRIVABLE.TYPE_ID, 1)

    set_t_in_trajectory(trajectory, 0.5, 0)
    set_x_in_trajectory(trajectory, 1.0, 0)
    set_y_in_trajectory(trajectory, -2.0, 0)
    set_theta_in_trajectory(trajectory, 0.25, 0)

    transform = TransformStamped()
    transform.header.frame_id = "target"
    transform.child_frame_id = "source"
    transform.header.stamp.sec = 3
    transform.header.stamp.nanosec = 700_000_000
    transform.transform.translation.x = 4.0
    transform.transform.translation.y = -1.0
    transform.transform.translation.z = 0.0
    qx, qy, qz, qw = _quaternion_from_yaw(math.pi / 3.0)
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw

    transformed = do_transform_trajectory(trajectory, transform)

    assert transformed.header.frame_id == transform.header.frame_id
    assert transformed.header.stamp == transform.header.stamp

    pose_in = Pose()
    pose_in.position.x = 1.0
    pose_in.position.y = -2.0
    pose_in.position.z = 0.0
    qx_in, qy_in, qz_in, qw_in = _quaternion_from_yaw(0.25)
    pose_in.orientation.x = qx_in
    pose_in.orientation.y = qy_in
    pose_in.orientation.z = qz_in
    pose_in.orientation.w = qw_in
    pose_expected = tf2_geometry_msgs.do_transform_pose(pose_in, transform)

    assert math.isclose(
        get_x_from_trajectory(transformed, 0),
        pose_expected.position.x,
        rel_tol=EPS,
        abs_tol=EPS,
    )
    assert math.isclose(
        get_y_from_trajectory(transformed, 0),
        pose_expected.position.y,
        rel_tol=EPS,
        abs_tol=EPS,
    )

    expected_yaw = _yaw_from_quaternion(
        (
            pose_expected.orientation.x,
            pose_expected.orientation.y,
            pose_expected.orientation.z,
            pose_expected.orientation.w,
        )
    )
    assert math.isclose(
        get_theta_from_trajectory(transformed, 0),
        expected_yaw,
        rel_tol=EPS,
        abs_tol=EPS,
    )

    time_offset = _stamp_to_sec(transform.header.stamp) - _stamp_to_sec(trajectory.header.stamp)
    assert math.isclose(
        get_t_from_trajectory(transformed, 0),
        0.5 + time_offset,
        rel_tol=EPS,
        abs_tol=EPS,
    )


if __name__ == "__main__":
    pytest.main()
