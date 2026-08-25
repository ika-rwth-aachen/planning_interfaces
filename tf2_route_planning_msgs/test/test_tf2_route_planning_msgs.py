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

import pytest
from geometry_msgs.msg import TransformStamped, Point, Pose
from route_planning_msgs.msg import Route, LaneBoundary, LaneElement, RegulatoryElement, RouteElement
from tf2_route_planning_msgs import do_transform_route

from route_planning_msgs_utils.route_setters import set_left_boundary_of_lane_element, set_right_boundary_of_lane_element

from copy import deepcopy

def test_do_transform_route():
    point_in = Point(x=1.0, y=2.0, z=3.0)
    point_out = Point(x=9.0, y=18.0, z=33.0)

    pose_in = Pose()
    pose_in.position = deepcopy(point_in)
    pose_in.orientation.x = 0.0
    pose_in.orientation.y = 0.0
    pose_in.orientation.z = 1.0
    pose_in.orientation.w = 0.0

    pose_out = Pose()
    pose_out.position = deepcopy(point_out)
    pose_out.orientation.x = 0.0
    pose_out.orientation.y = 0.0
    pose_out.orientation.z = 0.0
    pose_out.orientation.w = 1.0

    # Define a LaneBoundary
    lane_boundary = LaneBoundary()
    lane_boundary.point = deepcopy(point_in)

    # Define a LaneElement
    lane_element = LaneElement()
    lane_element.reference_pose = deepcopy(pose_in)
    set_left_boundary_of_lane_element(lane_element, deepcopy(lane_boundary))
    set_right_boundary_of_lane_element(lane_element, deepcopy(lane_boundary))

    # Define a RegulatoryElement
    regulatory_element = RegulatoryElement()
    regulatory_element.reference_line = [deepcopy(point_in), deepcopy(point_in)]
    regulatory_element.positions.append(deepcopy(point_in))

    # Define a RouteElement
    route_element = RouteElement()
    route_element.lane_elements.append(lane_element)
    route_element.left_boundary = deepcopy(point_in)
    route_element.right_boundary = deepcopy(point_in)
    route_element.regulatory_elements.append(regulatory_element)

    # Define a Route
    route = Route()
    route.destination = deepcopy(point_in)
    route.intermediate_destinations.append(deepcopy(point_in))
    route.intermediate_destinations.append(deepcopy(point_in))
    route.route_elements.append(deepcopy(route_element))

    # Create a TransformStamped object
    tf = TransformStamped()
    tf.transform.translation.x = 10.0
    tf.transform.translation.y = 20.0
    tf.transform.translation.z = 30.0
    tf.transform.rotation.x = 0.0
    tf.transform.rotation.y = 0.0
    tf.transform.rotation.z = 1.0
    tf.transform.rotation.w = 0.0

    # Perform the transformation
    route_tf = do_transform_route(route, tf)

    # Assert the transformed route
    assert route_tf.destination == point_out
    assert len(route_tf.intermediate_destinations) == 2
    assert route_tf.intermediate_destinations[0] == point_out
    assert route_tf.intermediate_destinations[1] == point_out
    assert len(route_tf.route_elements) == 1
    assert len(route_tf.route_elements[0].lane_elements) == 1
    assert route_tf.route_elements[0].left_boundary == point_out
    assert route_tf.route_elements[0].right_boundary == point_out
    assert route_tf.route_elements[0].lane_elements[0].reference_pose == pose_out
    assert route_tf.route_elements[0].lane_elements[0].left_boundary.point == point_out
    assert route_tf.route_elements[0].lane_elements[0].right_boundary.point == point_out
    assert len(route_tf.route_elements[0].regulatory_elements) == 1
    assert len(route_tf.route_elements[0].regulatory_elements[0].reference_line) == 2
    assert route_tf.route_elements[0].regulatory_elements[0].reference_line[0] == point_out
    assert route_tf.route_elements[0].regulatory_elements[0].reference_line[1] == point_out
    assert route_tf.route_elements[0].regulatory_elements[0].positions[0] == point_out

if __name__ == "__main__":
    pytest.main()