import pytest
import math

from route_planning_msgs.msg import Route, RouteElement, LaneElement, LaneBoundary
from geometry_msgs.msg import Point

from route_planning_msgs_utils.route_getters import get_width_of_lane_element, get_width_of_suggested_lane_element, get_width_of_current_suggested_lane_element
from route_planning_msgs_utils.route_setters import set_left_boundary_of_lane_element_from_point, set_right_boundary_of_lane_element_from_point

EPS = 1e-12

def test_setters():
    lane_element = LaneElement()

    left_boundary = Point()
    left_boundary.x = 1.0
    left_boundary.y = 2.0
    right_boundary = Point()
    right_boundary.x = 3.0
    right_boundary.y = 4.0
    set_left_boundary_of_lane_element_from_point(lane_element, left_boundary)
    set_right_boundary_of_lane_element_from_point(lane_element, right_boundary)

    assert lane_element.left_boundary.point.x == 1.0
    assert lane_element.left_boundary.point.y == 2.0
    assert lane_element.left_boundary.type == LaneBoundary.TYPE_UNKNOWN
    assert lane_element.right_boundary.point.x == 3.0
    assert lane_element.right_boundary.point.y == 4.0
    assert lane_element.right_boundary.type == LaneBoundary.TYPE_UNKNOWN

def test_getters():
    route = Route()
    route_element = RouteElement()
    lane_element = LaneElement()
    left_boundary = LaneBoundary()
    right_boundary = LaneBoundary()

    left_boundary.point.x = 1.0
    left_boundary.point.y = 2.0
    lane_element.left_boundary = left_boundary

    right_boundary.point.x = 3.0
    right_boundary.point.y = 4.0
    lane_element.right_boundary = right_boundary

    route_element.suggested_lane_idx = 0
    route_element.lane_elements.append(lane_element)
    route.route_elements.append(route_element)

    assert math.isclose(get_width_of_lane_element(lane_element), math.sqrt(8), rel_tol=EPS)
    assert math.isclose(get_width_of_suggested_lane_element(route_element), 2.8284271247461903, rel_tol=EPS)
    assert math.isclose(get_width_of_current_suggested_lane_element(route), 2.8284271247461903, rel_tol=EPS)

if __name__ == "__main__":
    pytest.main()