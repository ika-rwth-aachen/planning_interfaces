import pytest
import math

from route_planning_msgs.msg import Route, RouteElement, LaneElement
from geometry_msgs.msg import Point

from route_planning_msgs_utils.route_getters import get_width_of_lane_element, get_width_of_suggested_lane_element, get_width_of_current_suggested_lane_element

EPS = 1e-12

def test_getters():
    route = Route()
    route_element = RouteElement()
    lane_element = LaneElement()

    left_boundary = Point(1.0, 2.0)
    lane_element.left_boundary = left_boundary
    lane_element.has_left_boundary = True

    right_boundary = Point(3.0, 4.0)
    lane_element.right_boundary = right_boundary
    lane_element.has_right_boundary = True

    route_element.suggested_lane_idx = 0
    route_element.lane_elements.append(lane_element)
    route.remaining_route_elements.append(route_element)

    assert math.isclose(get_width_of_lane_element(lane_element), math.sqrt(8), rel_tol=EPS)
    assert math.isclose(get_width_of_suggested_lane_element(route_element), 2.8284271247461903, rel_tol=EPS)
    assert math.isclose(get_width_of_current_suggested_lane_element(route), 2.8284271247461903, rel_tol=EPS)

if __name__ == "__main__":
    pytest.main()