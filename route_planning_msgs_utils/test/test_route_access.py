import math
import pytest

from geometry_msgs.msg import Point
from route_planning_msgs.msg import (
    LaneBoundary,
    LaneElement,
    RegulatoryElement,
    Route,
    RouteElement,
)
from geometry_msgs.msg import Point

from route_planning_msgs_utils.route_getters import (
    get_adjacent_lane,
    get_current_suggested_lane_element,
    get_following_lane_element,
    get_following_lane_element_idx,
    get_idx_of_lane_in_route_element,
    get_lane_change_direction,
    get_preceding_lane_element_idx,
    get_regulatory_elements,
    get_regulatory_elements_of_lane_element,
    get_regulatory_elements_of_suggested_lane,
    get_regulatory_element_of_lane_element,
    get_remaining_route_elements,
    get_route_element_closest_to_s,
    get_route_element_idx_closest_to_s,
    get_traveled_route_elements,
    get_width_of_current_suggested_lane_element,
    get_width_of_lane_element,
    get_width_of_suggested_lane_element,
    has_adjacent_lane,
    has_left_adjacent_lane,
    has_right_adjacent_lane,
)
from route_planning_msgs_utils.route_setters import (
    set_left_boundary_of_lane_element_from_point,
    set_right_boundary_of_lane_element_from_point,
)

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


def _make_lane(left_xy, right_xy):
    lane = LaneElement()
    lane.left_boundary.point.x, lane.left_boundary.point.y = left_xy
    lane.right_boundary.point.x, lane.right_boundary.point.y = right_xy
    return lane


def test_route_helpers():
    route = Route()
    route.starting_route_element_idx = 0
    route.current_route_element_idx = 1
    route.destination_route_element_idx = 2

    route_element_0 = RouteElement()
    route_element_0.s = 0.0
    lane0 = _make_lane((0.0, 1.0), (0.0, 0.0))
    lane0.has_following_lane_idx = True
    lane0.following_lane_idx = 0
    route_element_0.lane_elements.append(lane0)
    route_element_0.suggested_lane_idx = 0

    route_element_1 = RouteElement()
    route_element_1.s = 10.0
    route_element_1.suggested_lane_idx = 0
    route_element_1.will_change_suggested_lane = True
    lane1_a = _make_lane((1.0, 1.0), (1.0, 0.0))
    lane1_a.has_following_lane_idx = True
    lane1_a.following_lane_idx = 0
    lane1_b = _make_lane((1.5, 1.0), (1.5, 0.0))
    lane1_b.has_following_lane_idx = True
    lane1_b.following_lane_idx = 1
    lane1_b.regulatory_element_idcs.append(0)
    reg = RegulatoryElement()
    route_element_1.regulatory_elements.append(reg)
    route_element_1.lane_elements.extend([lane1_a, lane1_b])

    route_element_2 = RouteElement()
    route_element_2.s = 20.0
    route_element_2.suggested_lane_idx = 1
    lane2_a = _make_lane((2.0, 1.0), (2.0, 0.0))
    lane2_b = _make_lane((2.5, 1.0), (2.5, 0.0))
    route_element_2.lane_elements.extend([lane2_a, lane2_b])

    route.route_elements.extend([route_element_0, route_element_1, route_element_2])

    traveled = get_traveled_route_elements(route)
    assert traveled == [route_element_0]

    remaining = get_remaining_route_elements(route)
    assert remaining == [route_element_1, route_element_2]

    assert get_route_element_idx_closest_to_s(route, 11.0) == 1
    assert get_route_element_closest_to_s(route, 11.0) is route_element_1

    current_lane = get_current_suggested_lane_element(route)
    assert current_lane is lane1_a

    assert get_idx_of_lane_in_route_element(lane1_b, route_element_1) == 1

    following_idx = get_following_lane_element_idx(lane0, route_element_1)
    assert following_idx == 0
    assert get_following_lane_element(lane0, route_element_1) is lane1_a

    preceding_idx = get_preceding_lane_element_idx(1, route_element_1)
    assert preceding_idx == 1
    assert get_following_lane_element_idx(lane1_b, route_element_2) == 1

    regulatory_elements = get_regulatory_elements(route_element_1)
    assert regulatory_elements == [reg]

    lane_reg_elements = get_regulatory_elements_of_lane_element(route_element_1, 1)
    assert lane_reg_elements == [reg]
    assert get_regulatory_element_of_lane_element(lane1_b, route_element_1.regulatory_elements) == [reg]

    suggested_reg_elements = get_regulatory_elements_of_suggested_lane(route_element_1)
    assert suggested_reg_elements == []

    assert has_adjacent_lane(route_element_1, 0, 1)
    assert has_right_adjacent_lane(route_element_1, 0)
    assert has_left_adjacent_lane(route_element_1, 1)
    assert get_adjacent_lane(route_element_1, 0, 1) is lane1_b

    lane_change_direction = get_lane_change_direction(route_element_1, route_element_2)
    assert lane_change_direction == 1


if __name__ == "__main__":
    pytest.main()
