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
from typing import List, Optional

from route_planning_msgs.msg import (
    LaneElement,
    RegulatoryElement,
    Route,
    RouteElement,
)


def get_traveled_route_elements(route: Route, incl_undershoot: bool = False) -> List[RouteElement]:
    n_route_elements = len(route.route_elements)
    start_idx = 0 if incl_undershoot else route.starting_route_element_idx
    end_idx = route.current_route_element_idx
    start_idx = min(start_idx, n_route_elements)
    end_idx = min(end_idx, n_route_elements)
    if start_idx >= end_idx:
        return []
    return list(route.route_elements[start_idx:end_idx])


def get_remaining_route_elements(route: Route, incl_overshoot: bool = False) -> List[RouteElement]:
    n_route_elements = len(route.route_elements)
    start_idx = route.current_route_element_idx
    end_idx = n_route_elements if incl_overshoot else route.destination_route_element_idx + 1
    start_idx = min(start_idx, n_route_elements)
    end_idx = min(end_idx, n_route_elements)
    if start_idx >= end_idx:
        return []
    return list(route.route_elements[start_idx:end_idx])


def get_idx_of_lane_in_route_element(lane_element: LaneElement, route_element: RouteElement) -> int:
    for idx, candidate in enumerate(route_element.lane_elements):
        if candidate == lane_element or candidate is lane_element:
            return idx
    raise ValueError("Lane element not found in route element")


def get_route_element_idx_closest_to_s(route: Route, s: float) -> int:
    n_route_elements = len(route.route_elements)
    if n_route_elements < 2:
        raise ValueError("Not enough route elements to calculate difference")

    min_difference = 2 * abs(route.route_elements[1].s - route.route_elements[0].s)
    closest_idx = min(
        range(n_route_elements),
        key=lambda idx: abs(route.route_elements[idx].s - s),
    )

    if abs(route.route_elements[closest_idx].s - s) >= min_difference:
        raise ValueError("No route element found within acceptable difference")

    return closest_idx


def get_route_element_closest_to_s(route: Route, s: float) -> RouteElement:
    return route.route_elements[get_route_element_idx_closest_to_s(route, s)]


def get_width_of_lane_element(lane_element: LaneElement) -> float:
    dx = lane_element.left_boundary.point.x - lane_element.right_boundary.point.x
    dy = lane_element.left_boundary.point.y - lane_element.right_boundary.point.y
    return math.sqrt(dx * dx + dy * dy)


def get_suggested_lane_element(route_element: RouteElement) -> LaneElement:
    lane_idx = route_element.suggested_lane_idx
    if lane_idx >= len(route_element.lane_elements):
        raise IndexError(f"Suggested lane index out of range: {lane_idx}")
    return route_element.lane_elements[lane_idx]


def get_current_suggested_lane_element(route: Route) -> LaneElement:
    idx = route.current_route_element_idx
    if idx >= len(route.route_elements):
        raise IndexError(f"Current route element index out of range: {idx}")
    return get_suggested_lane_element(route.route_elements[idx])


def get_following_lane_element_idx(
    lane_element: LaneElement, following_route_element: RouteElement
) -> Optional[int]:
    if (
        not lane_element.has_following_lane_idx
        or lane_element.following_lane_idx >= len(following_route_element.lane_elements)
    ):
        return None
    return lane_element.following_lane_idx


def get_following_lane_element(
    lane_element: LaneElement, following_route_element: RouteElement
) -> Optional[LaneElement]:
    idx = get_following_lane_element_idx(lane_element, following_route_element)
    if idx is None:
        return None
    return following_route_element.lane_elements[idx]


def get_preceding_lane_element(
    lane_element_idx: int, preceding_route_element: RouteElement
) -> Optional[LaneElement]:
    for preceding_lane in preceding_route_element.lane_elements:
        if (
            preceding_lane.has_following_lane_idx
            and preceding_lane.following_lane_idx == lane_element_idx
        ):
            return preceding_lane
    return None


def get_preceding_lane_element_idx(
    lane_element_idx: int, preceding_route_element: RouteElement
) -> Optional[int]:
    preceding_lane = get_preceding_lane_element(lane_element_idx, preceding_route_element)
    if preceding_lane is None:
        return None
    return get_idx_of_lane_in_route_element(preceding_lane, preceding_route_element)


def get_width_of_suggested_lane_element(route_element: RouteElement) -> float:
    return get_width_of_lane_element(get_suggested_lane_element(route_element))


def get_width_of_current_suggested_lane_element(route: Route) -> float:
    return get_width_of_lane_element(get_current_suggested_lane_element(route))


def get_regulatory_elements(route_element: RouteElement) -> List[RegulatoryElement]:
    return list(route_element.regulatory_elements)


def get_regulatory_elements_for_lane_element(
    lane_element: LaneElement, possible_regulatory_elements: List[RegulatoryElement]
) -> List[RegulatoryElement]:
    regulatory_elements: List[RegulatoryElement] = []
    for regulatory_element_idx in lane_element.regulatory_element_idcs:
        if regulatory_element_idx >= len(possible_regulatory_elements):
            raise ValueError(f"Regulatory element index out of range: {regulatory_element_idx}")
        regulatory_elements.append(possible_regulatory_elements[regulatory_element_idx])
    return regulatory_elements


def get_regulatory_elements_of_lane_element(
    route_element: RouteElement, lane_idx: int
) -> List[RegulatoryElement]:
    if lane_idx >= len(route_element.lane_elements):
        raise IndexError(f"Lane index out of range: {lane_idx}")
    return get_regulatory_elements_for_lane_element(
        route_element.lane_elements[lane_idx], route_element.regulatory_elements
    )


def get_regulatory_elements_of_suggested_lane(route_element: RouteElement) -> List[RegulatoryElement]:
    return get_regulatory_elements_of_lane_element(route_element, route_element.suggested_lane_idx)


def has_adjacent_lane(route_element: RouteElement, lane_idx: int, lane_diff_idx: int) -> bool:
    if lane_idx >= len(route_element.lane_elements):
        raise IndexError(f"Lane index out of range: {lane_idx}")
    adjacent_lane_idx = lane_idx + lane_diff_idx
    return 0 <= adjacent_lane_idx < len(route_element.lane_elements)


def has_right_adjacent_lane(route_element: RouteElement, lane_idx: int) -> bool:
    return has_adjacent_lane(route_element, lane_idx, 1)


def has_left_adjacent_lane(route_element: RouteElement, lane_idx: int) -> bool:
    return has_adjacent_lane(route_element, lane_idx, -1)


def get_adjacent_lane(route_element: RouteElement, lane_idx: int, lane_diff_idx: int) -> LaneElement:
    if not has_adjacent_lane(route_element, lane_idx, lane_diff_idx):
        raise ValueError(f"No adjacent lane found for lane index: {lane_idx}")
    return route_element.lane_elements[lane_idx + lane_diff_idx]


def get_lane_change_direction(route_element: RouteElement, following_route_element: RouteElement) -> int:
    if not route_element.will_change_suggested_lane:
        raise ValueError("Route element does not indicate a change of suggested lane")

    current_lane_idx = route_element.suggested_lane_idx
    preceding_idx = get_preceding_lane_element_idx(
        following_route_element.suggested_lane_idx, route_element
    )
    if preceding_idx is None:
        raise ValueError(
            f"No preceding lane element found for route element with suggested lane {route_element.suggested_lane_idx}"
        )
    return int(preceding_idx) - int(current_lane_idx)


# Backwards compatibility alias (deprecated name)
def get_regulatory_element_of_lane_element(
    lane_element: LaneElement, possible_regulatory_elements: List[RegulatoryElement]
) -> List[RegulatoryElement]:
    return get_regulatory_elements_for_lane_element(lane_element, possible_regulatory_elements)
