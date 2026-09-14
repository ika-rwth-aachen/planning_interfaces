# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: MIT

import math
from typing import List, Optional

DEFAULT_REFERENCE_SPEED_MPS = 50.0 / 3.6
UNLIMITED_SPEED_MPS = 130.0 / 3.6
REMAINING_TIME_ESTIMATION_FACTOR = 1.5

from route_planning_msgs.msg import (
    LaneElement,
    RegulatoryElement,
    Route,
    RouteElement,
)


def get_traveled_route_elements(route: Route, incl_undershoot: bool = False) -> List[RouteElement]:
    """Return the traveled elements available in the received route.

    Args:
        route: Route whose traveled elements are returned.
        incl_undershoot: Whether to include elements before the starting element.

    Returns:
        Traveled elements before the current element. If the starting element is
        outside the route, all available elements before the current one are returned.
        An invalid current index produces an empty list.
    """
    n_route_elements = len(route.route_elements)
    if route.current_route_element_idx >= n_route_elements:
        return []

    start_idx = (
        0
        if incl_undershoot or route.starting_route_element_idx >= n_route_elements
        else route.starting_route_element_idx
    )
    end_idx = route.current_route_element_idx
    if start_idx >= end_idx:
        return []
    return list(route.route_elements[start_idx:end_idx])


def get_remaining_route_elements(route: Route, incl_overshoot: bool = False) -> List[RouteElement]:
    """Return the remaining elements available in the received route.

    The destination element is included when it is present. If the destination
    is outside the received route, all elements through the end are returned.

    Args:
        route: Route whose remaining elements are returned.
        incl_overshoot: Whether to include elements beyond the destination element.

    Returns:
        Remaining elements from the current element onward. An invalid current
        index produces an empty list.
    """
    n_route_elements = len(route.route_elements)
    if route.current_route_element_idx >= n_route_elements:
        return []

    start_idx = route.current_route_element_idx
    end_idx = (
        route.destination_route_element_idx + 1
        if not incl_overshoot and route.destination_route_element_idx < n_route_elements
        else n_route_elements
    )
    if start_idx >= end_idx:
        return []
    return list(route.route_elements[start_idx:end_idx])


def get_idx_of_lane_in_route_element(lane_element: LaneElement, route_element: RouteElement) -> int:
    for idx, candidate in enumerate(route_element.lane_elements):
        if candidate == lane_element or candidate is lane_element:
            return idx
    raise ValueError("Lane element not found in route element")


def get_route_element_idx_closest_to_s(
    route: Route, s: float, max_distance: float = math.inf
) -> int:
    """Return the closest element index within ``max_distance`` meters.

    Args:
        route: Route whose elements are searched.
        s: Longitudinal route position in meters.
        max_distance: Maximum accepted absolute distance in meters. Defaults to
            infinity.

    Returns:
        Index of the route element closest to ``s``.

    Raises:
        ValueError: If ``max_distance`` is negative or NaN, the route is empty,
            or no element is within ``max_distance``.
    """
    if max_distance < 0.0 or math.isnan(max_distance):
        raise ValueError("Maximum distance must be non-negative")
    n_route_elements = len(route.route_elements)
    if not n_route_elements:
        raise ValueError("Cannot find a route element in an empty route")

    closest_idx = min(
        range(n_route_elements),
        key=lambda idx: abs(route.route_elements[idx].s - s),
    )
    if abs(route.route_elements[closest_idx].s - s) > max_distance:
        raise ValueError("No route element found within maximum distance")
    return closest_idx


def get_route_element_closest_to_s(
    route: Route, s: float, max_distance: float = math.inf
) -> RouteElement:
    """Return the closest route element within ``max_distance`` meters.

    Args:
        route: Route whose elements are searched.
        s: Longitudinal route position in meters.
        max_distance: Maximum accepted absolute distance in meters. Defaults to
            infinity.

    Returns:
        Route element closest to ``s``.

    Raises:
        ValueError: If ``max_distance`` is negative or NaN, the route is empty,
            or no element is within ``max_distance``.
    """
    return route.route_elements[get_route_element_idx_closest_to_s(route, s, max_distance)]


def get_width_of_lane_element(lane_element: LaneElement) -> float:
    """Return lane width from boundary points populated during route enrichment.

    Args:
        lane_element: Lane element whose width is calculated.

    Returns:
        Euclidean distance between the left and right boundary points in meters.
    """
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


def estimate_remaining_time(
    route: Route,
    reference_speed_mps: float = DEFAULT_REFERENCE_SPEED_MPS,
    calibration_factor: float = REMAINING_TIME_ESTIMATION_FACTOR,
) -> float:
    """Estimate time over the remaining elements available in this route.

    If the destination is outside a local route window, the result covers only
    the available window and is not an estimate of arrival time at the destination.

    Args:
        route: Route whose remaining travel time is estimated.
        reference_speed_mps: Speed used for unknown speed limits, in meters per second.
        calibration_factor: Factor applied to the raw driving time.

    Returns:
        Estimated travel time in seconds, or zero if fewer than two route elements
        are available or a parameter is non-positive.
    """
    remaining_elements = get_remaining_route_elements(route)
    if len(remaining_elements) < 2 or reference_speed_mps <= 0.0 or calibration_factor <= 0.0:
        return 0.0
    raw_time = 0.0
    for element, next_element in zip(remaining_elements, remaining_elements[1:]):
        speed_limit_kmh = get_suggested_lane_element(element).speed_limit
        speed_mps = speed_limit_kmh / 3.6
        if speed_limit_kmh == LaneElement.SPEED_LIMIT_UNKNOWN:
            speed_mps = reference_speed_mps
        if speed_limit_kmh == LaneElement.SPEED_LIMIT_UNLIMITED:
            speed_mps = UNLIMITED_SPEED_MPS
        raw_time += (next_element.s - element.s) / speed_mps
    return calibration_factor * raw_time


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
    """Return width of the suggested lane from an enriched route element.

    Args:
        route_element: Route element containing the suggested lane.

    Returns:
        Width of the suggested lane in meters.

    Raises:
        IndexError: If the suggested lane index is invalid.
    """
    return get_width_of_lane_element(get_suggested_lane_element(route_element))


def get_width_of_current_suggested_lane_element(route: Route) -> float:
    """Return current suggested-lane width from an enriched route.

    Args:
        route: Route containing the current suggested lane.

    Returns:
        Width of the current suggested lane in meters.

    Raises:
        IndexError: If the current route element or suggested lane index is invalid.
    """
    return get_width_of_lane_element(get_current_suggested_lane_element(route))


def get_regulatory_elements(route_element: RouteElement) -> List[RegulatoryElement]:
    """Return regulatory elements populated during route enrichment.

    An empty result from a non-enriched element does not mean that no regulations apply.

    Args:
        route_element: Enriched route element containing regulatory elements.

    Returns:
        Regulatory elements stored in ``route_element``.
    """
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
    """Return a lane's regulatory elements from an enriched route element.

    Args:
        route_element: Enriched route element containing the lane and regulatory elements.
        lane_idx: Index of the lane whose regulatory elements are returned.

    Returns:
        Regulatory elements referenced by the selected lane.

    Raises:
        IndexError: If ``lane_idx`` is invalid.
        ValueError: If a regulatory element index is invalid.
    """
    if lane_idx < 0 or lane_idx >= len(route_element.lane_elements):
        raise IndexError(f"Lane index out of range: {lane_idx}")
    return get_regulatory_elements_for_lane_element(
        route_element.lane_elements[lane_idx], route_element.regulatory_elements
    )


def get_regulatory_elements_of_suggested_lane(route_element: RouteElement) -> List[RegulatoryElement]:
    """Return the suggested lane's regulatory elements from an enriched route element.

    Args:
        route_element: Enriched route element containing the suggested lane.

    Returns:
        Regulatory elements referenced by the suggested lane.

    Raises:
        IndexError: If the suggested lane index is invalid.
        ValueError: If a regulatory element index is invalid.
    """
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
