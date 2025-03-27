from route_planning_msgs.msg import Route, RouteElement, LaneElement, RegulatoryElement
from typing import TypeVar, Union, List
import math

T = TypeVar('T', bound=Union[Route, LaneElement])

def get_width_of_lane_element(lane_element: LaneElement) -> float:
    if not lane_element.has_left_boundary or not lane_element.has_right_boundary:
        return 0.0
    dx = lane_element.left_boundary.point.x - lane_element.right_boundary.point.x
    dy = lane_element.left_boundary.point.y - lane_element.right_boundary.point.y
    return math.sqrt(dx * dx + dy * dy)

def get_suggested_lane_element(route_element: RouteElement) -> LaneElement:
    return route_element.lane_elements[route_element.suggested_lane_idx]

def get_current_suggested_lane_element(route: Route) -> LaneElement:
    # TODO: check if access functions still make sense
    return get_suggested_lane_element(route.remaining_route_elements[0])


def get_width_of_suggested_lane_element(route_element: RouteElement) -> float:
    return get_width_of_lane_element(get_suggested_lane_element(route_element))


def get_width_of_current_suggested_lane_element(route: Route) -> float:
    return get_width_of_suggested_lane_element(route.remaining_route_elements[0])


def get_regulatory_elements(route_element: RouteElement) -> List[RegulatoryElement]:
    return route_element.regulatory_elements


def get_regulatory_element_of_lane_element(lane_element: LaneElement, possible_regulatory_elements: List[RegulatoryElement]) -> List[RegulatoryElement]:
    regulatory_elements = []
    for regulatory_element_idx in lane_element.regulatory_element_idx:
        if regulatory_element_idx >= len(possible_regulatory_elements):
            raise ValueError(f"Regulatory element index out of range: {regulatory_element_idx}")
        regulatory_elements.append(possible_regulatory_elements[regulatory_element_idx])
    return regulatory_elements


def get_regulatory_elements_of_lane_element(route_element: RouteElement, lane_idx: int) -> List[RegulatoryElement]:
    return get_regulatory_element_of_lane_element(route_element.lane_elements[lane_idx], route_element.regulatory_elements)