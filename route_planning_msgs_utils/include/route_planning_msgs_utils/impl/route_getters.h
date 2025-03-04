#pragma once

#include <cmath>

#include <route_planning_msgs_utils/impl/utils.h>

namespace route_planning_msgs {

namespace route_access {

inline double getWidthOfLaneElement(const LaneElement& lane_element) {
  double dx = lane_element.lane_boundary_left.x - lane_element.lane_boundary_right.x;
  double dy = lane_element.lane_boundary_left.y - lane_element.lane_boundary_right.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline LaneElement getCurrentLaneElement(const RouteElement& route_element) {
  return route_element.lane_elements[route_element.current_lane_id];
}

inline LaneElement getCurrentLaneElement(const Route& route) {
  return getCurrentLaneElement(route.remaining_route[0]);
}

inline double getWidthOfCurrentLaneElement(const RouteElement& route_element) {
  return getWidthOfLaneElement(getCurrentLaneElement(route_element));
}

inline double getWidthOfCurrentLaneElement(const Route& route) {
  return getWidthOfCurrentLaneElement(route.remaining_route[0]);
}

}  // namespace route_access

}  // namespace route_planning_msgs