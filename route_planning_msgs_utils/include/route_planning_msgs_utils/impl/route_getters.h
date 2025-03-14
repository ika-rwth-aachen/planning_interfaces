#pragma once

#include <cmath>

#include <route_planning_msgs_utils/impl/utils.h>

namespace route_planning_msgs {

namespace route_access {

inline double getWidthOfLaneElement(const LaneElement& lane_element) {
  if (!lane_element.has_left_boundary || !lane_element.has_right_boundary) {
    return 0.0;
  }
  double dx = lane_element.left_boundary.x - lane_element.right_boundary.x;
  double dy = lane_element.left_boundary.y - lane_element.right_boundary.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline LaneElement getCurrentLaneElement(const RouteElement& route_element) {
  return route_element.lane_elements[route_element.suggested_lane_idx];
}

inline LaneElement getCurrentLaneElement(const Route& route) {
  // TODO: check if access functions still make sense
  return getCurrentLaneElement(route.route_elements[0]);
}

inline double getWidthOfCurrentLaneElement(const RouteElement& route_element) {
  return getWidthOfLaneElement(getCurrentLaneElement(route_element));
}

inline double getWidthOfCurrentLaneElement(const Route& route) {
  return getWidthOfCurrentLaneElement(route.route_elements[0]);
}

}  // namespace route_access

}  // namespace route_planning_msgs