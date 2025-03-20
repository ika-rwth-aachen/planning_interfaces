#pragma once

#include <cmath>

#include <route_planning_msgs_utils/impl/utils.h>

namespace route_planning_msgs {

namespace route_access {

inline double getWidthOfLaneElement(const LaneElement& lane_element) {
  if (!lane_element.has_left_boundary || !lane_element.has_right_boundary) {
    return 0.0;
  }
  double dx = lane_element.left_boundary.point.x - lane_element.right_boundary.point.x;
  double dy = lane_element.left_boundary.point.y - lane_element.right_boundary.point.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline LaneElement getSuggestedLaneElement(const RouteElement& route_element) {
  return route_element.lane_elements[route_element.suggested_lane_idx];
}

inline LaneElement getCurrentSuggestedLaneElement(const Route& route) {
  // TODO: check if access functions still make sense
  return getSuggestedLaneElement(route.remaining_route_elements[0]);
}

inline double getWidthOfSuggestedLaneElement(const RouteElement& route_element) {
  return getWidthOfLaneElement(getSuggestedLaneElement(route_element));
}

inline double getWidthOfCurrentSuggestedLaneElement(const Route& route) {
  return getWidthOfSuggestedLaneElement(route.remaining_route_elements[0]);
}

inline std::vector<RegulatoryElement> getRegulatoryElements(const RouteElement& route_element) {
  return route_element.regulatory_elements;
}

inline std::vector<RegulatoryElement> getRegulatoryElementOfLaneElement(const LaneElement& lane_element, const std::vector<RegulatoryElement> possible_regulatory_elements) {
  std::vector<RegulatoryElement> regulatory_elements;
  if (lane_element.has_regulatory_elements) {
    for (const auto& regulatory_element_idx : lane_element.regulatory_element_idx) {
      if (regulatory_element_idx >= possible_regulatory_elements.size()) {
        throw std::invalid_argument("Regulatory element index out of range: " + std::to_string(regulatory_element_idx));
      }
      regulatory_elements.push_back(possible_regulatory_elements[regulatory_element_idx]);
    }
  }
  return regulatory_elements;
}

inline std::vector<RegulatoryElement> getRegulatoryElementsOfLaneElement(const RouteElement& route_element, const uint8_t lane_idx) {
  return getRegulatoryElementOfLaneElement(route_element.lane_elements[lane_idx], route_element.regulatory_elements);
}

}  // namespace route_access

}  // namespace route_planning_msgs