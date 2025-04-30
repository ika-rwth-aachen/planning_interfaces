/** ============================================================================
MIT License

Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

#pragma once

#include <cmath>
#include <optional>

#include <route_planning_msgs_utils/utils.h>

namespace route_planning_msgs {

namespace route_access {

inline std::vector<RouteElement> getTraveledRouteElements(const Route& route, const bool incl_undershoot = false) {
  auto first = route.route_elements.begin();
  if (!incl_undershoot) {
    first += route.starting_route_element_idx;
  }
  auto behind_last = route.route_elements.begin() + route.current_route_element_idx;
  return std::vector<RouteElement>(first, behind_last);
}

inline std::vector<RouteElement> getRemainingRouteElements(const Route& route, const bool incl_overshoot = false) {
  auto first = route.route_elements.begin() + route.current_route_element_idx;
  auto behind_last = route.route_elements.begin() + route.destination_route_element_idx + 1;
  if (incl_overshoot) {
    behind_last = route.route_elements.end();
  }
  return std::vector<RouteElement>(first, behind_last);
}

inline double getWidthOfLaneElement(const LaneElement& lane_element) {
  double dx = lane_element.left_boundary.point.x - lane_element.right_boundary.point.x;
  double dy = lane_element.left_boundary.point.y - lane_element.right_boundary.point.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline LaneElement getSuggestedLaneElement(const RouteElement& route_element) {
  return route_element.lane_elements[route_element.suggested_lane_idx];
}

inline LaneElement getCurrentSuggestedLaneElement(const Route& route) {
  // TODO: check if access functions still make sense
  return getSuggestedLaneElement(route.route_elements[route.current_route_element_idx]);
}

inline std::optional<LaneElement> getFollowingLaneElement(const LaneElement& lane_element,
                                                          const RouteElement& following_route_element) {
  if (!lane_element.has_following_lane_idx ||
      (lane_element.following_lane_idx >= following_route_element.lane_elements.size())) {
    return std::nullopt;
  }
  return following_route_element.lane_elements[lane_element.following_lane_idx];
}

inline std::optional<LaneElement> getPrecedingLaneElement(const size_t lane_element_idx,
                                                          const RouteElement& preceding_route_element) {
  for (const auto& preceding_lane_element : preceding_route_element.lane_elements) {
    if (preceding_lane_element.has_following_lane_idx &&
        preceding_lane_element.following_lane_idx == lane_element_idx) {
      return preceding_lane_element;
    }
  }
  return std::nullopt;
}

inline double getWidthOfSuggestedLaneElement(const RouteElement& route_element) {
  return getWidthOfLaneElement(getSuggestedLaneElement(route_element));
}

inline double getWidthOfCurrentSuggestedLaneElement(const Route& route) {
  return getWidthOfSuggestedLaneElement(route.route_elements[route.current_route_element_idx]);
}

inline std::vector<RegulatoryElement> getRegulatoryElements(const RouteElement& route_element) {
  return route_element.regulatory_elements;
}

inline std::vector<RegulatoryElement> getRegulatoryElementOfLaneElement(
    const LaneElement& lane_element, const std::vector<RegulatoryElement> possible_regulatory_elements) {
  std::vector<RegulatoryElement> regulatory_elements;
  for (const auto& regulatory_element_idx : lane_element.regulatory_element_idcs) {
    if (regulatory_element_idx >= possible_regulatory_elements.size()) {
      throw std::invalid_argument("Regulatory element index out of range: " + std::to_string(regulatory_element_idx));
    }
    regulatory_elements.push_back(possible_regulatory_elements[regulatory_element_idx]);
  }
  return regulatory_elements;
}

inline std::vector<RegulatoryElement> getRegulatoryElementsOfLaneElement(const RouteElement& route_element,
                                                                         const uint8_t lane_idx) {
  return getRegulatoryElementOfLaneElement(route_element.lane_elements[lane_idx], route_element.regulatory_elements);
}

}  // namespace route_access

}  // namespace route_planning_msgs