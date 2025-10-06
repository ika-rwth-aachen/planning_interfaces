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
  const size_t n = route.route_elements.size();
  size_t start_idx = incl_undershoot ? 0 : route.starting_route_element_idx;
  size_t end_idx = route.current_route_element_idx;
  // Clamp indices to valid range
  start_idx = std::min(start_idx, n);
  end_idx = std::min(end_idx, n);
  if (start_idx >= end_idx) {
    return {};
  }
  return std::vector<RouteElement>(route.route_elements.begin() + start_idx,
                                   route.route_elements.begin() + end_idx);
}

inline std::vector<RouteElement> getRemainingRouteElements(const Route& route, const bool incl_overshoot = false) {
  const size_t n = route.route_elements.size();
  size_t start_idx = route.current_route_element_idx;
  size_t end_idx = incl_overshoot ? n : (route.destination_route_element_idx + 1);
  // Clamp indices to valid range
  start_idx = std::min(start_idx, n);
  end_idx = std::min(end_idx, n);
  if (start_idx >= end_idx) {
    return {};
  }
  return std::vector<RouteElement>(route.route_elements.begin() + start_idx,
                                   route.route_elements.begin() + end_idx);
}

inline size_t getIdxOfLaneInRouteElement(const LaneElement& lane_element, const RouteElement& route_element) {
  auto it = std::find(route_element.lane_elements.begin(), route_element.lane_elements.end(), lane_element);
  if (it == route_element.lane_elements.end()) {
    throw std::invalid_argument("Lane element not found in route element");
  }
  return std::distance(route_element.lane_elements.begin(), it);
}

inline size_t getRouteElementIdxClosestToS(const Route& route, const double s) {
  if (route.route_elements.size() < 2) {
    throw std::runtime_error("Not enough route elements to calculate difference");
  }

  double min_difference = 2 * std::abs(route.route_elements[1].s - route.route_elements[0].s);
  auto it = std::min_element(route.route_elements.begin(), route.route_elements.end(),
                             [s](const RouteElement& a, const RouteElement& b) {
                               return std::abs(a.s - s) < std::abs(b.s - s);
                             });

  if (it == route.route_elements.end() || std::abs(it->s - s) >= min_difference) {
    throw std::runtime_error("No route element found within acceptable difference");
  }

  return std::distance(route.route_elements.begin(), it);
}

inline RouteElement getRouteElementClosestToS(const Route& route, const double s) {
  return route.route_elements[getRouteElementIdxClosestToS(route, s)];
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
  return getSuggestedLaneElement(route.route_elements[route.current_route_element_idx]);
}

inline std::optional<size_t> getFollowingLaneElementIdx(const LaneElement& lane_element,
                                                        const RouteElement& following_route_element) {
  if (!lane_element.has_following_lane_idx ||
      (lane_element.following_lane_idx >= following_route_element.lane_elements.size())) {
    return std::nullopt;
  }
  return lane_element.following_lane_idx;
}

inline std::optional<LaneElement> getFollowingLaneElement(const LaneElement& lane_element,
                                                          const RouteElement& following_route_element) {
  if (auto result = getFollowingLaneElementIdx(lane_element, following_route_element)) {
    return following_route_element.lane_elements[*result];
  }
  return std::nullopt;
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

inline std::optional<size_t> getPrecedingLaneElementIdx(const size_t lane_element_idx,
                                                        const RouteElement& preceding_route_element) {
  if (auto result = getPrecedingLaneElement(lane_element_idx, preceding_route_element)) {
    return getIdxOfLaneInRouteElement(*result, preceding_route_element);
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

inline std::vector<RegulatoryElement> getRegulatoryElementsOfLaneElement(
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
  return getRegulatoryElementsOfLaneElement(route_element.lane_elements[lane_idx], route_element.regulatory_elements);
}

inline std::vector<RegulatoryElement> getRegulatoryElementsOfSuggestedLane(const RouteElement& route_element) {
  return getRegulatoryElementsOfLaneElement(route_element, route_element.suggested_lane_idx);
}

inline bool hasAdjacentLane(const RouteElement& route_element, const size_t lane_idx, const int lane_diff_idx){
  int adjacent_lane_idx = lane_idx + lane_diff_idx;
  if (lane_idx >= route_element.lane_elements.size()) {
    throw std::invalid_argument("Lane index out of range: " + std::to_string(lane_idx));
  }

  if (adjacent_lane_idx >= static_cast<int>(route_element.lane_elements.size()) || adjacent_lane_idx < 0) {
    return false;
  }
  return true;
}

inline bool hasRightAdjacentLane(const RouteElement& route_element, const size_t lane_idx) {
  return hasAdjacentLane(route_element, lane_idx, 1);
}

inline bool hasLeftAdjacentLane(const RouteElement& route_element, const size_t lane_idx) {
  return hasAdjacentLane(route_element, lane_idx, -1);
}

inline LaneElement getAdjacentLane(const RouteElement& route_element, const size_t lane_idx, const int lane_diff_idx) {
  if (!hasAdjacentLane(route_element, lane_idx, lane_diff_idx)) {
    throw std::invalid_argument("No adjacent lane found for lane index: " + std::to_string(lane_idx));
  }
  int adjacent_lane_idx = lane_idx + lane_diff_idx;
  return route_element.lane_elements[adjacent_lane_idx];
}

inline int getLaneChangeDirection(const RouteElement& route_element, const RouteElement& following_route_element) {
  if (!route_element.will_change_suggested_lane) {
    throw std::invalid_argument("Route element does not indicate a change of suggested lane");
  }
  size_t current_lane_idx = route_element.suggested_lane_idx;
  size_t target_lane_idx;
  if (auto result = getPrecedingLaneElementIdx(following_route_element.suggested_lane_idx, route_element)) {
    target_lane_idx = *result;
  } else {
    throw std::invalid_argument("No preceding lane element found for route element with suggested lane " + std::to_string(route_element.suggested_lane_idx));
  }
  int lane_change_direction = target_lane_idx - current_lane_idx;
  return lane_change_direction;
}

}  // namespace route_access

}  // namespace route_planning_msgs
