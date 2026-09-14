// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <cmath>
#include <limits>
#include <optional>

#include <route_planning_msgs_utils/utils.h>

namespace route_planning_msgs {

namespace route_access {

inline constexpr double DEFAULT_REFERENCE_SPEED_MPS = 50.0 / 3.6;
inline constexpr double UNLIMITED_SPEED_MPS = 130.0 / 3.6;
inline constexpr double REMAINING_TIME_ESTIMATION_FACTOR = 1.5;

/**
 * @brief Returns traveled elements available in the received route.
 *
 * Excludes the current element. If the starting point is outside the received
 * route, returns all available elements before the current element. With
 * @p incl_undershoot, also includes elements before an available starting
 * element. Returns an empty vector if the current index is invalid.
 */
inline std::vector<RouteElement> getTraveledRouteElements(const Route& route, const bool incl_undershoot = false) {
  const size_t n = route.route_elements.size();
  if (route.current_route_element_idx >= n) {
    return {};
  }

  const size_t start_idx = incl_undershoot || route.starting_route_element_idx >= n
                               ? 0
                               : static_cast<size_t>(route.starting_route_element_idx);
  const size_t end_idx = static_cast<size_t>(route.current_route_element_idx);

  if (start_idx >= end_idx) {
    return {};
  }
  return std::vector<RouteElement>(route.route_elements.begin() + start_idx,
                                   route.route_elements.begin() + end_idx);
}

/**
 * @brief Returns remaining elements within the received route.
 *
 * Includes the current element and, if present, the destination element. If the
 * destination is outside the received route, returns all elements from the
 * current element to the end of the received route. With @p incl_overshoot,
 * also includes elements beyond the destination. Returns an empty vector if
 * the current index is invalid.
 */
inline std::vector<RouteElement> getRemainingRouteElements(const Route& route, const bool incl_overshoot = false) {
  const size_t n = route.route_elements.size();
  if (route.current_route_element_idx >= n) {
    return {};
  }

  const size_t start_idx = static_cast<size_t>(route.current_route_element_idx);
  const size_t end_idx = !incl_overshoot && route.destination_route_element_idx < n
                             ? static_cast<size_t>(route.destination_route_element_idx) + 1
                             : n;

  if (start_idx >= end_idx) {
    return {};
  }
  return std::vector<RouteElement>(route.route_elements.begin() + start_idx,
                                   route.route_elements.begin() + end_idx);
}

/**
 * @brief Returns pointers to remaining elements in the received route.
 *
 * Follows the same local-window rules as getRemainingRouteElements, but lets
 * callers modify the route elements in place.
 */
inline std::vector<RouteElement*> getRemainingRouteElementsAsPointers(Route& route, const bool incl_overshoot = false) {
  const size_t n = route.route_elements.size();
  if (route.current_route_element_idx >= n) {
    return {};
  }

  const size_t start_idx = static_cast<size_t>(route.current_route_element_idx);
  const size_t end_idx = !incl_overshoot && route.destination_route_element_idx < n
                             ? static_cast<size_t>(route.destination_route_element_idx) + 1
                             : n;
  if (start_idx >= end_idx) {
    return {};
  }

  std::vector<RouteElement*> result;
  result.reserve(end_idx - start_idx);
  for (size_t idx = start_idx; idx < end_idx; ++idx) {
    result.push_back(&route.route_elements[idx]);
  }
  return result;
}

inline size_t getIdxOfLaneInRouteElement(const LaneElement& lane_element, const RouteElement& route_element) {
  auto it = std::find(route_element.lane_elements.begin(), route_element.lane_elements.end(), lane_element);
  if (it == route_element.lane_elements.end()) {
    throw std::invalid_argument("Lane element not found in route element");
  }
  return std::distance(route_element.lane_elements.begin(), it);
}

/**
 * @brief Returns the index of the route element closest to @p s.
 *
 * @param route Route whose elements are searched.
 * @param s Longitudinal route position in meters.
 * @param max_distance Maximum accepted absolute distance in meters. By default, the closest element is returned regardless of distance.
 * @throws std::invalid_argument If @p max_distance is negative or NaN.
 * @throws std::runtime_error If the route is empty or no element is within
 *                            @p max_distance.
 */
inline size_t getRouteElementIdxClosestToS(const Route& route, const double s,
                                           const double max_distance = std::numeric_limits<double>::infinity()) {
  if (max_distance < 0.0 || std::isnan(max_distance)) {
    throw std::invalid_argument("Maximum distance must be non-negative");
  }
  if (route.route_elements.empty()) {
    throw std::runtime_error("Cannot find a route element in an empty route");
  }

  auto it = std::min_element(route.route_elements.begin(), route.route_elements.end(),
                             [s](const RouteElement& a, const RouteElement& b) {
                               return std::abs(a.s - s) < std::abs(b.s - s);
                             });
  if (std::abs(it->s - s) > max_distance) {
    throw std::runtime_error("No route element found within maximum distance");
  }
  return std::distance(route.route_elements.begin(), it);
}

inline RouteElement getRouteElementClosestToS(const Route& route, const double s,
                                              const double max_distance = std::numeric_limits<double>::infinity()) {
  return route.route_elements[getRouteElementIdxClosestToS(route, s, max_distance)];
}

/**
 * @brief Returns lane width from boundary points populated during route enrichment.
 *
 * Non-enriched routes do not provide meaningful lane-boundary geometry.
 */
inline double getWidthOfLaneElement(const LaneElement& lane_element) {
  double dx = lane_element.left_boundary.point.x - lane_element.right_boundary.point.x;
  double dy = lane_element.left_boundary.point.y - lane_element.right_boundary.point.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline LaneElement getSuggestedLaneElement(const RouteElement& route_element) {
  if (route_element.suggested_lane_idx >= route_element.lane_elements.size()) {
    throw std::out_of_range("Suggested lane index " + std::to_string(route_element.suggested_lane_idx) +
                            " out of range (" + std::to_string(route_element.lane_elements.size()) + ")");
  }
  return route_element.lane_elements[route_element.suggested_lane_idx];
}

inline LaneElement getCurrentSuggestedLaneElement(const Route& route) {
  if (route.current_route_element_idx >= route.route_elements.size()) {
    throw std::out_of_range("Current route element index " + std::to_string(route.current_route_element_idx) +
                            " out of range (" + std::to_string(route.route_elements.size()) + ")");
  }
  return getSuggestedLaneElement(route.route_elements[route.current_route_element_idx]);
}

/**
 * @brief Estimates remaining travel time from route segments and speed limits.
 *
 * Unspecified speed limits use @p reference_speed_mps. The result is multiplied
 * by @p calibration_factor to account for non-driving time. If the destination
 * is outside the received route, this estimates only the time to the end of the
 * available local route window, not the time to the destination.
 */
inline double estimateRemainingTime(const Route& route,
                                    const double reference_speed_mps = DEFAULT_REFERENCE_SPEED_MPS,
                                    const double calibration_factor = REMAINING_TIME_ESTIMATION_FACTOR) {
  const std::vector<RouteElement> remaining_route_elements =
      getRemainingRouteElements(route);
  if (remaining_route_elements.size() < 2 || reference_speed_mps <= 0.0 || calibration_factor <= 0.0) {
    return 0.0;
  }
  double remaining_time = 0.0;
  for (size_t i = 0; i + 1 < remaining_route_elements.size(); ++i) {
    const auto& route_element = remaining_route_elements[i];
    const auto& next_route_element = remaining_route_elements[i + 1];
    const uint8_t speed_limit_kmh = getSuggestedLaneElement(route_element).speed_limit;
    double speed_mps = static_cast<double>(speed_limit_kmh) / 3.6;
    if (speed_limit_kmh == LaneElement::SPEED_LIMIT_UNKNOWN) {
      speed_mps = reference_speed_mps;
    }
    if (speed_limit_kmh == LaneElement::SPEED_LIMIT_UNLIMITED) {
      speed_mps = UNLIMITED_SPEED_MPS;
    }
    remaining_time += (next_route_element.s - route_element.s) / speed_mps;
  }
  return calibration_factor * remaining_time;
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
  // Lane boundaries are only meaningful for enriched route elements.
  return getWidthOfLaneElement(getSuggestedLaneElement(route_element));
}

inline double getWidthOfCurrentSuggestedLaneElement(const Route& route) {
  return getWidthOfLaneElement(getCurrentSuggestedLaneElement(route));
}

/**
 * @brief Returns regulatory elements populated during route enrichment.
 *
 * An empty result on a non-enriched route does not imply that no regulations
 * apply to the corresponding road section.
 */
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

/**
 * @brief Returns a lane's regulatory elements from an enriched route element.
 */
inline std::vector<RegulatoryElement> getRegulatoryElementsOfLaneElement(const RouteElement& route_element,
                                                                         const uint8_t lane_idx) {
  if (lane_idx >= route_element.lane_elements.size()) {
    throw std::out_of_range("Lane index " + std::to_string(lane_idx) + " out of range (" +
                            std::to_string(route_element.lane_elements.size()) + ")");
  }
  return getRegulatoryElementsOfLaneElement(route_element.lane_elements[lane_idx], route_element.regulatory_elements);
}

inline std::vector<RegulatoryElement> getRegulatoryElementsOfSuggestedLane(const RouteElement& route_element) {
  return getRegulatoryElementsOfLaneElement(route_element, route_element.suggested_lane_idx);
}

/**
 * @brief Returns pointers to the suggested lane's regulatory elements.
 *
 * Preserves the lane's regulatory-element index order and lets callers modify
 * the regulatory elements in the route element in place.
 */
inline std::vector<RegulatoryElement*> getRegulatoryElementsOfSuggestedLaneAsPointers(RouteElement& route_element) {
  if (route_element.suggested_lane_idx >= route_element.lane_elements.size()) {
    throw std::out_of_range("Suggested lane index " + std::to_string(route_element.suggested_lane_idx) +
                            " out of range (" + std::to_string(route_element.lane_elements.size()) + ")");
  }

  const auto& lane = route_element.lane_elements[route_element.suggested_lane_idx];
  std::vector<RegulatoryElement*> result;
  result.reserve(lane.regulatory_element_idcs.size());
  for (const auto idx : lane.regulatory_element_idcs) {
    if (idx >= route_element.regulatory_elements.size()) {
      throw std::invalid_argument("Regulatory element index out of range: " + std::to_string(idx));
    }
    result.push_back(&route_element.regulatory_elements[idx]);
  }
  return result;
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
