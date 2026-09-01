// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <iostream>

#include <route_planning_msgs_utils/utils.h>

namespace route_planning_msgs {

namespace route_access {

inline void setLaneBoundary(LaneBoundary& lane_boundary, const gm::Point& point, const uint8_t type=LaneBoundary::TYPE_UNKNOWN) {
  lane_boundary.point = point;
  if (type > 4) {
    throw std::invalid_argument("Invalid lane boundary type: " + std::to_string(type));
  }
  lane_boundary.type = type;
}

inline void setLeftBoundaryOfLaneElement(LaneElement& lane_element, const LaneBoundary& left_boundary) {
  lane_element.left_boundary = left_boundary;
}

inline void setLeftBoundaryOfLaneElement(LaneElement& lane_element, const gm::Point& point, const uint8_t type=LaneBoundary::TYPE_UNKNOWN) {
  LaneBoundary left_boundary;
  setLaneBoundary(left_boundary, point, type);
  setLeftBoundaryOfLaneElement(lane_element, left_boundary);
}

inline void setRightBoundaryOfLaneElement(LaneElement& lane_element, const LaneBoundary& right_boundary) {
  lane_element.right_boundary = right_boundary;
}

inline void setRightBoundaryOfLaneElement(LaneElement& lane_element, const gm::Point& point, const uint8_t type=LaneBoundary::TYPE_UNKNOWN) {
  LaneBoundary right_boundary;
  setLaneBoundary(right_boundary, point, type);
  setRightBoundaryOfLaneElement(lane_element, right_boundary);
}

}  // namespace route_access

}  // namespace route_planning_msgs