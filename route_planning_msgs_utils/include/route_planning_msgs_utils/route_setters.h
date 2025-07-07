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
  lane_element.has_left_boundary = true;
}

inline void setLeftBoundaryOfLaneElement(LaneElement& lane_element, const gm::Point& point, const uint8_t type=LaneBoundary::TYPE_UNKNOWN) {
  LaneBoundary left_boundary;
  setLaneBoundary(left_boundary, point, type);
  setLeftBoundaryOfLaneElement(lane_element, left_boundary);
}

inline void setRightBoundaryOfLaneElement(LaneElement& lane_element, const LaneBoundary& right_boundary) {
  lane_element.right_boundary = right_boundary;
  lane_element.has_right_boundary = true;
}

inline void setRightBoundaryOfLaneElement(LaneElement& lane_element, const gm::Point& point, const uint8_t type=LaneBoundary::TYPE_UNKNOWN) {
  LaneBoundary right_boundary;
  setLaneBoundary(right_boundary, point, type);
  setRightBoundaryOfLaneElement(lane_element, right_boundary);
}

}  // namespace route_access

}  // namespace route_planning_msgs