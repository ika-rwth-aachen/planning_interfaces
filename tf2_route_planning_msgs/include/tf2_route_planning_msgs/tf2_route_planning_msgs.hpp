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

#include <tf2/convert.h>
#include <tf2_ros/buffer_interface.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <route_planning_msgs/msg/route.hpp>

namespace tf2 {

using namespace route_planning_msgs::msg;

// LaneBoundary
template <>
inline void doTransform(const LaneBoundary& lane_boundary_in, LaneBoundary& lane_boundary_out,
                        const geometry_msgs::msg::TransformStamped& transform) {
  lane_boundary_out = lane_boundary_in;

  // point
  doTransform(lane_boundary_in.point, lane_boundary_out.point, transform);
}

// LaneElement
template <>
inline void doTransform(const LaneElement& lane_element_in, LaneElement& lane_element_out,
                        const geometry_msgs::msg::TransformStamped& transform) {
  lane_element_out = lane_element_in;

  doTransform(lane_element_in.reference_pose, lane_element_out.reference_pose, transform);
  doTransform(lane_element_in.left_boundary, lane_element_out.left_boundary, transform);
  doTransform(lane_element_in.right_boundary, lane_element_out.right_boundary, transform);
}

// RegulatoryElement
template <>
inline void doTransform(const RegulatoryElement& regulatory_element_in, RegulatoryElement& regulatory_element_out,
                        const geometry_msgs::msg::TransformStamped& transform) {
  regulatory_element_out = regulatory_element_in;

  // effect line
  for (size_t i = 0; i < regulatory_element_in.reference_line.size(); i++) {
    doTransform(regulatory_element_in.reference_line[i], regulatory_element_out.reference_line[i], transform);
  }

  // sign positions
  for (size_t i = 0; i < regulatory_element_in.positions.size(); i++) {
    doTransform(regulatory_element_in.positions[i], regulatory_element_out.positions[i], transform);
  }
}

// RouteElement
template <>
inline void doTransform(const RouteElement& route_element_in, RouteElement& route_element_out,
                        const geometry_msgs::msg::TransformStamped& transform) {
  route_element_out = route_element_in;

  // lane elements
  for (size_t i = 0; i < route_element_in.lane_elements.size(); i++) {
    doTransform(route_element_in.lane_elements[i], route_element_out.lane_elements[i], transform);
  }

  // boundaries (drivable space)
  doTransform(route_element_in.left_boundary, route_element_out.left_boundary, transform);
  doTransform(route_element_in.right_boundary, route_element_out.right_boundary, transform);

  // regulatory elements
  for (size_t i = 0; i < route_element_in.regulatory_elements.size(); i++) {
    doTransform(route_element_in.regulatory_elements[i], route_element_out.regulatory_elements[i], transform);
  }
}

// Route
template <>
inline void doTransform(const Route& route_in, Route& route_out, const geometry_msgs::msg::TransformStamped& transform) {
  route_out = route_in;
  route_out.header.stamp = transform.header.stamp;
  route_out.header.frame_id = transform.header.frame_id;

  // destination
  doTransform(route_in.destination, route_out.destination, transform);

  // route elements
  for (size_t i = 0; i < route_in.route_elements.size(); i++) {
    doTransform(route_in.route_elements[i], route_out.route_elements[i], transform);
  }
}

template <>
inline tf2::TimePoint getTimestamp(const Route& route) { return tf2_ros::fromMsg(route.header.stamp); }

template <>
inline std::string getFrameId(const Route& route) { return route.header.frame_id; }

}  // namespace tf2
