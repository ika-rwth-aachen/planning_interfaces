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

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <route_planning_msgs/msg/driveable_space.hpp>
#include <route_planning_msgs/msg/route.hpp>

namespace tf2 {

using namespace route_planning_msgs::msg;

// DriveableSpace

template <>
inline void doTransform(const DriveableSpace& ds_in, DriveableSpace& ds_out,
                        const geometry_msgs::msg::TransformStamped& transform) {
  ds_out = ds_in;

  // boundariess
  // left
  for (size_t i = 0; i < ds_in.boundaries.left.size(); i++) {
    doTransform(ds_in.boundaries.left[i], ds_out.boundaries.left[i], transform);
  }
  // right
  for (size_t i = 0; i < ds_in.boundaries.right.size(); i++) {
    doTransform(ds_in.boundaries.right[i], ds_out.boundaries.right[i], transform);
  }

  // restricted_areas
  for (size_t i = 0; i < ds_in.restricted_areas.size(); i++) {
    for (size_t j = 0; j < ds_in.restricted_areas[i].points.size(); j++) {
      geometry_msgs::msg::Point p_in, p_out;
      p_in.x = ds_in.restricted_areas[i].points[j].x;
      p_in.y = ds_in.restricted_areas[i].points[j].y;
      p_in.z = ds_in.restricted_areas[i].points[j].z;
      doTransform(p_in, p_out, transform);
      ds_out.restricted_areas[i].points[j].x = (float)p_out.x;
      ds_out.restricted_areas[i].points[j].y = (float)p_out.y;
      ds_out.restricted_areas[i].points[j].z = (float)p_out.z;
    }
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

  // boundaries
  // left
  for (size_t i = 0; i < route_in.boundaries.left.size(); i++) {
    doTransform(route_in.boundaries.left[i], route_out.boundaries.left[i], transform);
  }
  // right
  for (size_t i = 0; i < route_in.boundaries.right.size(); i++) {
    doTransform(route_in.boundaries.right[i], route_out.boundaries.right[i], transform);
  }

  // driveable space
  doTransform(route_in.driveable_space, route_out.driveable_space, transform);

  // route
  for (size_t i = 0; i < route_in.route_elements.size(); i++) {
    geometry_msgs::msg::Point p_in, p_out;
    p_in.x = route_in.route_elements[i].x;
    p_in.y = route_in.route_elements[i].y;
    p_in.z = 0.0;
    doTransform(p_in, p_out, transform);
    route_out.route_elements[i].x = p_out.x;
    route_out.route_elements[i].y = p_out.y;
    route_out.route_elements[i].z = route_in.route_elements[i].z;
  }

  // lanes
  for (size_t i = 0; i < route_in.lanes.size(); i++) {
    for (size_t j = 0; j < route_in.lanes[i].left.line.size(); j++) {
      doTransform(route_in.lanes[i].left.line[j], route_out.lanes[i].left.line[j], transform);
    }
    for (size_t j = 0; j < route_in.lanes[i].right.line.size(); j++) {
      doTransform(route_in.lanes[i].right.line[j], route_out.lanes[i].right.line[j], transform);
    }
    for (size_t j = 0; j < route_in.lanes[i].centerline.size(); j++) {
      doTransform(route_in.lanes[i].centerline[j], route_out.lanes[i].centerline[j], transform);
    }
  }

  //regulatory_elements
  for (size_t i = 0; i < route_in.regulatory_elements.size(); i++) {
    for (size_t j = 0; j < route_in.regulatory_elements[i].effect_line.size(); j++) {
      geometry_msgs::msg::Point p_in, p_out;
      p_in.x = route_in.regulatory_elements[i].effect_line[j].x;
      p_in.y = route_in.regulatory_elements[i].effect_line[j].y;
      p_in.z = 0.0;
      doTransform(p_in, p_out, transform);
      route_out.regulatory_elements[i].effect_line[j].x = p_out.x;
      route_out.regulatory_elements[i].effect_line[j].y = p_out.y;
      route_out.regulatory_elements[i].effect_line[j].z = route_in.regulatory_elements[i].effect_line[j].z;
    }
    for (size_t j = 0; j < route_in.regulatory_elements[i].sign_positions.size(); j++) {
      doTransform(route_in.regulatory_elements[i].sign_positions[j],
                  route_out.regulatory_elements[i].sign_positions[j], transform);
    }
  }
}

template <>
inline tf2::TimePoint getTimestamp(const Route& route) { return tf2_ros::fromMsg(route.header.stamp); }

template <>
inline std::string getFrameId(const Route& route) { return route.header.frame_id; }

}  // namespace tf2
