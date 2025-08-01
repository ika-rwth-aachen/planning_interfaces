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

#include <route_planning_msgs/msg/route.hpp>

#include <route_planning_msgs_utils/route_access.hpp>

#include <tf2_route_planning_msgs/tf2_route_planning_msgs.hpp>

namespace gm = geometry_msgs::msg;
using namespace route_planning_msgs::msg;
using namespace route_planning_msgs::route_access;

#include <cmath>

#include <gtest/gtest.h>

TEST(tf2_route_planning_msgs, test_doTransform_Route) {

  gm::Point point_in, point_out;
  point_in.x = 1.0;
  point_in.y = 2.0;
  point_in.z = 3.0;

  point_out.x = 9.0;
  point_out.y = 18.0;
  point_out.z = 33.0;

  gm::Pose pose_in, pose_out;
  pose_in.position = point_in;
  pose_in.orientation.x = 0.0;
  pose_in.orientation.y = 0.0;
  pose_in.orientation.z = 1.0;
  pose_in.orientation.w = 0.0;

  pose_out.position = point_out;
  pose_out.orientation.x = 0.0;
  pose_out.orientation.y = 0.0;
  pose_out.orientation.z = 0.0;
  pose_out.orientation.w = 1.0;

  // Define a LaneBoundary
  LaneBoundary lane_boundary;
  lane_boundary.point = point_in;

  // Define a LaneElement
  LaneElement lane_element;
  lane_element.reference_pose = pose_in;
  setLeftBoundaryOfLaneElement(lane_element, lane_boundary);
  setRightBoundaryOfLaneElement(lane_element, lane_boundary);

  // Define a RegulatoryElement
  RegulatoryElement regulatory_element;
  regulatory_element.reference_line[0] = point_in;
  regulatory_element.reference_line[1] = point_in;
  regulatory_element.positions.push_back(point_in);

  // Define a route element
  RouteElement route_element;
  route_element.lane_elements.push_back(lane_element);
  route_element.left_boundary = point_in;
  route_element.right_boundary = point_in;
  route_element.regulatory_elements.push_back(regulatory_element);

  // Define a route 
  Route route, route_tf;
  route.destination = point_in;
  route.intermediates.push_back(point_in);
  route.intermediates.push_back(point_in);
  route.route_elements.push_back(route_element);

  gm::TransformStamped tf;
  tf.transform.translation.x = 10.0;
  tf.transform.translation.y = 20.0;
  tf.transform.translation.z = 30.0;
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 1.0;
  tf.transform.rotation.w = 0.0;

  tf2::doTransform(route, route_tf, tf);

  // transformed route
  EXPECT_EQ(route_tf.destination, point_out);
  EXPECT_EQ(route_tf.intermediates.size(), 2);
  EXPECT_EQ(route_tf.intermediates[0], point_out);
  EXPECT_EQ(route_tf.intermediates[1], point_out);
  EXPECT_EQ(route_tf.route_elements.size(), 1);
  EXPECT_EQ(route_tf.route_elements[0].lane_elements.size(), 1);
  EXPECT_EQ(route_tf.route_elements[0].left_boundary, point_out);
  EXPECT_EQ(route_tf.route_elements[0].right_boundary, point_out);
  EXPECT_EQ(route_tf.route_elements[0].lane_elements[0].reference_pose, pose_out);
  EXPECT_EQ(route_tf.route_elements[0].lane_elements[0].left_boundary.point, point_out);
  EXPECT_EQ(route_tf.route_elements[0].lane_elements[0].right_boundary.point, point_out);
  EXPECT_EQ(route_tf.route_elements[0].regulatory_elements.size(), 1);
  EXPECT_EQ(route_tf.route_elements[0].regulatory_elements[0].reference_line.size(), 2);
  EXPECT_EQ(route_tf.route_elements[0].regulatory_elements[0].reference_line[0], point_out);
  EXPECT_EQ(route_tf.route_elements[0].regulatory_elements[0].reference_line[1], point_out);
  EXPECT_EQ(route_tf.route_elements[0].regulatory_elements[0].positions[0], point_out);

}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
