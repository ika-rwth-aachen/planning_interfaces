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

#include <route_planning_msgs_utils/route_access.hpp>

#include <gtest/gtest.h>
#include <random>

using namespace route_planning_msgs::msg;
using namespace route_planning_msgs::route_access;

std::uniform_real_distribution<double> uniform_distribution(-1, 1);
std::default_random_engine random_engine;

double randomValue() { return uniform_distribution(random_engine); }
static const double EPS = 1e-12;

TEST(route_planning_msgs, test_setters) {

  LaneElement lane_element;

  geometry_msgs::msg::Point left_boundary;
  left_boundary.x = 1.0;
  left_boundary.y = 2.0;
  geometry_msgs::msg::Point right_boundary;
  right_boundary.x = 3.0;
  right_boundary.y = 4.0;
  setLeftBoundaryOfLaneElement(lane_element, left_boundary);
  setRightBoundaryOfLaneElement(lane_element, right_boundary);

  EXPECT_EQ(lane_element.left_boundary.point.x, 1.0);
  EXPECT_EQ(lane_element.left_boundary.point.y, 2.0);
  EXPECT_EQ(lane_element.left_boundary.type, LaneBoundary::TYPE_UNKNOWN);
  EXPECT_EQ(lane_element.right_boundary.point.x, 3.0);
  EXPECT_EQ(lane_element.right_boundary.point.y, 4.0);
  EXPECT_EQ(lane_element.right_boundary.type, LaneBoundary::TYPE_UNKNOWN);
  
}

TEST(route_planning_msgs, test_getters) {
  Route route;
  RouteElement route_element;
  LaneElement lane_element;

  geometry_msgs::msg::Point left_boundary;
  left_boundary.x = 1.0;
  left_boundary.y = 2.0;
  geometry_msgs::msg::Point right_boundary;
  right_boundary.x = 3.0;
  right_boundary.y = 4.0;
  setLeftBoundaryOfLaneElement(lane_element, left_boundary);
  setRightBoundaryOfLaneElement(lane_element, right_boundary);
  route_element.suggested_lane_idx = 0;
  route_element.lane_elements.push_back(lane_element);
  route_element.is_enriched = true;
  route.route_elements.push_back(route_element);
  route.starting_route_element_idx = 0;
  route.current_route_element_idx = 0;
  route.destination_route_element_idx = 0;

  EXPECT_NEAR(getWidthOfLaneElement(lane_element), sqrt(8), EPS);
  EXPECT_NEAR(getWidthOfSuggestedLaneElement(route_element), 2.8284271247461903, EPS);
  EXPECT_NEAR(getWidthOfCurrentSuggestedLaneElement(route), 2.8284271247461903, EPS);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}