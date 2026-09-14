// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

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

TEST(route_planning_msgs, test_estimate_remaining_time) {
  Route route;
  for (const double s : {0.0, 100.0, 200.0}) {
    RouteElement route_element;
    route_element.s = s;
    route_element.suggested_lane_idx = 0;
    LaneElement lane_element;
    lane_element.speed_limit = 36;
    route_element.lane_elements.push_back(lane_element);
    route.route_elements.push_back(route_element);
  }
  route.current_route_element_idx = 0;
  route.destination_route_element_idx = 2;

  EXPECT_NEAR(estimateRemainingTime(route), 20.0 * REMAINING_TIME_ESTIMATION_FACTOR, EPS);
  EXPECT_NEAR(estimateRemainingTime(route, 1.0, 1.0), 20.0, EPS);

  route.route_elements.front().lane_elements.front().speed_limit = LaneElement::SPEED_LIMIT_UNLIMITED;
  EXPECT_NEAR(estimateRemainingTime(route, 10.0, 1.0), 100.0 / UNLIMITED_SPEED_MPS + 10.0, EPS);

  route.destination_route_element_idx = Route::INVALID_ROUTE_ELEMENT_IDX;
  EXPECT_NEAR(estimateRemainingTime(route, 10.0, 1.0), 100.0 / UNLIMITED_SPEED_MPS + 10.0, EPS);
}

TEST(route_planning_msgs, test_get_traveled_route_elements) {
  Route route;
  for (const double s : {0.0, 10.0, 20.0, 30.0}) {
    RouteElement route_element;
    route_element.s = s;
    route.route_elements.push_back(route_element);
  }
  route.starting_route_element_idx = 1;
  route.current_route_element_idx = 3;

  auto traveled = getTraveledRouteElements(route);
  ASSERT_EQ(traveled.size(), 2u);
  EXPECT_DOUBLE_EQ(traveled.front().s, 10.0);
  EXPECT_DOUBLE_EQ(traveled.back().s, 20.0);

  traveled = getTraveledRouteElements(route, true);
  ASSERT_EQ(traveled.size(), 3u);
  EXPECT_DOUBLE_EQ(traveled.front().s, 0.0);

  route.starting_route_element_idx = Route::INVALID_ROUTE_ELEMENT_IDX;
  EXPECT_EQ(getTraveledRouteElements(route).size(), 3u);
  EXPECT_EQ(getTraveledRouteElements(route, true).size(), 3u);

  route.starting_route_element_idx = route.current_route_element_idx;
  EXPECT_TRUE(getTraveledRouteElements(route).empty());

  route.starting_route_element_idx = 3;
  route.current_route_element_idx = 1;
  EXPECT_TRUE(getTraveledRouteElements(route).empty());
  EXPECT_EQ(getTraveledRouteElements(route, true).size(), 1u);

  route.current_route_element_idx = Route::INVALID_ROUTE_ELEMENT_IDX;
  EXPECT_TRUE(getTraveledRouteElements(route).empty());

  route.route_elements.clear();
  route.current_route_element_idx = 0;
  EXPECT_TRUE(getTraveledRouteElements(route).empty());
}

TEST(route_planning_msgs, test_get_remaining_route_elements) {
  Route route;
  for (const double s : {0.0, 10.0, 20.0, 30.0}) {
    RouteElement route_element;
    route_element.s = s;
    route.route_elements.push_back(route_element);
  }
  route.current_route_element_idx = 1;
  route.destination_route_element_idx = 2;

  auto remaining = getRemainingRouteElements(route);
  ASSERT_EQ(remaining.size(), 2u);
  EXPECT_DOUBLE_EQ(remaining[0].s, 10.0);
  EXPECT_DOUBLE_EQ(remaining[1].s, 20.0);

  remaining = getRemainingRouteElements(route, true);
  ASSERT_EQ(remaining.size(), 3u);
  EXPECT_DOUBLE_EQ(remaining.back().s, 30.0);

  route.destination_route_element_idx = Route::INVALID_ROUTE_ELEMENT_IDX;
  remaining = getRemainingRouteElements(route);
  ASSERT_EQ(remaining.size(), 3u);
  EXPECT_DOUBLE_EQ(remaining.front().s, 10.0);
  EXPECT_DOUBLE_EQ(remaining.back().s, 30.0);
  EXPECT_EQ(getRemainingRouteElements(route, true).size(), 3u);

  route.destination_route_element_idx = route.current_route_element_idx;
  remaining = getRemainingRouteElements(route);
  ASSERT_EQ(remaining.size(), 1u);
  EXPECT_DOUBLE_EQ(remaining.front().s, 10.0);

  route.destination_route_element_idx = 0;
  EXPECT_TRUE(getRemainingRouteElements(route).empty());
  EXPECT_EQ(getRemainingRouteElements(route, true).size(), 3u);

  route.current_route_element_idx = route.route_elements.size();
  EXPECT_TRUE(getRemainingRouteElements(route).empty());

  route.route_elements.clear();
  route.current_route_element_idx = 0;
  EXPECT_TRUE(getRemainingRouteElements(route).empty());
}

TEST(route_planning_msgs, test_get_route_element_closest_to_s) {
  Route route;
  for (const double s : {0.0, 1.0, 101.0}) {
    RouteElement route_element;
    route_element.s = s;
    route.route_elements.push_back(route_element);
  }

  EXPECT_EQ(getRouteElementIdxClosestToS(route, 50.0), 1u);
  EXPECT_DOUBLE_EQ(getRouteElementClosestToS(route, 90.0).s, 101.0);
  EXPECT_EQ(getRouteElementIdxClosestToS(route, 50.0, 49.0), 1u);
  EXPECT_THROW(getRouteElementIdxClosestToS(route, 50.0, 48.9), std::runtime_error);
  EXPECT_THROW(getRouteElementClosestToS(route, 90.0, 10.0), std::runtime_error);
  EXPECT_THROW(getRouteElementIdxClosestToS(route, 50.0, -1.0), std::invalid_argument);

  route.route_elements.erase(route.route_elements.begin() + 1, route.route_elements.end());
  EXPECT_EQ(getRouteElementIdxClosestToS(route, 1000.0), 0u);

  route.route_elements.clear();
  EXPECT_THROW(getRouteElementIdxClosestToS(route, 0.0), std::runtime_error);
}

TEST(route_planning_msgs, test_checked_route_getters) {
  Route route;
  RouteElement route_element;
  route.route_elements.push_back(route_element);

  route.current_route_element_idx = Route::INVALID_ROUTE_ELEMENT_IDX;
  EXPECT_THROW(getWidthOfCurrentSuggestedLaneElement(route), std::out_of_range);

  EXPECT_THROW(getRegulatoryElementsOfLaneElement(route_element, 0), std::out_of_range);
  EXPECT_THROW(getRegulatoryElementsOfSuggestedLane(route_element), std::out_of_range);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
