#include <gtest/gtest.h>
#include <random>

using namespace route_planning_msgs;
using namespace route_planning_msgs::route_access;

std::uniform_real_distribution<double> uniform_distribution(-1, 1);
std::default_random_engine random_engine;

double randomValue() { return uniform_distribution(random_engine); }
static const double EPS = 1e-12;

TEST(route_planning_msgs, test_getters) {
  Route route;
  RouteElement route_element;
  LaneElement lane_element;

  gm::Point left_boundary;
  left_boundary.x = 1.0;
  left_boundary.y = 2.0;
  lane_element.left_boundary.point = left_boundary;
  lane_element.has_left_boundary = true;
  gm::Point right_boundary;
  right_boundary.x = 3.0;
  right_boundary.y = 4.0;
  lane_element.right_boundary.point = right_boundary;
  lane_element.has_right_boundary = true;
  route_element.suggested_lane_idx = 0;
  route_element.lane_elements.push_back(lane_element);
  route.remaining_route_elements.push_back(route_element);

  EXPECT_NEAR(getWidthOfLaneElement(lane_element), sqrt(8), EPS);
  EXPECT_NEAR(getWidthOfSuggestedLaneElement(route_element), 2.8284271247461903, EPS);
  EXPECT_NEAR(getWidthOfCurrentSuggestedLaneElement(route), 2.8284271247461903, EPS);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}