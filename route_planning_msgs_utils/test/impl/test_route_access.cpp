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

  gm::Point lane_boundary_left;
  lane_boundary_left.x = 1.0;
  lane_boundary_left.y = 2.0;
  lane_element.lane_boundary_left = lane_boundary_left;
  gm::Point lane_boundary_right;
  lane_boundary_right.x = 3.0;
  lane_boundary_right.y = 4.0;
  lane_element.lane_boundary_right = lane_boundary_right;
  route_element.current_lane_id = 0;
  route_element.lane_elements.push_back(lane_element);
  route.remaining_route.push_back(route_element);

  EXPECT_NEAR(getWidthOfLaneElement(lane_element), sqrt(8), EPS);
  EXPECT_NEAR(getWidthOfCurrentLaneElement(route_element), 2.8284271247461903, EPS);
  EXPECT_NEAR(getWidthOfCurrentLaneElement(route), 2.8284271247461903, EPS);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}