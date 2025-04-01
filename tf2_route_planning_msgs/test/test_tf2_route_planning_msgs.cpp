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

#include <tf2_route_planning_msgs/tf2_route_planning_msgs.hpp>

namespace gm = geometry_msgs::msg;
using namespace route_planning_msgs::msg;

#include <cmath>

#include <gtest/gtest.h>

static const double EPS = 1e-12;

TEST(tf2_route_planning_msgs, test_doTransform_Route) {

  Route route, route_tf;
  route.destination.x = 1.0;
  route.destination.y = 2.0;
  route.destination.z = 3.0;

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
  EXPECT_NEAR(route_tf.destination.x, 9.0, EPS);
  EXPECT_NEAR(route_tf.destination.y, 18.0, EPS);
  EXPECT_NEAR(route_tf.destination.z, 33.0, EPS);

}


int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
