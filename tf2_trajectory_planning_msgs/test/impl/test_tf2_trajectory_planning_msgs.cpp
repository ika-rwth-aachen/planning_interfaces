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

#include <cmath>

#include <gtest/gtest.h>

using namespace trajectory_planning_msgs::trajectory_access;

static const double EPS = 1e-12;

TEST(tf2_trajectory_planning_msgs, test_doTransform_REFERENCE) {

  Trajectory trajectory, trajectory_tf;
  initializeTrajectory(trajectory, REFERENCE::TYPE_ID, 2);

  for (int i = 0; i < getSamplePointSize(trajectory); i++) {
    setT(trajectory, i*1.0, i);
    setX(trajectory, i*1.0, i);
    setY(trajectory, 1.0, i);
    setV(trajectory, 1.0, i);
  }

  gm::TransformStamped tf;
  tf.transform.translation.x = 10.0;
  tf.transform.translation.y = 20.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = sin(M_PI / 4);
  tf.transform.rotation.w = cos(M_PI / 4);

  tf2::doTransform(trajectory, trajectory_tf, tf);

  for (int i = 0; i < getSamplePointSize(trajectory); i++) {

    // transformed state
    EXPECT_NEAR(getX(trajectory_tf, i), 9, EPS);
    EXPECT_NEAR(getY(trajectory_tf, i), i + 20, EPS);
    EXPECT_NEAR(getTheta(trajectory_tf, i), M_PI / 2, EPS);

    // transform-invariant state
    EXPECT_NEAR(getV(trajectory_tf, i), 1.0, EPS);
  }
}


TEST(tf2_trajectory_planning_msgs, test_doTransform_DRIVABLE) {

  Trajectory trajectory, trajectory_tf;
  initializeTrajectory(trajectory, DRIVABLE::TYPE_ID, 2);

  for (int i = 0; i < getSamplePointSize(trajectory); i++) {
    setT(trajectory, 1.0, i);
    setX(trajectory, 1.0, i);
    setY(trajectory, 2.0, i);
    setV(trajectory, 1.0, i);
    setTheta(trajectory, M_PI, i);
    setA(trajectory, 1.0, i);
    setDeltaAck(trajectory, 1.0, i);
    setS(trajectory, 1.0, i);
  }

  gm::TransformStamped tf;
  tf.transform.translation.x = 10.0;
  tf.transform.translation.y = 20.0;
  tf.transform.translation.z = 30.0;
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 1.0;
  tf.transform.rotation.w = 0.0;

  tf2::doTransform(trajectory, trajectory_tf, tf);

  for (int i = 0; i < getSamplePointSize(trajectory); i++) {

    // transformed state
    EXPECT_NEAR(getX(trajectory_tf, i), 9.0, EPS);
    EXPECT_NEAR(getY(trajectory_tf, i), 18.0, EPS);
    EXPECT_NEAR(getTheta(trajectory_tf, i), 0.0, EPS);

    // transform-invariant state
    EXPECT_NEAR(getV(trajectory_tf, i), 1.0, EPS);
    EXPECT_NEAR(getA(trajectory_tf, i), 1.0, EPS);
    EXPECT_NEAR(getDeltaAck(trajectory_tf, i), 1.0, EPS);
    EXPECT_NEAR(getS(trajectory_tf, i), 1.0, EPS);
  }
}

TEST(tf2_trajectory_planning_msgs, test_doTransform_DRIVABLERWS) {

  Trajectory trajectory, trajectory_tf;
  initializeTrajectory(trajectory, DRIVABLERWS::TYPE_ID, 2);

  double delta_front = 1.2;
  double delta_rear = 0.2;
  double distance_front_axle = 1.5;
  double distance_rear_axle = 1.9;
  double beta = atan((distance_rear_axle / (distance_front_axle + distance_rear_axle)) * tan(delta_front) 
                  + (distance_front_axle / (distance_front_axle + distance_rear_axle)) * tan(delta_rear));

  for (int i = 0; i < getSamplePointSize(trajectory); i++) {
    setT(trajectory, 1.0, i);
    setX(trajectory, 1.0, i);
    setY(trajectory, 2.0, i);
    setV(trajectory, 1.0, i);
    setTheta(trajectory, M_PI, i);
    setA(trajectory, 1.0, i);
    setBeta(trajectory, delta_front, delta_rear, distance_front_axle, distance_rear_axle, i);
    setDeltaFront(trajectory, M_PI, i);
    setDeltaRear(trajectory, M_PI, i);
    setS(trajectory, 1.0, i);
  }

  gm::TransformStamped tf;
  tf.transform.translation.x = 10.0;
  tf.transform.translation.y = 20.0;
  tf.transform.translation.z = 30.0;
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 1.0;
  tf.transform.rotation.w = 0.0;

  tf2::doTransform(trajectory, trajectory_tf, tf);

  for (int i = 0; i < getSamplePointSize(trajectory); i++) {

    // transformed state
    EXPECT_NEAR(getX(trajectory_tf, i), 9.0, EPS);
    EXPECT_NEAR(getY(trajectory_tf, i), 18.0, EPS);
    EXPECT_NEAR(getTheta(trajectory_tf, i), 0.0, EPS);

    // transform-invariant state
    EXPECT_NEAR(getV(trajectory_tf, i), 1.0, EPS);
    EXPECT_NEAR(getA(trajectory_tf, i), 1.0, EPS);
    EXPECT_NEAR(getBeta(trajectory_tf, i), beta, EPS);
    EXPECT_NEAR(getDeltaFront(trajectory_tf, i), M_PI, EPS);
    EXPECT_NEAR(getDeltaRear(trajectory_tf, i), M_PI, EPS);
    EXPECT_NEAR(getS(trajectory_tf, i), 1.0, EPS);
  }
}


int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
