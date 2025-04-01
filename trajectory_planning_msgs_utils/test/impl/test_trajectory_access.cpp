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

#include <gtest/gtest.h>
#include <random>

using namespace trajectory_planning_msgs;
using namespace trajectory_planning_msgs::trajectory_access;

std::uniform_real_distribution<double> uniform_distribution(-1, 1);
std::default_random_engine random_engine;

double randomValue() { return uniform_distribution(random_engine); }
static const double EPS = 1e-12;

TEST(trajectory_planning_msgs, test_set_get_REFERENCE) {
  Trajectory tra;
  int nSamplePoints = 8;
  initializeTrajectory(tra, REFERENCE::TYPE_ID, nSamplePoints);

  double val;

  for (int i = 0; i < getSamplePointSize(tra); i++) {
    // set/getT
    val = randomValue();
    setT(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getT(tra, i));

    // set/getX
    val = randomValue();
    setX(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getX(tra, i));

    // set/getY
    val = randomValue();
    setY(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getY(tra, i));

    // set/getV
    val = randomValue();
    setV(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getV(tra, i));
  }

  Trajectory theta_tra;
  nSamplePoints = 3;
  initializeTrajectory(theta_tra, REFERENCE::TYPE_ID, nSamplePoints);
  // setX_0/Y_0
  setX(theta_tra, 0.0, 0);
  setY(theta_tra, 0.0, 0);
  // setX_1/Y_1
  setX(theta_tra, 1.0, 1);
  setY(theta_tra, 0.0, 1);
  // setX_2/Y_2
  setX(theta_tra, 2.0, 2);
  setY(theta_tra, 1.0, 2);

  EXPECT_NEAR(getTheta(theta_tra, 0), 0.0, EPS);
  EXPECT_NEAR(getTheta(theta_tra, 1), std::atan(1.0/2.0), EPS);
  EXPECT_NEAR(getTheta(theta_tra, 2), std::atan(1.0), EPS);
}

TEST(trajectory_planning_msgs, test_set_get_DRIVABLE) {
  Trajectory tra;
  int nSamplePoints = 8;
  initializeTrajectory(tra, DRIVABLE::TYPE_ID, nSamplePoints);

  double val;

  for (int i = 0; i < getSamplePointSize(tra); i++) {
    // set/getT
    val = randomValue();
    setT(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getT(tra, i));

    // set/getX
    val = randomValue();
    setX(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getX(tra, i));

    // set/getY
    val = randomValue();
    setY(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getY(tra, i));

    // set/getTheta
    val = randomValue();
    setTheta(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getTheta(tra, i));

    // set/getV
    val = randomValue();
    setV(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getV(tra, i));

    // set/getA
    val = randomValue();
    setA(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getA(tra, i));

    // set/getDeltaAck
    val = randomValue();
    setDeltaAck(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getDeltaAck(tra, i));

    // set/getS
    val = randomValue();
    setS(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getS(tra, i));
  }
}

TEST(trajectory_planning_msgs, test_set_get_DRIVABLERWS) {
  Trajectory tra;
  int nSamplePoints = 8;
  initializeTrajectory(tra, DRIVABLERWS::TYPE_ID, nSamplePoints);

  double val;
  double delta_front = 1.2;
  double delta_rear = 0.2;
  double distance_front_axle = 1.5;
  double distance_rear_axle = 1.9;
  double beta = atan((distance_rear_axle / (distance_front_axle + distance_rear_axle)) * tan(delta_front) 
                  + (distance_front_axle / (distance_front_axle + distance_rear_axle)) * tan(delta_rear));

  for (int i = 0; i < getSamplePointSize(tra); i++) {
    // set/getT
    val = randomValue();
    setT(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getT(tra, i));

    // set/getX
    val = randomValue();
    setX(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getX(tra, i));

    // set/getY
    val = randomValue();
    setY(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getY(tra, i));

     // set/getV
    val = randomValue();
    setV(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getV(tra, i));

    // set/getTheta
    val = randomValue();
    setTheta(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getTheta(tra, i));

    // set/getA
    val = randomValue();
    setA(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getA(tra, i));

    // set/getBeta(direct and indirect)
    setBeta(tra, beta, i);
    EXPECT_DOUBLE_EQ(beta, getBeta(tra, i));
    setBeta(tra, delta_front, delta_rear, distance_front_axle, distance_rear_axle, i);
    EXPECT_NEAR(getBeta(tra, i), beta, EPS);

    // set/getDeltaFront
    val = randomValue();
    setDeltaFront(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getDeltaFront(tra, i));

    // set/getDeltaRear
    val = randomValue();
    setDeltaRear(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getDeltaRear(tra, i));

    // set/getS
    val = randomValue();
    setS(tra, val, i);
    EXPECT_DOUBLE_EQ(val, getS(tra, i));
  }
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}