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

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_planning_msgs/Trajectory.h>
#include <trajectory_planning_msgs_utils/trajectory_access.h>

namespace tf2 {
namespace gm = geometry_msgs;
using namespace trajectory_planning_msgs;
using Time = ros::Time;
#ifndef STAMP2TIME
#define STAMP2TIME
inline const Time& stampToTime(const ros::Time& t) { return t; }
#endif

#ifndef STAMPS2TIMEDELTA
#define STAMPS2TIMEDELTA
/**
 * @brief Computes the time difference between two ROS time stamps.
 *
 * This function calculates the difference in seconds between two given ROS time stamps.
 *
 * @param t1 The first ROS time stamp.
 * @param t2 The second ROS time stamp.
 * @return The time difference (t2-t1) in seconds as a double.
 */
inline double stampsToTimeDelta(const ros::Time& t1, const ros::Time& t2) { return (t2-t1).toSec(); }
#endif
}  // namespace tf2

#include <tf2_trajectory_planning_msgs/impl/tf2_trajectory_planning_msgs.h>
