// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

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
