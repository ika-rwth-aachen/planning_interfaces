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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_planning_msgs/msg/trajectory.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>

namespace tf2 {
namespace gm = geometry_msgs::msg;
using namespace trajectory_planning_msgs::msg;
using Time = tf2::TimePoint;
#ifndef STAMP2TIME
#define STAMP2TIME
/**
 * @brief Converts a ROS2 builtin_interfaces::msg::Time message to a tf2_ros::Time object.
 * 
 * @param t The ROS2 builtin_interfaces::msg::Time message to be converted.
 * @return Time The corresponding tf2_ros::Time object.
 */
inline Time stampToTime(const builtin_interfaces::msg::Time& t) { return tf2_ros::fromMsg(t); }
#endif

#ifndef STAMPS2TIMEDELTA
#define STAMPS2TIMEDELTA
/**
 * @brief Computes the time difference in seconds between two ROS2 Time messages.
 *
 * This function takes two `builtin_interfaces::msg::Time` objects and calculates the 
 * difference between them in seconds. It converts the `builtin_interfaces::msg::Time` 
 * objects to `rclcpp::Time` objects and then computes the difference.
 *
 * @param t1 The first time stamp.
 * @param t2 The second time stamp.
 * @return The time difference (t2-t1) in seconds as a double.
 */
inline double stampsToTimeDelta(const builtin_interfaces::msg::Time& t1, const builtin_interfaces::msg::Time& t2) { return (rclcpp::Time(t2)-rclcpp::Time(t1)).seconds(); }
#endif
}  // namespace tf2

#include <tf2_trajectory_planning_msgs/impl/tf2_trajectory_planning_msgs.h>
