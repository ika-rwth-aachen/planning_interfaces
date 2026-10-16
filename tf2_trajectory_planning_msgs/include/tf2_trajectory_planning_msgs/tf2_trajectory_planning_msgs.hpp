// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_planning_msgs/msg/trajectory.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>

namespace tf2_trajectory_planning_msgs {
/**
 * @brief Converts a ROS2 builtin_interfaces::msg::Time message to a tf2_ros::Time object.
 * 
 * @param t The ROS2 builtin_interfaces::msg::Time message to be converted.
 * @return Time The corresponding tf2_ros::Time object.
 */
inline tf2::TimePoint stampToTime(const builtin_interfaces::msg::Time& t) { return tf2_ros::fromMsg(t); }

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
}  // namespace tf2_trajectory_planning_msgs

#include <tf2/convert.hpp>
#include <tf2/utils.hpp>

namespace tf2 {
namespace gm = geometry_msgs::msg;
using namespace trajectory_planning_msgs::msg;
using Time = tf2::TimePoint;

using namespace trajectory_planning_msgs::trajectory_access;

inline void doTransform(const std::vector<double>& state_in, std::vector<double>& state_out,
                        const unsigned char& type_id, const gm::TransformStamped& transform, const double time_offset) {
  state_out = state_in;

  double x = getX(state_in, type_id);
  double y = getY(state_in, type_id);
  double theta = 0;
  double t = getT(state_in, type_id);
  if (type_id == DRIVABLE::TYPE_ID || type_id == DRIVABLERWS::TYPE_ID) theta = getTheta(state_in, type_id);

  gm::PoseStamped pose_in;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);

  pose_in.pose.position.x = x;
  pose_in.pose.position.y = y;
  pose_in.pose.position.z = 0;
  pose_in.pose.orientation = tf2::toMsg(q);

  gm::PoseStamped pose_out;
  doTransform(pose_in, pose_out, transform);

  setX(state_out, type_id, pose_out.pose.position.x);
  setY(state_out, type_id, pose_out.pose.position.y);
  setT(state_out, type_id, t + time_offset);
  if (type_id == DRIVABLE::TYPE_ID || type_id == DRIVABLERWS::TYPE_ID) setTheta(state_out, type_id, tf2::getYaw(pose_out.pose.orientation));
}

template <>
inline void doTransform(const Trajectory& trajectory_in, Trajectory& trajectory_out,
                        const gm::TransformStamped& transform) {
  trajectory_out = trajectory_in;
  trajectory_out.header.stamp = transform.header.stamp;
  trajectory_out.header.frame_id = transform.header.frame_id;
  double time_offset = tf2_trajectory_planning_msgs::stampsToTimeDelta(transform.header.stamp, trajectory_in.header.stamp);

  for (int i = 0; i < getSamplePointSize(trajectory_in); i++) {
    std::vector<double> state_in = getState(trajectory_in, i);
    std::vector<double> state_out;
    unsigned char type_id = trajectory_in.type_id;
    doTransform(state_in, state_out, type_id, transform, time_offset);
    setState(trajectory_out, state_out, i);
  }
}

template <>
inline Time getTimestamp(const Trajectory& trajectory) {
  const Time& t = tf2_trajectory_planning_msgs::stampToTime(trajectory.header.stamp);
  return t;
}

template <>
inline std::string getFrameId(const Trajectory& trajectory) {
  return trajectory.header.frame_id;
}

}  // namespace tf2
