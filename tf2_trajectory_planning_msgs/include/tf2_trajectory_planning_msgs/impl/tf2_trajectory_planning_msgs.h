// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <tf2/convert.hpp>
#include <tf2/utils.hpp>

namespace tf2 {

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
  double time_offset = stampsToTimeDelta(transform.header.stamp, trajectory_in.header.stamp);

  for (int i = 0; i < getSamplePointSize(trajectory_in); i++) {
    std::vector<double> state_in = getState(trajectory_in, i);
    std::vector<double> state_out;
    unsigned char type_id = trajectory_in.type_id;
    doTransform(state_in, state_out, type_id, transform, time_offset);
    setState(trajectory_out, state_out, i);
  }
}

#ifdef ROS1
#define TF2_TRAJECTORY_PLANNING_MSGS_TIME_TYPE const Time&
#else
#define TF2_TRAJECTORY_PLANNING_MSGS_TIME_TYPE Time
#endif
template <>
inline TF2_TRAJECTORY_PLANNING_MSGS_TIME_TYPE getTimestamp(const Trajectory& trajectory) {
  const Time& t = stampToTime(trajectory.header.stamp);
  return t;
}

#ifdef ROS1
#define TF2_TRAJECTORY_PLANNING_MSGS_FRAME_TYPE const std::string&
#else
#define TF2_TRAJECTORY_PLANNING_MSGS_FRAME_TYPE std::string
#endif
template <>
inline TF2_TRAJECTORY_PLANNING_MSGS_FRAME_TYPE getFrameId(const Trajectory& trajectory) {
  return trajectory.header.frame_id;
}

}  // namespace tf2
