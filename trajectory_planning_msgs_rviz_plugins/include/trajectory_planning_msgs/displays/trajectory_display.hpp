// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rviz_common/message_filter_display.hpp"
#include "rviz_default_plugins/visibility_control.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "trajectory_planning_msgs/msg/trajectory.hpp"
#include "trajectory_planning_msgs_utils/trajectory_access.hpp"

namespace Ogre {
class ManualObject;
}

namespace rviz_common {
namespace properties {
class ColorProperty;
class FloatProperty;
}  // namespace properties
}  // namespace rviz_common

namespace trajectory_planning_msgs {
namespace displays {
/**
 * \class TrajectoryDisplay
 * \brief Displays a trajectory_planning_msgs::Trajectory message
 */

class TrajectoryDisplay : public rviz_common::MessageFilterDisplay<trajectory_planning_msgs::msg::Trajectory> {
  Q_OBJECT

 public:
  TrajectoryDisplay();
  ~TrajectoryDisplay() override;

  void onInitialize() override;

  void reset() override;

  void timeoutTimerCallback();

 protected:
  void processMessage(trajectory_planning_msgs::msg::Trajectory::ConstSharedPtr msg) override;

  Ogre::ManualObject *vel_trj_, *time_trj_, *acc_trj_, *s_trj_;
  Ogre::MaterialPtr material_vel_, material_time_, material_acc_, material_s_;
  std::vector<std::shared_ptr<rviz_rendering::Shape>> vel_point_spheres_, time_point_spheres_, acc_point_spheres_,
      s_point_spheres_;
  std::vector<std::shared_ptr<rviz_rendering::Arrow>> yaw_arrows_;

  rviz_common::properties::BoolProperty *viz_vel_, *viz_time_, *viz_acc_, *viz_s_;
  rviz_common::properties::ColorProperty *color_property_vel_, *color_property_time_, *color_property_acc_,
      *color_property_s_;
  rviz_common::properties::BoolProperty *viz_yaw_arrows_;
  rviz_common::properties::ColorProperty *color_property_yaw_;
  rviz_common::properties::BoolProperty *viz_vel_points_, *viz_time_points_, *viz_acc_points_, *viz_s_points_;
  rviz_common::properties::FloatProperty *alpha_property_, *size_property_points_, *scale_property_yaw_arrows_;

  // timeout
  rviz_common::properties::BoolProperty *enable_timeout_property_;
  rviz_common::properties::FloatProperty *timeout_property_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

}  // namespace displays
}  // namespace trajectory_planning_msgs
