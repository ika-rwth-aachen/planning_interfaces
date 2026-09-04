// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#include <QObject>
#include <string>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"
#include "rviz_rendering/objects/shape.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <route_planning_msgs/action/plan_route.hpp>

namespace rviz_common {
class DisplayContext;
namespace properties {
class StringProperty;
class QosProfileProperty;
class BoolProperty;
class FloatProperty;
class VectorProperty;
}  // namespace properties
}  // namespace rviz_common

namespace route_planning_msgs {
namespace tools {
class PlanRouteTool : public rviz_default_plugins::tools::PoseTool {
  Q_OBJECT

 public:
  PlanRouteTool();

  ~PlanRouteTool() override;

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

 protected:
  void onPoseSet(double x, double y, double theta) override; // needs to be overridden because of base class
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

 private Q_SLOTS:
  void updateActionServer();
  int processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
  int processMouseRightButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
  int processMouseMiddleButtonPressed();
  void planRoute();
  void drawIntermediates(std::vector<geometry_msgs::msg::PointStamped>& points);

 private:

  rclcpp_action::Client<route_planning_msgs::action::PlanRoute>::SharedPtr action_client_;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::QoS qos_profile_;

  rviz_common::properties::StringProperty* action_server_property_;
  rviz_common::properties::QosProfileProperty* qos_profile_property_;

  geometry_msgs::msg::PointStamped destination_;
  std::vector<geometry_msgs::msg::PointStamped> intermediate_destinations_;
  std::vector<std::shared_ptr<rviz_rendering::Shape>> intermediate_shapes_;

  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
};

}  // namespace tools
}  // namespace route_planning_msgs