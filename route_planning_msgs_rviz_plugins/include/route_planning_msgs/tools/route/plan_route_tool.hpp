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
  void onPoseSet(double x, double y, double theta) override;
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

 private Q_SLOTS:
  void updateActionServer();
  int processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
  int processMouseRightButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
  int processMouseMiddleButtonPressed();
  void planRoute();
  void drawIntermediates(std::vector<geometry_msgs::msg::Point> points);

 private:

  rclcpp_action::Client<route_planning_msgs::action::PlanRoute>::SharedPtr action_client_;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::QoS qos_profile_;

  rviz_common::properties::StringProperty* action_server_property_;
  rviz_common::properties::QosProfileProperty* qos_profile_property_;

  geometry_msgs::msg::PointStamped destination_;
  std::vector<geometry_msgs::msg::PointStamped> intermediates_;
  std::vector<std::shared_ptr<rviz_rendering::Shape>> intermediate_shapes_;

  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
};

}  // namespace tools
}  // namespace route_planning_msgs