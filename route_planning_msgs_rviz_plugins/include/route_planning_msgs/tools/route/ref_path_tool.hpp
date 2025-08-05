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
#include "rviz_rendering/objects/line.hpp"

#include "route_planning_msgs/msg/route.hpp"

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
class ReferencePathTool : public rviz_default_plugins::tools::PoseTool {
  Q_OBJECT

 public:
  ReferencePathTool();

  ~ReferencePathTool() override;

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

 protected:
  void onPoseSet(double x, double y, double theta) override;
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

 private Q_SLOTS:
  void drawLinesBetweenPoints(std::vector<geometry_msgs::msg::PointStamped> points);
  void updateTopic();
  void updateInitPoint();
  bool setInitPoint();
  void updateFrame();
  int processMouseLeftButtonPressed();
  int processMouseRightButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
  int processMouseMiddleButtonPressed();
  bool fillRoute(std::vector<geometry_msgs::msg::PointStamped> ref_path_points);
  void initRoute();
  void updateGeoFence();
  void updateShowGeoFence();
  void updateFirstPoint();
  void updateSecondPoint();
  void updateThirdPoint();
  void updateFourthPoint();
  void updateSampling();
  void updateSamplingDistance();
  double calculateArea(double x1, double y1, double x2, double y2, double x3, double y3);
  bool isPointInTriangle(double px, double py, double x1, double y1, double x2, double y2, double x3, double y3);
  bool checkIfPointIsInsideGeoFence(double px, double py);
  void drawGeoFence(std::vector<geometry_msgs::msg::Point> points);

 private:
  rclcpp::Publisher<route_planning_msgs::msg::Route>::SharedPtr route_publisher_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::QoS qos_profile_;

  rviz_common::properties::StringProperty* topic_property_;
  rviz_common::properties::QosProfileProperty* qos_profile_property_;
  rviz_common::properties::BoolProperty* init_point_property_;
  rviz_common::properties::StringProperty* frame_property_;
  rviz_common::properties::BoolProperty* geo_fence_property_;
  rviz_common::properties::BoolProperty* geo_fence_visible_property_;
  rviz_common::properties::VectorProperty* first_point_property_;
  rviz_common::properties::VectorProperty* second_point_property_;
  rviz_common::properties::VectorProperty* third_point_property_;
  rviz_common::properties::VectorProperty* fourth_point_property_;
  rviz_common::properties::BoolProperty* sampling_property_;
  rviz_common::properties::FloatProperty* sampling_distance_property_;

  std::vector<geometry_msgs::msg::PointStamped> ref_path_points_;
  std::vector<geometry_msgs::msg::Point> points_quadrilateral_;
  std::vector<std::shared_ptr<rviz_rendering::Line>> lines_;
  std::vector<std::shared_ptr<rviz_rendering::Line>> lines_geo_fence_;

  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  route_planning_msgs::msg::Route route_;
};

}  // namespace tools
}  // namespace route_planning_msgs