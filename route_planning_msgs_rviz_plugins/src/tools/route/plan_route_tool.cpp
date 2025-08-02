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

#include "route_planning_msgs/tools/route/plan_route_tool.hpp"

#include <OgreSceneNode.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/line.hpp"
#include "rviz_rendering/render_window.hpp"

namespace route_planning_msgs {
namespace tools {

PlanRouteTool::PlanRouteTool() : rviz_default_plugins::tools::PoseTool(), qos_profile_(5) {
  shortcut_key_ = 'p';

  action_server_property_ = new rviz_common::properties::StringProperty("Action server", "/lanelet2_route_planning/plan_route",
                                                                        "The action server to call for planning a route.",
                                                                        getPropertyContainer(), SLOT(updateActionServer()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(action_server_property_, qos_profile_);
}

PlanRouteTool::~PlanRouteTool() = default;

void PlanRouteTool::onInitialize() {
  arrow_ = std::make_shared<rviz_rendering::Arrow>(scene_manager_, nullptr, 0.5f, 0.1f, 0.2f,
                                                   0.35f);  // does not matter which shape, but it must be initialized
  arrow_->getSceneNode()->setVisible(false);
  qos_profile_property_->initialize([this](rclcpp::QoS profile) { this->qos_profile_ = profile; });
  updateActionServer();
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_, tf2::Duration(std::chrono::seconds(60)));
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
}

void PlanRouteTool::activate() { PoseTool::activate(); }

void PlanRouteTool::deactivate() { PoseTool::deactivate(); }

void PlanRouteTool::updateActionServer() {
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  action_client_ = rclcpp_action::create_client<route_planning_msgs::action::PlanRoute>(raw_node, action_server_property_->getStdString());
  clock_ = raw_node->get_clock();
}

void PlanRouteTool::onPoseSet(double x, double y, double theta) {
  (void)theta; // theta is not used in this tool, do not warn about unused variable
  geometry_msgs::msg::PointStamped point;
  point.header.stamp = clock_->now();
  point.header.frame_id = context_->getFixedFrame().toStdString();
  point.point.x = x;
  point.point.y = y;
  point.point.z = 0.0;

  destination_ = point;
}

int PlanRouteTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
  auto point_projection_on_xy_plane =
      projection_finder_->getViewportPointProjectionOnXYPlane(event.panel->getRenderWindow(), event.x, event.y);

  if (event.leftDown()) {
    return processMouseLeftButtonPressed(point_projection_on_xy_plane); // add destination and plan route
  } else if (event.rightDown()) {
    return processMouseRightButtonPressed(point_projection_on_xy_plane); // add intermediate point
  } else if (event.middleDown()) {
    return processMouseMiddleButtonPressed(); // remove last intermediate point
  }

  return 0;
}

int PlanRouteTool::processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection) {
  int flags = 0;

  if (xy_plane_intersection.first) {
    double position_x = xy_plane_intersection.second.x;
    double position_y = xy_plane_intersection.second.y;
    destination_.header.stamp = clock_->now();
    destination_.header.frame_id = context_->getFixedFrame().toStdString();
    destination_.point.x = position_x;
    destination_.point.y = position_y;
    // onPoseSet(position_x, position_y, 0.0);

    planRoute();

    flags |= (Finished | Render);
  }

  return flags;
}

int PlanRouteTool::processMouseRightButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection) {
  int flags = 0;

  if (xy_plane_intersection.first) {
    double position_x = xy_plane_intersection.second.x;
    double position_y = xy_plane_intersection.second.y;
    geometry_msgs::msg::PointStamped point;
    point.header.stamp = clock_->now();
    point.header.frame_id = context_->getFixedFrame().toStdString();
    point.point.x = position_x;
    point.point.y = position_y;

    intermediates_.push_back(point);

    flags |= Render;
  }

  return flags;
}

int PlanRouteTool::processMouseMiddleButtonPressed() {
  int flags = 0;

  if (intermediates_.size() > 0) {
    intermediates_.pop_back();
    drawIntermediates(intermediates_);
    flags |= Render;
  }

  return flags;
}

void PlanRouteTool::drawIntermediates(std::vector<geometry_msgs::msg::PointStamped>& points) {
  intermediate_shapes_.clear();

  for (const auto& point : points) {
    std::shared_ptr<rviz_rendering::Shape> sphere = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, nullptr);
    Ogre::Vector3 pos(point.point.x, point.point.y, point.point.z);
    sphere->setPosition(pos);
    sphere->setColor(0.0f, 1.0f, 0.0f, 1.0f); // Set sphere color to green
    sphere->setScale(Ogre::Vector3(1.0, 1.0, 1.0));
    intermediate_shapes_.push_back(sphere);
  }
}

void PlanRouteTool::planRoute() {
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(rclcpp::get_logger("PlanRouteTool"), "Action server not available, cannot plan route.");
    return;
  }

  route_planning_msgs::action::PlanRoute::Goal goal;
  goal.destination = destination_;
  goal.intermediates = intermediates_;

  auto send_goal_options = rclcpp_action::Client<route_planning_msgs::action::PlanRoute>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<route_planning_msgs::action::PlanRoute>::SharedPtr goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(rclcpp::get_logger("PlanRouteTool"), "Goal was rejected by the action server.");
        } else {
          RCLCPP_INFO(rclcpp::get_logger("PlanRouteTool"), "Goal accepted by the action server, waiting for result.");
        }
      };

  send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<route_planning_msgs::action::PlanRoute>::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          if (result.result->destination_reached) {
            RCLCPP_INFO(rclcpp::get_logger("PlanRouteTool"), "Goal succeeded: destination reached after %.2fm and %ds", result.result->distance_traveled,
                        result.result->time_traveled.sec);
          } else {
            RCLCPP_WARN(rclcpp::get_logger("PlanRouteTool"), "Goal succeeded, but destination not reached after %.2fm and %ds",
                        result.result->distance_traveled, result.result->time_traveled.sec);
          }
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          RCLCPP_WARN(rclcpp::get_logger("PlanRouteTool"), "Goal canceled: traveled %.2fm and %ds", result.result->distance_traveled, result.result->time_traveled.sec);
        } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
          RCLCPP_ERROR(rclcpp::get_logger("PlanRouteTool"), "Goal aborted: traveled %.2fm and %ds", result.result->distance_traveled, result.result->time_traveled.sec);
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("PlanRouteTool"), "Goal finished with unknown result code: %d", static_cast<int>(result.code));
        }
      };

  action_client_->async_send_goal(goal, send_goal_options);
}


}  // namespace tools
}  // namespace route_planning_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(route_planning_msgs::tools::PlanRouteTool, rviz_common::Tool)