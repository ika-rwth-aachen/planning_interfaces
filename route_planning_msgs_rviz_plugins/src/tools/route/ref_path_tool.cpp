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

#include "route_planning_msgs/tools/route/ref_path_tool.hpp"

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

ReferencePathTool::ReferencePathTool() : rviz_default_plugins::tools::PoseTool(), qos_profile_(5) {
  shortcut_key_ = 't';

  topic_property_ = new rviz_common::properties::StringProperty("Topic", "lanelet2_route_planning/route",
                                                                "The topic on which to publish the route/path.",
                                                                getPropertyContainer(), SLOT(updateTopic()), this);

  init_point_property_ = new rviz_common::properties::BoolProperty(
      "Set (0, 0, 0) as initial point", true, "If checked, the first point will be set to (0, 0, 0).",
      getPropertyContainer(), SLOT(updateInitPoint()), this);

  frame_property_ = new rviz_common::properties::StringProperty(
      "Frame for route", "base_link", "The frame to use for the defined initial point of path.", init_point_property_,
      SLOT(updateFrame()), this);

  geo_fence_property_ =
      new rviz_common::properties::BoolProperty("Geo-fence", false,
                                                "If checked, the route will be limited to a defined geo-fence defined "
                                                "by a rectangle (adding points outside the geo-fence is not possible).",
                                                getPropertyContainer(), SLOT(updateGeoFence()), this);

  geo_fence_visible_property_ =
      new rviz_common::properties::BoolProperty("Show geo-fence", false, "If checked, the geo-fence will be shown.",
                                                geo_fence_property_, SLOT(updateShowGeoFence()), this);

  first_point_property_ = new rviz_common::properties::VectorProperty(
      "First point", Ogre::Vector3(0.0, 0.0, 0.0), "The coordinates of the first point of the geo-fence.",
      geo_fence_property_, SLOT(updateFirstPoint()), this);

  second_point_property_ = new rviz_common::properties::VectorProperty(
      "Second point", Ogre::Vector3(0.0, 0.0, 0.0), "The coordinates of the second point of the geo-fence.",
      geo_fence_property_, SLOT(updateSecondPoint()), this);

  third_point_property_ = new rviz_common::properties::VectorProperty(
      "Third point", Ogre::Vector3(0.0, 0.0, 0.0), "The coordinates of the third point of the geo-fence.",
      geo_fence_property_, SLOT(updateThirdPoint()), this);

  fourth_point_property_ = new rviz_common::properties::VectorProperty(
      "Fourth point", Ogre::Vector3(0.0, 0.0, 0.0), "The coordinates of the fourth point of the geo-fence.",
      geo_fence_property_, SLOT(updateFourthPoint()), this);

  sampling_property_ = new rviz_common::properties::BoolProperty(
      "Use additional equidistant sampling of path", true,
      "If checked, an equidistant sampling of the path will be performed before publishing.", getPropertyContainer(),
      SLOT(updateSampling()), this);

  sampling_distance_property_ = new rviz_common::properties::FloatProperty(
      "Minimum sampling distance [m]", 0.5,
      "The distance between two consecutive points of the equidistant sampling in meter.", sampling_property_,
      SLOT(updateSamplingDistance()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(topic_property_, qos_profile_);
}

ReferencePathTool::~ReferencePathTool() = default;

void ReferencePathTool::onInitialize() {
  arrow_ = std::make_shared<rviz_rendering::Arrow>(scene_manager_, nullptr, 0.5f, 0.1f, 0.2f,
                                                   0.35f);  // does not matter which shape, but it must be initialized
  arrow_->getSceneNode()->setVisible(false);
  qos_profile_property_->initialize([this](rclcpp::QoS profile) { this->qos_profile_ = profile; });
  updateTopic();
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_, tf2::Duration(std::chrono::seconds(60)));
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
}

void ReferencePathTool::activate() {
  PoseTool::activate();
  initRoute();
}

void ReferencePathTool::deactivate() { PoseTool::deactivate(); }

void ReferencePathTool::updateTopic() {
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  route_publisher_ = raw_node->template create_publisher<route_planning_msgs::msg::Route>(
      topic_property_->getStdString(), qos_profile_);
  clock_ = raw_node->get_clock();
}

void ReferencePathTool::updateInitPoint() { ref_path_points_.clear(); }

bool ReferencePathTool::setInitPoint() {
  if (init_point_property_->getBool() && ref_path_points_.empty()) {
    geometry_msgs::msg::PointStamped point;
    point.point.x = 0.0;
    point.point.y = 0.0;
    point.point.z = 0.0;
    point.header.frame_id = frame_property_->getStdString();
    point.header.stamp = route_.header.stamp;

    //check if point frame is different from route frame
    geometry_msgs::msg::PointStamped point_tf;
    if (point.header.frame_id != route_.header.frame_id) {
      // transform the point to the frame of the route
      try {
        point_tf = tf2_buffer_->transform(point, route_.header.frame_id, tf2::durationFromSec(1.0));
      } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("ReferencePathTool"), "Transform error: %s", ex.what());
        return false;
      }
    } else {
      point_tf = point;
    }

    ref_path_points_.push_back(point_tf);
    RCLCPP_INFO(rclcpp::get_logger("ReferencePathTool"), "Initial point set to (%f, %f, %f) in frame '%s'",
                point_tf.point.x, point_tf.point.y, point_tf.point.z, point_tf.header.frame_id.c_str());
    return true;
  } else { // no initial point required or already set
    return false;
  }
}

void ReferencePathTool::updateFrame() {}

void ReferencePathTool::updateGeoFence() {
  if (geo_fence_property_->getBool()) {

    points_quadrilateral_.clear();
    geometry_msgs::msg::Point point1;
    point1.x = first_point_property_->getVector().x;
    point1.y = first_point_property_->getVector().y;

    geometry_msgs::msg::Point point2;
    point2.x = second_point_property_->getVector().x;
    point2.y = second_point_property_->getVector().y;

    geometry_msgs::msg::Point point3;
    point3.x = third_point_property_->getVector().x;
    point3.y = third_point_property_->getVector().y;

    geometry_msgs::msg::Point point4;
    point4.x = fourth_point_property_->getVector().x;
    point4.y = fourth_point_property_->getVector().y;

    points_quadrilateral_.push_back(point1);
    points_quadrilateral_.push_back(point2);
    points_quadrilateral_.push_back(point3);
    points_quadrilateral_.push_back(point4);
    points_quadrilateral_.push_back(point1);
  }
  updateShowGeoFence();
}

void ReferencePathTool::updateShowGeoFence() {
  if (geo_fence_visible_property_->getBool()) {
    drawGeoFence(points_quadrilateral_);
  } else {
    lines_geo_fence_.clear();
  }
}

void ReferencePathTool::updateFirstPoint() {
  if (geo_fence_property_->getBool()) {
    updateGeoFence();
  }
}

void ReferencePathTool::updateSecondPoint() {
  if (geo_fence_property_->getBool()) {
    updateGeoFence();
  }
}

void ReferencePathTool::updateThirdPoint() {
  if (geo_fence_property_->getBool()) {
    updateGeoFence();
  }
}

void ReferencePathTool::updateFourthPoint() {
  if (geo_fence_property_->getBool()) {
    updateGeoFence();
  }
}

void ReferencePathTool::updateSampling() {}

void ReferencePathTool::updateSamplingDistance() {}

double ReferencePathTool::calculateArea(double x1, double y1, double x2, double y2, double x3, double y3) {
  return 0.5 * std::abs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
}

bool ReferencePathTool::isPointInTriangle(double px, double py, double x1, double y1, double x2, double y2, double x3,
                                          double y3) {
  double totalArea = calculateArea(x1, y1, x2, y2, x3, y3);

  // Compute areas of sub-triangles (PAB, PAC, PBC)
  double area1 = calculateArea(px, py, x2, y2, x3, y3);
  double area2 = calculateArea(x1, y1, px, py, x3, y3);
  double area3 = calculateArea(x1, y1, x2, y2, px, py);

  // Check if the sum of the sub-triangle areas is equal to the total area
  return (totalArea == area1 + area2 + area3);
}

bool ReferencePathTool::checkIfPointIsInsideGeoFence(double px, double py) {
  double x1 = first_point_property_->getVector().x;
  double y1 = first_point_property_->getVector().y;
  double x2 = second_point_property_->getVector().x;
  double y2 = second_point_property_->getVector().y;
  double x3 = third_point_property_->getVector().x;
  double y3 = third_point_property_->getVector().y;
  double x4 = fourth_point_property_->getVector().x;
  double y4 = fourth_point_property_->getVector().y;

  bool inFirstTriangle = isPointInTriangle(px, py, x1, y1, x2, y2, x3, y3);
  bool inSecondTriangle = isPointInTriangle(px, py, x1, y1, x3, y3, x4, y4);

  if (inFirstTriangle || inSecondTriangle) {
    // point is inside the geo-fence
    return true;
  } else {
    // point is outside the geo-fence
    return false;
  }
}

void ReferencePathTool::onPoseSet(double x, double y, double theta) {
  (void)theta; // theta is not used in this tool, do not warn about unused variable
  geometry_msgs::msg::PointStamped point;
  point.header.stamp = clock_->now();
  point.header.frame_id = context_->getFixedFrame().toStdString();
  point.point.x = x;
  point.point.y = y;
  point.point.z = 0.0;

  if (geo_fence_property_->getBool()) {
    if (checkIfPointIsInsideGeoFence(x, y) &&
        checkIfPointIsInsideGeoFence(ref_path_points_.back().point.x, ref_path_points_.back().point.y)) {
      ref_path_points_.push_back(point);

      drawLinesBetweenPoints(ref_path_points_);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("ReferencePathTool"), "Point is outside the geo fence!");
    }
  } else {
    ref_path_points_.push_back(point);

    drawLinesBetweenPoints(ref_path_points_);
  }
}

int ReferencePathTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
  auto point_projection_on_xy_plane =
      projection_finder_->getViewportPointProjectionOnXYPlane(event.panel->getRenderWindow(), event.x, event.y);

  if (event.leftDown()) {
    return processMouseLeftButtonPressed();
  } else if (event.rightDown()) {
    return processMouseRightButtonPressed(point_projection_on_xy_plane);
  } else if (event.middleDown()) {
    return processMouseMiddleButtonPressed();
  }

  return 0;
}

int ReferencePathTool::processMouseRightButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection) {
  int flags = 0;
  if (init_point_property_->getBool() && ref_path_points_.empty()) {
    if (setInitPoint()) {
      if (xy_plane_intersection.first) {
        double position_x = xy_plane_intersection.second.x;
        double position_y = xy_plane_intersection.second.y;
        onPoseSet(position_x, position_y, 0.0);

        flags |= Render;
      }
    }
  } else {
    if (xy_plane_intersection.first) {
      double position_x = xy_plane_intersection.second.x;
      double position_y = xy_plane_intersection.second.y;
      onPoseSet(position_x, position_y, 0.0);

      flags |= Render;
    }
  }
  return flags;
}

int ReferencePathTool::processMouseMiddleButtonPressed() {
  int flags = 0;
  if ((init_point_property_->getBool() && ref_path_points_.size() > 1) ||
      (!init_point_property_->getBool() && !ref_path_points_.empty())) {
    ref_path_points_.pop_back();

    flags |= Render;

    drawLinesBetweenPoints(ref_path_points_);
  }

  return flags;
}

int ReferencePathTool::processMouseLeftButtonPressed() {
  int flags = 0;
  std::vector<geometry_msgs::msg::PointStamped> route_points;

  if(ref_path_points_.size()<2) {
    RCLCPP_ERROR(rclcpp::get_logger("ReferencePathTool"), "No points defined, publishing empty route!");
    route_planning_msgs::msg::Route empty_route;
    empty_route.header = route_.header;
    route_publisher_->publish(empty_route);
    return flags;
  }

  if (sampling_property_->getBool()) {
    double sampling_distance = sampling_distance_property_->getFloat();
    // add additional points in between two points of the reference path with a distance of sampling_distance
    for (size_t i = 1; i < ref_path_points_.size(); ++i) {
      route_points.push_back(ref_path_points_[i - 1]);
      auto& p1 = ref_path_points_[i].point;
      auto& p2 = ref_path_points_[i - 1].point;
      double dx = p1.x - p2.x;
      double dy = p1.y - p2.y;
      double distance = std::sqrt(dx * dx + dy * dy);
      int num_points = std::ceil(distance / sampling_distance);
      double step = 1.0 / num_points;
      for (int j = 1; j < num_points; ++j) {
        geometry_msgs::msg::PointStamped point;
        double s = std::sqrt(j * j * step * step * dx * dx + j * j * step * step * dy * dy);
        point.point.x = p2.x + j * step * dx;
        point.point.y = p2.y + j * step * dy;
        point.header = ref_path_points_[i].header;
        if (s < distance) {
          route_points.push_back(point);
        } else {
          route_points.push_back(ref_path_points_[i]);
          break;
        }
      }
    }
  } else {
    route_points = ref_path_points_;
  }

  if (fillRoute(route_points)) {
    route_publisher_->publish(route_);
    RCLCPP_INFO(rclcpp::get_logger("ReferencePathTool"), "Route published!");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ReferencePathTool"), "Route could not be filled!");
  }

  ref_path_points_.clear();

  drawLinesBetweenPoints(ref_path_points_);

  flags |= (Finished | Render);

  return flags;
}

void ReferencePathTool::drawLinesBetweenPoints(std::vector<geometry_msgs::msg::PointStamped> points) {
  lines_.clear();

  if (!points.empty()) {
    if (points[0].header.frame_id != context_->getFixedFrame().toStdString()) {
      geometry_msgs::msg::PointStamped point_tf;
      try {
        point_tf =
            tf2_buffer_->transform(points[0], context_->getFixedFrame().toStdString(), tf2::durationFromSec(0.0));
        points[0] = point_tf;
      } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("ReferencePathTool"), "Transform error: %s", ex.what());
      }
    }
  }

  for (size_t i = 1; i < points.size(); ++i) {
    if (points[i].header.frame_id != context_->getFixedFrame().toStdString()) {
      geometry_msgs::msg::PointStamped point_tf;
      try {
        point_tf =
            tf2_buffer_->transform(points[i], context_->getFixedFrame().toStdString(), tf2::durationFromSec(0.0));
        points[i] = point_tf;
      } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("ReferencePathTool"), "Transform error: %s", ex.what());
      }
    }
    auto line = std::make_shared<rviz_rendering::Line>(scene_manager_, nullptr);
    Ogre::Vector3 point1(points[i - 1].point.x, points[i - 1].point.y, points[i - 1].point.z);
    Ogre::Vector3 point2(points[i].point.x, points[i].point.y, points[i].point.z);
    line->setPoints(point1, point2);
    line->setColor(1.0f, 0.0f, 0.0f, 1.0f);  // Set line color to red
    lines_.push_back(line);
  }
}

void ReferencePathTool::drawGeoFence(std::vector<geometry_msgs::msg::Point> points) {
  lines_geo_fence_.clear();

  for (size_t i = 1; i < points.size(); ++i) {
    auto line = std::make_shared<rviz_rendering::Line>(scene_manager_, nullptr);
    Ogre::Vector3 point1(points[i - 1].x, points[i - 1].y, points[i - 1].z);
    Ogre::Vector3 point2(points[i].x, points[i].y, points[i].z);
    line->setPoints(point1, point2);
    line->setColor(0.0f, 1.0f, 0.0f, 1.0f);  // Set line color to green
    lines_geo_fence_.push_back(line);
  }
}

void ReferencePathTool::initRoute() {
  route_.header.stamp = clock_->now();
  route_.header.frame_id = context_->getFixedFrame().toStdString();
  route_.route_elements.clear();
}

bool ReferencePathTool::fillRoute(std::vector<geometry_msgs::msg::PointStamped> ref_path_points) {
  double s = 0.0;
  for (size_t i = 0; i < ref_path_points.size(); ++i) {
    //check if point frame is different from route frame
    geometry_msgs::msg::PointStamped point_tf;
    if (ref_path_points[i].header.frame_id != route_.header.frame_id) {
      // transform the point to the frame of the route
      try {
        point_tf = tf2_buffer_->transform(ref_path_points[i], route_.header.frame_id, tf2::durationFromSec(0.0));
      } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("ReferencePathTool"), "Transform error: %s", ex.what());
        return false;
      }
    } else {
      point_tf = ref_path_points[i];
    }

    // get distance and orientation
    double theta = 0.0;
    if (i > 0) {
      double dx = ref_path_points[i].point.x - ref_path_points[i - 1].point.x;
      double dy = ref_path_points[i].point.y - ref_path_points[i - 1].point.y;
      s += std::sqrt(dx * dx + dy * dy);
      theta = std::atan2(dy, dx);
    }

    // update orientation to the next point if not the last point
    if (i < ref_path_points.size() - 1) {
      double dx = ref_path_points[i + 1].point.x - ref_path_points[i].point.x;
      double dy = ref_path_points[i + 1].point.y - ref_path_points[i].point.y;
      theta = std::atan2(dy, dx);
    }

    route_planning_msgs::msg::RouteElement route_element;
    route_planning_msgs::msg::LaneElement lane_element;

    lane_element.reference_pose.position = point_tf.point;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    lane_element.reference_pose.orientation = tf2::toMsg(q);    
    if (i != ref_path_points.size() - 1) {
      lane_element.has_following_lane_idx = true;
      lane_element.following_lane_idx = 0;
    } else {
      lane_element.has_following_lane_idx = false;
    }
    route_element.lane_elements.push_back(lane_element);

    route_element.suggested_lane_idx = 0;
    route_element.s = s;
    route_element.is_enriched = false;
    route_.route_elements.push_back(route_element);
  }
  route_.destination = route_.route_elements.back().lane_elements[0].reference_pose.position;
  route_.starting_route_element_idx = 0;
  route_.current_route_element_idx = 0;
  route_.destination_route_element_idx = route_.route_elements.size() - 1;
  return true;
}

}  // namespace tools
}  // namespace route_planning_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(route_planning_msgs::tools::ReferencePathTool, rviz_common::Tool)