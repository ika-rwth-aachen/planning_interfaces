// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#include "route_planning_msgs/displays/reference_line/reference_line_display.hpp"

#include <rviz_common/properties/parse_color.hpp>

namespace route_planning_msgs::displays {

void ReferenceLineDisplay::onInitialize() {
  qos_profile = rclcpp::QoS(1).reliable().transient_local();
  MFDClass::onInitialize();

  color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 170, 0), "Color to draw the global reference line.",
      this);
  line_width_property_ =
      std::make_unique<rviz_common::properties::FloatProperty>(
          "Line Width", 0.15, "Width of the global reference line.", this);
  line_ = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_,
                                                          scene_node_);
}

void ReferenceLineDisplay::processMessage(
    const route_planning_msgs::msg::ReferenceLine::ConstSharedPtr msg) {
  line_->clear();
  line_->setMaxPointsPerLine(static_cast<int>(msg->delta_points.size() + 1));
  line_->setNumLines(1);
  line_->addPoint(Ogre::Vector3(msg->origin.x, msg->origin.y, 0.0));
  for (const auto &delta_point : msg->delta_points) {
    line_->addPoint(Ogre::Vector3(msg->origin.x + delta_point.x,
                                  msg->origin.y + delta_point.y, 0.0));
  }
  const auto color =
      rviz_common::properties::qtToOgre(color_property_->getColor());
  line_->setColor(color.r, color.g, color.b, color.a);
  line_->setLineWidth(line_width_property_->getFloat());
  line_->finishLine();
}

void ReferenceLineDisplay::reset() {
  MFDClass::reset();
  if (line_) {
    line_->clear();
  }
}

} // namespace route_planning_msgs::displays

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(route_planning_msgs::displays::ReferenceLineDisplay,
                       rviz_common::Display)
