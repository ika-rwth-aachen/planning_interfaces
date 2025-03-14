#include "route_planning_msgs/displays/route/new_route_display.hpp"

#include <rviz_common/logging.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_common/properties/parse_color.hpp>

namespace route_planning_msgs {
namespace displays {

void NewRouteDisplay::onInitialize() {
  MFDClass::onInitialize();

  viz_suggested_lane_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Suggested Lane", true, "Whether to display the reference and lane boundary points of the suggested lane.", this, SLOT(updateStyle()));
  color_property_suggested_lane_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(36, 64, 142), "Color to draw reference and lane boundary points of the suggested lane.", viz_suggested_lane_.get(), SLOT(updateStyle()));
  scale_property_suggested_lane_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.1, "Scale of the reference and lane boundary points of the suggested lane.", viz_suggested_lane_.get(), SLOT(updateStyle()));
  
  updateStyle();
}

bool validateFloats(const route_planning_msgs::msg::RouteElement& msg) {
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg.left_boundary);
  valid = valid && rviz_common::validateFloats(msg.right_boundary);
  valid = valid && rviz_common::validateFloats(msg.s);
  for (size_t i = 0; i < msg.lane_elements.size(); ++i) {
    valid = valid && rviz_common::validateFloats(msg.lane_elements[i].reference_pose);
    valid = valid && rviz_common::validateFloats(msg.lane_elements[i].left_boundary.point);
    valid = valid && rviz_common::validateFloats(msg.lane_elements[i].right_boundary.point);
    for (size_t j = 0; j < msg.lane_elements[i].regulatory_elements.size(); ++j) {
      valid = valid && rviz_common::validateFloats(msg.lane_elements[i].regulatory_elements[j].effect_line);
      valid = valid && rviz_common::validateFloats(msg.lane_elements[i].regulatory_elements[j].sign_positions);
    }
  }
  return valid;
}

bool validateFloats(const route_planning_msgs::msg::Route::ConstSharedPtr msg) {
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg->destination);
  for (size_t i = 0; i < msg->traveled_route_elements.size(); ++i) {
    valid = valid && validateFloats(msg->traveled_route_elements[i]);
  }
  for (size_t i = 0; i < msg->remaining_route_elements.size(); ++i) {
    valid = valid && validateFloats(msg->remaining_route_elements[i]);
  }
  return valid;
}

void NewRouteDisplay::processMessage(const route_planning_msgs::msg::Route::ConstSharedPtr msg) {
  // validate floats in message
  if (!validateFloats(msg)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
              "Message contained invalid floating point values (nans or infs)");
    return;
  }

  // transform scene node to message frame
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Error transforming from frame '" << msg->header.frame_id <<
        "' to frame '" << qPrintable(fixed_frame_) << "'");
  }
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  // display suggested lane points
  suggested_lane_points_.clear();
  if (viz_suggested_lane_->getBool()) {
    Ogre::ColourValue color_suggested_lane = rviz_common::properties::qtToOgre(color_property_suggested_lane_->getColor());
    float scale_suggested_lane = scale_property_suggested_lane_->getFloat();
    displaySuggestedLanePoints(msg->remaining_route_elements, color_suggested_lane, scale_suggested_lane);
  }
}

void NewRouteDisplay::displaySuggestedLanePoints(const std::vector<route_planning_msgs::msg::RouteElement>& route_elements, const Ogre::ColourValue& color, float scale) {
  for (const auto& route_element : route_elements) {
    route_planning_msgs::msg::LaneElement suggested_lane = route_planning_msgs::route_access::getCurrentLaneElement(route_element);
    std::shared_ptr<rviz_rendering::Shape> cube = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cube, scene_manager_, scene_node_);
    Ogre::Vector3 reference_point(suggested_lane.reference_pose.position.x, suggested_lane.reference_pose.position.y, suggested_lane.reference_pose.position.z);
    cube->setPosition(reference_point);
    cube->setColor(color);
    cube->setScale(Ogre::Vector3(scale, scale, scale));
    suggested_lane_points_.push_back(cube);
  }
}

void NewRouteDisplay::updateStyle() {
  // suggested lane points
  if (viz_suggested_lane_->getBool()) {
    Ogre::ColourValue color_suggested_lane = rviz_common::properties::qtToOgre(color_property_suggested_lane_->getColor());
    float scale_suggested_lane = scale_property_suggested_lane_->getFloat();
    for (auto& cube : suggested_lane_points_) {
      cube->setColor(color_suggested_lane);
      cube->setScale(Ogre::Vector3(scale_suggested_lane, scale_suggested_lane, scale_suggested_lane));
    }
  }
}

}  // namespace displays
}  // namespace route_planning_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(route_planning_msgs::displays::NewRouteDisplay, rviz_common::Display)