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

  viz_other_lanes_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Other Lanes", true, "Whether to display the reference and lane boundary points of other lanes.", this, SLOT(updateStyle()));
  color_property_other_lanes_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 0, 0), "Color to draw reference and lane boundary points of other lanes.", viz_other_lanes_.get(), SLOT(updateStyle()));
  scale_property_other_lanes_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.1, "Scale of the reference and lane boundary points of other lanes.", viz_other_lanes_.get(), SLOT(updateStyle()));

  viz_driveable_space_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Driveable Space", true, "Whether to display the reference and lane boundary points of the driveable space.", this, SLOT(updateStyle()));
  color_property_driveable_space_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(0, 255, 0), "Color to draw reference and lane boundary points of the driveable space.", viz_driveable_space_.get(), SLOT(updateStyle()));
  scale_property_driveable_space_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.1, "Scale of the reference and lane boundary points of the driveable space.", viz_driveable_space_.get(), SLOT(updateStyle()));  
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

  // clear previous points
  suggested_lane_points_.clear();
  other_lane_points_.clear();
  drivable_space_points_.clear();

  // Get visualization settings once
  bool show_suggested_lane = viz_suggested_lane_->getBool();
  bool show_other_lanes = viz_other_lanes_->getBool();
  bool show_drivable_space = viz_driveable_space_->getBool();

  Ogre::ColourValue color_suggested_lane = rviz_common::properties::qtToOgre(color_property_suggested_lane_->getColor());
  Ogre::ColourValue color_other_lanes = rviz_common::properties::qtToOgre(color_property_other_lanes_->getColor());
  Ogre::ColourValue color_driveable_space = rviz_common::properties::qtToOgre(color_property_driveable_space_->getColor());

  float scale_suggested_lane = scale_property_suggested_lane_->getFloat();
  float scale_other_lanes = scale_property_other_lanes_->getFloat();
  float scale_driveable_space = scale_property_driveable_space_->getFloat();

  // loop over route elements
  for (const auto& route_element : msg->remaining_route_elements) {
    // display suggested lane points
    if (show_suggested_lane) {
      const auto& suggested_lane = route_planning_msgs::route_access::getCurrentLaneElement(route_element);
      suggested_lane_points_.push_back(generateRenderPoint(suggested_lane.reference_pose.position, color_suggested_lane, scale_suggested_lane));
    }

    // display other lane points
    if (show_other_lanes) {
      for (size_t i = 0; i < route_element.lane_elements.size(); ++i) {
        if (i != route_element.suggested_lane_idx) {
          const auto& other_lane = route_element.lane_elements[i];
          other_lane_points_.push_back(generateRenderPoint(other_lane.reference_pose.position, color_other_lanes, scale_other_lanes));
        }
      }
    }

    // display driveable space points
    if (show_drivable_space) {
      drivable_space_points_.push_back(generateRenderPoint(route_element.left_boundary, color_driveable_space, scale_driveable_space));
      drivable_space_points_.push_back(generateRenderPoint(route_element.right_boundary, color_driveable_space, scale_driveable_space));
    }
  }
}

std::shared_ptr<rviz_rendering::Shape> NewRouteDisplay::generateRenderPoint(const geometry_msgs::msg::Point& point, const Ogre::ColourValue& color, const float scale) {
  std::shared_ptr<rviz_rendering::Shape> cube = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cube, scene_manager_, scene_node_);
  Ogre::Vector3 pos(point.x, point.y, point.z);
  cube->setPosition(pos);
  cube->setColor(color);
  cube->setScale(Ogre::Vector3(scale, scale, scale));
  return cube;
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

  // other lane points
  if (viz_other_lanes_->getBool()) {
    Ogre::ColourValue color_other_lanes = rviz_common::properties::qtToOgre(color_property_other_lanes_->getColor());
    float scale_other_lanes = scale_property_other_lanes_->getFloat();
    for (auto& cube : other_lane_points_) {
      cube->setColor(color_other_lanes);
      cube->setScale(Ogre::Vector3(scale_other_lanes, scale_other_lanes, scale_other_lanes));
    }
  }

  // driveable space points
  if (viz_driveable_space_->getBool()) {
    Ogre::ColourValue color_driveable_space = rviz_common::properties::qtToOgre(color_property_driveable_space_->getColor());
    float scale_driveable_space = scale_property_driveable_space_->getFloat();
    for (auto& cube : drivable_space_points_) {
      cube->setColor(color_driveable_space);
      cube->setScale(Ogre::Vector3(scale_driveable_space, scale_driveable_space, scale_driveable_space));
    }
  }
}

}  // namespace displays
}  // namespace route_planning_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(route_planning_msgs::displays::NewRouteDisplay, rviz_common::Display)