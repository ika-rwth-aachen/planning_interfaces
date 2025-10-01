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

#include "route_planning_msgs/displays/route/route_display.hpp"

#include <rviz_common/logging.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <algorithm>

namespace route_planning_msgs {
namespace displays {

RouteDisplay::~RouteDisplay() {
  if (timeout_timer_) {
    timeout_timer_->cancel();
  }
  timeout_timer_.reset();
}

void RouteDisplay::onInitialize() {
  MFDClass::onInitialize();

  // destination
  viz_destination_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Destination", true, "Whether to display the destination arrow.", this);
  color_property_destination_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color (final)", QColor(255, 0, 255), "Color to draw the destination arrow.", viz_destination_.get());
  scale_property_destination_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale (final)", 1.0, "Scale of the destination arrow.", viz_destination_.get());
  color_property_intermediate_destinations_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color (intermediate)", QColor(150, 0, 255), "Color to draw the intermediate destinations.", viz_destination_.get());
  scale_property_intermediate_destinations_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale (intermediate)", 1.0, "Scale of the intermediate destinations.", viz_destination_.get());

  // traveled route
  viz_traveled_route_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Traveled Route", true, "Whether to display the traveled route.", this);
  opacity_property_traveled_route_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Opacity", 0.5, "Opacity of the traveled route.", viz_traveled_route_.get());

  // suggested lane
  viz_suggested_lane_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Suggested Lane", true, "Whether to display the suggested lane.", this);
  viz_suggested_lane_reference_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Reference Line", true, "Whether to display the reference line of the suggested lane.", viz_suggested_lane_.get());
  viz_suggested_lane_boundaries_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Boundaries", true, "Whether to display the reference boundaries of the suggested lane.", viz_suggested_lane_.get());

  viz_suggested_lane_reference_poses_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Poses", false, "Whether to display the reference poses of the suggested lane.", viz_suggested_lane_reference_.get());
  color_property_suggested_lane_reference_poses_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(0, 255, 0), "Color to draw reference poses of the suggested lane.", viz_suggested_lane_reference_poses_.get());
  scale_property_suggested_lane_reference_poses_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.5, "Scale of the reference poses of the suggested lane.", viz_suggested_lane_reference_poses_.get());

  viz_suggested_lane_reference_line_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Line", true, "Whether to display the reference line of the suggested lane.", viz_suggested_lane_reference_.get());
  color_property_suggested_lane_reference_line_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(0, 255, 0), "Color to draw reference line of the suggested lane.", viz_suggested_lane_reference_line_.get());
  scale_property_suggested_lane_reference_line_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.1, "Scale of the reference line of the suggested lane.", viz_suggested_lane_reference_line_.get());

  viz_suggested_lane_boundary_points_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Points", false, "Whether to display the reference and lane boundary points of the suggested lane.", viz_suggested_lane_boundaries_.get());
  color_property_suggested_lane_boundary_points_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(0, 170, 0), "Color to draw reference and lane boundary points of the suggested lane.", viz_suggested_lane_boundary_points_.get());
  scale_property_suggested_lane_boundary_points_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.25, "Scale of the reference and lane boundary points of the suggested lane.", viz_suggested_lane_boundary_points_.get());

  viz_suggested_lane_boundary_lines_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Lines", true, "Whether to display the lane boundary lines of the suggested lane.", viz_suggested_lane_boundaries_.get());
  color_property_suggested_lane_boundary_lines_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(0, 170, 0), "Color to draw lane boundary lines of the suggested lane.", viz_suggested_lane_boundary_lines_.get());
  scale_property_suggested_lane_boundary_lines_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.1, "Scale of the lane boundary lines of the suggested lane.", viz_suggested_lane_boundary_lines_.get());

  viz_suggested_lane_regulatory_elements_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Regulatory Elements", true, "Whether to display the regulatory elements of the suggested lane.", viz_suggested_lane_.get());
  color_property_suggested_lane_regulatory_elements_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Default Color", QColor(200, 200, 200), "Default state color of regulatory elements of the suggested lane.", viz_suggested_lane_regulatory_elements_.get());
  scale_property_suggested_lane_regulatory_elements_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.2, "Scale of the regulatory elements of the suggested lane.", viz_suggested_lane_regulatory_elements_.get());
  viz_suggested_lane_regulatory_elements_sign_positions_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Positions", true, "Whether to display the sign positions of the regulatory elements of the suggested lane.", viz_suggested_lane_regulatory_elements_.get());
  viz_suggested_lane_regulatory_elements_timing_information_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Validity stamp", false, "Whether to display the validity stamp of the regulatory elements of the suggested lane.", viz_suggested_lane_regulatory_elements_.get());
  color_property_suggested_lane_regulatory_elements_timing_information_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 255, 255), "Color to draw validity stamp of the regulatory elements of the suggested lane.", viz_suggested_lane_regulatory_elements_timing_information_.get());
  scale_property_suggested_lane_regulatory_elements_timing_information_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.6, "Scale of the validity stamp of the regulatory elements of the suggested lane.", viz_suggested_lane_regulatory_elements_timing_information_.get());

  // lane change
  viz_lane_change_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Lane Change", true, "Whether to display the lane change lines.", viz_suggested_lane_.get());
  color_property_lane_change_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(0, 255, 0), "Color to draw lane change lines.", viz_lane_change_.get());
  scale_property_lane_change_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.2, "Scale of the lane change lines.", viz_lane_change_.get());

  // adjacent lanes
  viz_adjacent_lanes_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Adjacent Lanes", true, "Whether to display the reference and lane boundary points of adjacent lanes.", this);
  viz_adjacent_lanes_reference_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Reference Line", true, "Whether to display the reference line of adjacent lanes.", viz_adjacent_lanes_.get());
  viz_adjacent_lanes_boundaries_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Boundaries", true, "Whether to display the reference boundaries of adjacent lanes.", viz_adjacent_lanes_.get());

  viz_adjacent_lanes_reference_poses_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Poses", false, "Whether to display the reference poses of adjacent lanes.", viz_adjacent_lanes_reference_.get());
  color_property_adjacent_lanes_reference_poses_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 255, 0), "Color to draw reference poses of adjacent lanes.", viz_adjacent_lanes_reference_poses_.get());
  scale_property_adjacent_lanes_reference_poses_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.5, "Scale of the reference poses of adjacent lanes.", viz_adjacent_lanes_reference_poses_.get());

  viz_adjacent_lanes_reference_line_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Line", true, "Whether to display the reference line of adjacent lanes.", viz_adjacent_lanes_reference_.get());
  color_property_adjacent_lanes_reference_line_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 255, 0), "Color to draw reference line of adjacent lanes.", viz_adjacent_lanes_reference_line_.get());
  scale_property_adjacent_lanes_reference_line_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.05, "Scale of the reference line of adjacent lanes.", viz_adjacent_lanes_reference_line_.get());

  viz_adjacent_lanes_boundary_points_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Points", false, "Whether to display the reference and lane boundary points of adjacent lanes.", viz_adjacent_lanes_boundaries_.get());
  color_property_adjacent_lanes_boundary_points_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 170, 0), "Color to draw reference and lane boundary points of adjacent lanes.", viz_adjacent_lanes_boundary_points_.get());
  scale_property_adjacent_lanes_boundary_points_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.2, "Scale of the reference and lane boundary points of adjacent lanes.", viz_adjacent_lanes_boundary_points_.get());

  viz_adjacent_lanes_boundary_lines_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Lines", true, "Whether to display the lane boundary lines of adjacent lanes.", viz_adjacent_lanes_boundaries_.get());
  color_property_adjacent_lanes_boundary_lines_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 170, 0), "Color to draw lane boundary lines of adjacent lanes.", viz_adjacent_lanes_boundary_lines_.get());
  scale_property_adjacent_lanes_boundary_lines_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.05, "Scale of the lane boundary lines of adjacent lanes.", viz_adjacent_lanes_boundary_lines_.get());

  viz_adjacent_lane_regulatory_elements_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Regulatory Elements", true, "Whether to display the regulatory elements of adjacent lanes.", viz_adjacent_lanes_.get());
  color_property_adjacent_lane_regulatory_elements_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Default Color", QColor(200, 200, 200), "Default state color of regulatory elements of adjacent lanes.", viz_adjacent_lane_regulatory_elements_.get());
  scale_property_adjacent_lane_regulatory_elements_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.1, "Scale of the regulatory elements of adjacent lanes.", viz_adjacent_lane_regulatory_elements_.get());
  viz_adjacent_lane_regulatory_elements_sign_positions_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Positions", true, "Whether to display the sign positions of the regulatory elements of adjacent lanes.", viz_adjacent_lane_regulatory_elements_.get());
  viz_adjacent_lane_regulatory_elements_timing_information_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Validity stamp", false, "Whether to display the validity stamp of the regulatory elements of adjacent lanes.", viz_adjacent_lane_regulatory_elements_.get());
  color_property_adjacent_lane_regulatory_elements_timing_information_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 255, 255), "Color to draw validity stamp of the regulatory elements of adjacent lanes.", viz_adjacent_lane_regulatory_elements_timing_information_.get());
  scale_property_adjacent_lane_regulatory_elements_timing_information_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.6, "Scale of the validity stamp of the regulatory elements of adjacent lanes.", viz_adjacent_lane_regulatory_elements_timing_information_.get());

  // drivable space
  viz_drivable_space_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Drivable Space", false, "Whether to display the the drivable space.", this);

  viz_drivable_space_points_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Points", false, "Whether to display the points of the drivable space.", viz_drivable_space_.get());
  color_property_drivable_space_points_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 0, 0), "Color to draw points of the drivable space.", viz_drivable_space_points_.get());
  scale_property_drivable_space_points_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.2, "Scale of the points of the drivable space.", viz_drivable_space_points_.get());

  viz_drivable_space_lines_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Lines", true, "Whether to display the lines of the drivable space.", viz_drivable_space_.get());
  color_property_drivable_space_lines_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 0, 0), "Color to draw lines of the drivable space.", viz_drivable_space_lines_.get());
  scale_property_drivable_space_lines_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.1, "Scale of the lines of the drivable space.", viz_drivable_space_lines_.get());

  // timeout properties
  enable_timeout_property_ = new rviz_common::properties::BoolProperty("Timeout", true, "Remove renderings after timeout if no new msgs have been received", this);
  timeout_property_ = new rviz_common::properties::FloatProperty("Duration", 1.0, "Timeout duration in seconds (wall time)", enable_timeout_property_);

  // Create persistent billboard line chains for batched rendering
  bl_suggested_ref_same_remaining_ = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
  bl_suggested_ref_same_traveled_  = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
  bl_suggested_ref_adj_remaining_  = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
  bl_suggested_ref_adj_traveled_   = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);

  bl_suggested_bound_same_remaining_ = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
  bl_suggested_bound_same_traveled_  = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
  bl_suggested_bound_adj_remaining_  = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
  bl_suggested_bound_adj_traveled_   = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);

  bl_adjacent_ref_remaining_ = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
  bl_adjacent_ref_traveled_  = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);

  bl_adjacent_bound_remaining_ = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
  bl_adjacent_bound_traveled_  = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);

  bl_drivable_remaining_ = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
  bl_drivable_traveled_  = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);

  bl_lane_change_remaining_ = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
  bl_lane_change_traveled_  = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
}

void RouteDisplay::reset() {
  MFDClass::reset();
  destination_arrows_.clear();
  suggested_lane_reference_poses_.clear();
  // Clear persistent line chains (keep objects alive to avoid reallocation)
  if (bl_suggested_ref_same_remaining_) bl_suggested_ref_same_remaining_->clear();
  if (bl_suggested_ref_same_traveled_)  bl_suggested_ref_same_traveled_->clear();
  if (bl_suggested_ref_adj_remaining_)  bl_suggested_ref_adj_remaining_->clear();
  if (bl_suggested_ref_adj_traveled_)   bl_suggested_ref_adj_traveled_->clear();
  suggested_lane_boundary_points_.clear();
  if (bl_suggested_bound_same_remaining_) bl_suggested_bound_same_remaining_->clear();
  if (bl_suggested_bound_same_traveled_)  bl_suggested_bound_same_traveled_->clear();
  if (bl_suggested_bound_adj_remaining_)  bl_suggested_bound_adj_remaining_->clear();
  if (bl_suggested_bound_adj_traveled_)   bl_suggested_bound_adj_traveled_->clear();
  suggested_lane_regulatory_elements_.clear();
  suggested_lane_regulatory_elements_sign_positions_.clear();
  suggested_lane_regulatory_elements_timing_information_.clear();
  adjacent_lanes_reference_poses_.clear();
  if (bl_adjacent_ref_remaining_) bl_adjacent_ref_remaining_->clear();
  if (bl_adjacent_ref_traveled_)  bl_adjacent_ref_traveled_->clear();
  adjacent_lanes_boundary_points_.clear();
  if (bl_adjacent_bound_remaining_) bl_adjacent_bound_remaining_->clear();
  if (bl_adjacent_bound_traveled_)  bl_adjacent_bound_traveled_->clear();
  adjacent_lane_regulatory_elements_.clear();
  adjacent_lane_regulatory_elements_sign_positions_.clear();
  adjacent_lane_regulatory_elements_timing_information_.clear();
  drivable_space_points_.clear();
  if (bl_drivable_remaining_) bl_drivable_remaining_->clear();
  if (bl_drivable_traveled_)  bl_drivable_traveled_->clear();
  if (bl_lane_change_remaining_) bl_lane_change_remaining_->clear();
  if (bl_lane_change_traveled_)  bl_lane_change_traveled_->clear();
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
  }
  for (size_t i = 0; i < msg.regulatory_elements.size(); ++i) {
    valid = valid && rviz_common::validateFloats(msg.regulatory_elements[i].reference_line);
    valid = valid && rviz_common::validateFloats(msg.regulatory_elements[i].positions);
  }
  return valid;
}

bool validateFloats(const route_planning_msgs::msg::Route::ConstSharedPtr msg) {
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg->destination);
  for (size_t i = 0; i < msg->intermediate_destinations.size(); ++i) {
    valid = valid && rviz_common::validateFloats(msg->intermediate_destinations[i]);
  }
  for (size_t i = 0; i < msg->route_elements.size(); ++i) {
    valid = valid && validateFloats(msg->route_elements[i]);
  }
  return valid;
}

void RouteDisplay::processMessage(const route_planning_msgs::msg::Route::ConstSharedPtr msg) {
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

  // clear previous primitives but keep persistent OGRE objects alive
  reset();

  // display destination and intermediate destinations
  if (viz_destination_->getBool()) {
    std::shared_ptr<rviz_rendering::Arrow> destination_arrow = std::make_shared<rviz_rendering::Arrow>(scene_manager_, scene_node_,
      ARROW_SHAFT_LENGTH, ARROW_SHAFT_DIAMETER, ARROW_HEAD_LENGTH, ARROW_HEAD_DIAMETER);
    destination_arrow->setColor(rviz_common::properties::qtToOgre(color_property_destination_->getColor()));
    destination_arrow->setScale(Ogre::Vector3(scale_property_destination_->getFloat(), scale_property_destination_->getFloat(), scale_property_destination_->getFloat()));
    Ogre::Vector3 pos(msg->destination.x, msg->destination.y, msg->destination.z + ARROW_SHAFT_LENGTH + ARROW_HEAD_LENGTH);
    destination_arrow->setPosition(pos);
    destination_arrows_.push_back(destination_arrow);

    for (size_t i = 0; i < msg->intermediate_destinations.size(); ++i) {
      std::shared_ptr<rviz_rendering::Arrow> intermediate_arrow = std::make_shared<rviz_rendering::Arrow>(scene_manager_, scene_node_,
      ARROW_SHAFT_LENGTH, ARROW_SHAFT_DIAMETER, ARROW_HEAD_LENGTH, ARROW_HEAD_DIAMETER);
      intermediate_arrow->setColor(rviz_common::properties::qtToOgre(color_property_intermediate_destinations_->getColor()));
      intermediate_arrow->setScale(Ogre::Vector3(scale_property_intermediate_destinations_->getFloat(), scale_property_intermediate_destinations_->getFloat(), scale_property_intermediate_destinations_->getFloat()));
      Ogre::Vector3 pos(msg->intermediate_destinations[i].x, msg->intermediate_destinations[i].y, msg->intermediate_destinations[i].z + ARROW_SHAFT_LENGTH + ARROW_HEAD_LENGTH);
      intermediate_arrow->setPosition(pos);
      destination_arrows_.push_back(intermediate_arrow);
    }
  }

  // Get visualization settings once
  bool viz_suggested_lane_reference = viz_suggested_lane_->getBool() && viz_suggested_lane_reference_->getBool();
  bool viz_suggested_lane_boundaries = viz_suggested_lane_->getBool() && viz_suggested_lane_boundaries_->getBool();
  bool viz_adjacent_lanes_reference = viz_adjacent_lanes_->getBool() && viz_adjacent_lanes_reference_ ->getBool();
  bool viz_adjacent_lanes_boundaries = viz_adjacent_lanes_->getBool() && viz_adjacent_lanes_boundaries_->getBool();

  bool show_suggested_lane_reference_poses = viz_suggested_lane_reference && viz_suggested_lane_reference_poses_->getBool();
  bool show_suggested_lane_reference_line = viz_suggested_lane_reference && viz_suggested_lane_reference_line_->getBool();
  bool show_suggested_lane_boundary_points = viz_suggested_lane_boundaries && viz_suggested_lane_boundary_points_->getBool();
  bool show_suggested_lane_boundary_lines = viz_suggested_lane_boundaries && viz_suggested_lane_boundary_lines_->getBool();
  bool show_suggested_lane_regulatory_elements = viz_suggested_lane_->getBool() && viz_suggested_lane_regulatory_elements_->getBool();
  bool show_suggested_lane_regulatory_elements_sign_positions = show_suggested_lane_regulatory_elements && viz_suggested_lane_regulatory_elements_sign_positions_->getBool();
  bool show_suggested_lane_regulatory_elements_timing_information = show_suggested_lane_regulatory_elements && viz_suggested_lane_regulatory_elements_timing_information_->getBool();
  bool show_lane_change = viz_suggested_lane_->getBool() && viz_lane_change_->getBool();
  bool show_adjacent_lanes_reference_poses = viz_adjacent_lanes_reference && viz_adjacent_lanes_reference_poses_->getBool();
  bool show_adjacent_lanes_reference_line = viz_adjacent_lanes_reference && viz_adjacent_lanes_reference_line_->getBool();
  bool show_adjacent_lanes_boundary_points = viz_adjacent_lanes_boundaries && viz_adjacent_lanes_boundary_points_->getBool();
  bool show_adjacent_lanes_boundary_lines = viz_adjacent_lanes_boundaries && viz_adjacent_lanes_boundary_lines_->getBool();
  bool show_adjacent_lane_regulatory_elements = viz_adjacent_lanes_->getBool() && viz_adjacent_lane_regulatory_elements_->getBool();
  bool show_adjacent_lane_regulatory_elements_sign_positions = show_adjacent_lane_regulatory_elements && viz_adjacent_lane_regulatory_elements_sign_positions_->getBool();
  bool show_adjacent_lane_regulatory_elements_timing_information = show_adjacent_lane_regulatory_elements && viz_adjacent_lane_regulatory_elements_timing_information_->getBool();
  bool show_drivable_space_points = viz_drivable_space_->getBool() && viz_drivable_space_points_->getBool();
  bool show_drivable_space_lines = viz_drivable_space_->getBool() && viz_drivable_space_lines_->getBool();

  Ogre::ColourValue color_suggested_lane_reference_poses = rviz_common::properties::qtToOgre(color_property_suggested_lane_reference_poses_->getColor());
  Ogre::ColourValue color_suggested_lane_boundary_points = rviz_common::properties::qtToOgre(color_property_suggested_lane_boundary_points_->getColor());
  Ogre::ColourValue color_suggested_lane_regulatory_elements = rviz_common::properties::qtToOgre(color_property_suggested_lane_regulatory_elements_->getColor());
  Ogre::ColourValue color_suggested_lane_regulatory_elements_timing_information = rviz_common::properties::qtToOgre(color_property_suggested_lane_regulatory_elements_timing_information_->getColor());
  Ogre::ColourValue color_adjacent_lanes_reference_poses = rviz_common::properties::qtToOgre(color_property_adjacent_lanes_reference_poses_->getColor());
  Ogre::ColourValue color_adjacent_lanes_boundary_points = rviz_common::properties::qtToOgre(color_property_adjacent_lanes_boundary_points_->getColor());
  Ogre::ColourValue color_adjacent_lane_regulatory_elements = rviz_common::properties::qtToOgre(color_property_adjacent_lane_regulatory_elements_->getColor());
  Ogre::ColourValue color_adjacent_lane_regulatory_elements_timing_information = rviz_common::properties::qtToOgre(color_property_adjacent_lane_regulatory_elements_timing_information_->getColor());
  Ogre::ColourValue color_drivable_space_points = rviz_common::properties::qtToOgre(color_property_drivable_space_points_->getColor());

  float scale_suggested_lane_reference_poses = scale_property_suggested_lane_reference_poses_->getFloat();
  float scale_suggested_lane_boundary_points = scale_property_suggested_lane_boundary_points_->getFloat();
  float scale_suggested_lane_regulatory_elements = scale_property_suggested_lane_regulatory_elements_->getFloat();
  float scale_suggested_lane_regulatory_elements_timing_information = scale_property_suggested_lane_regulatory_elements_timing_information_->getFloat();
  float scale_adjacent_lanes_reference_poses = scale_property_adjacent_lanes_reference_poses_->getFloat();
  float scale_adjacent_lanes_boundary_points = scale_property_adjacent_lanes_boundary_points_->getFloat();
  float scale_adjacent_lane_regulatory_elements = scale_property_adjacent_lane_regulatory_elements_->getFloat();
  float scale_adjacent_lane_regulatory_elements_timing_information = scale_property_adjacent_lane_regulatory_elements_timing_information_->getFloat();
  float scale_drivable_space_points = scale_property_drivable_space_points_->getFloat();

  // Pre-count segments for each batched line to size chains properly
  const size_t n = msg->route_elements.size();
  size_t cnt_sugg_ref_same_remaining = 0, cnt_sugg_ref_same_traveled = 0;
  size_t cnt_sugg_ref_adj_remaining = 0, cnt_sugg_ref_adj_traveled = 0;
  size_t cnt_sugg_bound_same_remaining = 0, cnt_sugg_bound_same_traveled = 0;
  size_t cnt_sugg_bound_adj_remaining = 0, cnt_sugg_bound_adj_traveled = 0;
  size_t cnt_adj_ref_remaining = 0, cnt_adj_ref_traveled = 0;
  size_t cnt_adj_bound_remaining = 0, cnt_adj_bound_traveled = 0;
  size_t cnt_drivable_remaining = 0, cnt_drivable_traveled = 0; // each segment counts as 2 (left+right)
  size_t cnt_lane_change_remaining = 0, cnt_lane_change_traveled = 0;

  if (n >= 2) {
    for (size_t i = 0; i < n - 1; ++i) {
      const auto & re = msg->route_elements[i];
      const auto & ren = msg->route_elements[i + 1];
      const bool is_traveled = (i < msg->current_route_element_idx);

      // Suggested lane reference/boundary segments
      if (viz_suggested_lane_reference && viz_suggested_lane_reference_line_->getBool()) {
        const auto & sl = route_planning_msgs::route_access::getSuggestedLaneElement(re);
        if (auto res = route_planning_msgs::route_access::getFollowingLaneElement(sl, ren)) {
          bool is_adj = false;
          if (auto idx_res = route_planning_msgs::route_access::getFollowingLaneElementIdx(sl, ren)) {
            is_adj = (*idx_res != ren.suggested_lane_idx);
          }
          if (is_adj) {
            (is_traveled ? cnt_sugg_ref_adj_traveled : cnt_sugg_ref_adj_remaining) += 1;
          } else {
            (is_traveled ? cnt_sugg_ref_same_traveled : cnt_sugg_ref_same_remaining) += 1;
          }
        }
      }
      if (viz_suggested_lane_boundaries && viz_suggested_lane_boundary_lines_->getBool() && re.is_enriched && ren.is_enriched) {
        const auto & sl = route_planning_msgs::route_access::getSuggestedLaneElement(re);
        if (auto res = route_planning_msgs::route_access::getFollowingLaneElement(sl, ren)) {
          bool is_adj = false;
          if (auto idx_res = route_planning_msgs::route_access::getFollowingLaneElementIdx(sl, ren)) {
            is_adj = (*idx_res != ren.suggested_lane_idx);
          }
          // two segments (left/right)
          if (is_adj) {
            (is_traveled ? cnt_sugg_bound_adj_traveled : cnt_sugg_bound_adj_remaining) += 2;
          } else {
            (is_traveled ? cnt_sugg_bound_same_traveled : cnt_sugg_bound_same_remaining) += 2;
          }
        }
      }

      // Adjacent lane reference/boundary segments
      if (viz_adjacent_lanes_reference && viz_adjacent_lanes_reference_line_->getBool() && re.is_enriched && ren.is_enriched) {
        for (size_t j = 0; j < re.lane_elements.size(); ++j) {
          if (j == re.suggested_lane_idx) continue;
          const auto & adj = re.lane_elements[j];
          if (auto res = route_planning_msgs::route_access::getFollowingLaneElement(adj, ren)) {
            (is_traveled ? cnt_adj_ref_traveled : cnt_adj_ref_remaining) += 1;
          }
        }
      }
      if (viz_adjacent_lanes_boundaries && viz_adjacent_lanes_boundary_lines_->getBool() && re.is_enriched && ren.is_enriched) {
        for (size_t j = 0; j < re.lane_elements.size(); ++j) {
          if (j == re.suggested_lane_idx) continue;
          const auto & adj = re.lane_elements[j];
          if (auto res = route_planning_msgs::route_access::getFollowingLaneElement(adj, ren)) {
            (is_traveled ? cnt_adj_bound_traveled : cnt_adj_bound_remaining) += 2;
          }
        }
      }

      // Drivable space
      if (viz_drivable_space_->getBool() && viz_drivable_space_lines_->getBool() && re.is_enriched && ren.is_enriched) {
        (is_traveled ? cnt_drivable_traveled : cnt_drivable_remaining) += 2;
      }

      // Lane change
      if (viz_suggested_lane_->getBool() && viz_lane_change_->getBool() && re.will_change_suggested_lane) {
        (is_traveled ? cnt_lane_change_traveled : cnt_lane_change_remaining) += 1;
      }
    }
  }

  // Prepare chains: allocate lines and points per line once per message
  // Important: set max points per line BEFORE setting number of lines to avoid
  // excessive default values multiplying into huge allocations inside OGRE.
  auto prep_chain = [](std::shared_ptr<rviz_rendering::BillboardLine> &chain, size_t num_lines){
    if (!chain) return;
    chain->setMaxPointsPerLine(2);
    // Clamp to a sane upper bound in case of malformed messages
    const size_t kMaxLines = static_cast<size_t>(std::numeric_limits<int>::max() - 1);
    const size_t clamped = std::min(std::max<size_t>(num_lines, 1), kMaxLines);
    chain->setNumLines(static_cast<int>(clamped));
  };

  prep_chain(bl_suggested_ref_same_remaining_, cnt_sugg_ref_same_remaining);
  prep_chain(bl_suggested_ref_same_traveled_,  cnt_sugg_ref_same_traveled);
  prep_chain(bl_suggested_ref_adj_remaining_,  cnt_sugg_ref_adj_remaining);
  prep_chain(bl_suggested_ref_adj_traveled_,   cnt_sugg_ref_adj_traveled);

  prep_chain(bl_suggested_bound_same_remaining_, cnt_sugg_bound_same_remaining);
  prep_chain(bl_suggested_bound_same_traveled_,  cnt_sugg_bound_same_traveled);
  prep_chain(bl_suggested_bound_adj_remaining_,  cnt_sugg_bound_adj_remaining);
  prep_chain(bl_suggested_bound_adj_traveled_,   cnt_sugg_bound_adj_traveled);

  prep_chain(bl_adjacent_ref_remaining_, cnt_adj_ref_remaining);
  prep_chain(bl_adjacent_ref_traveled_,  cnt_adj_ref_traveled);
  prep_chain(bl_adjacent_bound_remaining_, cnt_adj_bound_remaining);
  prep_chain(bl_adjacent_bound_traveled_,  cnt_adj_bound_traveled);

  prep_chain(bl_drivable_remaining_, cnt_drivable_remaining);
  prep_chain(bl_drivable_traveled_,  cnt_drivable_traveled);

  prep_chain(bl_lane_change_remaining_, cnt_lane_change_remaining);
  prep_chain(bl_lane_change_traveled_,  cnt_lane_change_traveled);

  // loop over route elements and fill chains
  for (size_t i = 0; i < msg->route_elements.size(); ++i) {
    const auto& route_element = msg->route_elements[i];

    // check if traveled or remaining route
    const bool is_traveled_route = (i < msg->current_route_element_idx);
    if (!viz_traveled_route_->getBool() && is_traveled_route) {
      continue;
    }
    const float opacity = is_traveled_route ? opacity_property_traveled_route_->getFloat() : 1.0;

    // display suggested lane reference poses
    if (show_suggested_lane_reference_poses) {
      const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
      suggested_lane_reference_poses_.push_back(generateRenderArrow(suggested_lane.reference_pose, color_suggested_lane_reference_poses, scale_suggested_lane_reference_poses, opacity));
    }

    // display suggested lane reference line (batched)
    if (show_suggested_lane_reference_line && (i < msg->route_elements.size() - 1)) {
      const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
      if (auto result = route_planning_msgs::route_access::getFollowingLaneElement(suggested_lane, msg->route_elements[i + 1])) {
        const auto& following_lane = *result;
        bool is_adjacent_follow = false;
        if (auto inner_result = route_planning_msgs::route_access::getFollowingLaneElementIdx(suggested_lane, msg->route_elements[i + 1])) {
          is_adjacent_follow = (*inner_result != msg->route_elements[i + 1].suggested_lane_idx);
        }
        auto chain = (is_adjacent_follow
                        ? (is_traveled_route ? bl_suggested_ref_adj_traveled_ : bl_suggested_ref_adj_remaining_)
                        : (is_traveled_route ? bl_suggested_ref_same_traveled_ : bl_suggested_ref_same_remaining_));
        const float z_off = is_adjacent_follow ? 0.0f : VERTICAL_OFFSET_EPSILON;
        chain->addPoint(Ogre::Vector3(suggested_lane.reference_pose.position.x,
                                      suggested_lane.reference_pose.position.y,
                                      suggested_lane.reference_pose.position.z + z_off));
        chain->addPoint(Ogre::Vector3(following_lane.reference_pose.position.x,
                                      following_lane.reference_pose.position.y,
                                      following_lane.reference_pose.position.z + z_off));
        chain->finishLine();
      }
    }

    // display suggested lane boundary points
    if (show_suggested_lane_boundary_points && route_element.is_enriched) {
      const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
      suggested_lane_boundary_points_.push_back(generateRenderPoint(suggested_lane.left_boundary.point, color_suggested_lane_boundary_points, scale_suggested_lane_boundary_points, opacity));
      suggested_lane_boundary_points_.push_back(generateRenderPoint(suggested_lane.right_boundary.point, color_suggested_lane_boundary_points, scale_suggested_lane_boundary_points, opacity));
    }

    // display suggested lane boundary lines (batched)
    if (show_suggested_lane_boundary_lines && (i < msg->route_elements.size() - 1) && route_element.is_enriched && msg->route_elements[i + 1].is_enriched) {
      const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
      if (auto result = route_planning_msgs::route_access::getFollowingLaneElement(suggested_lane, msg->route_elements[i + 1])) {
        const auto& following_lane = *result;
        bool is_adjacent_follow = false;
        if (auto inner_result = route_planning_msgs::route_access::getFollowingLaneElementIdx(suggested_lane, msg->route_elements[i + 1])) {
          is_adjacent_follow = (*inner_result != msg->route_elements[i + 1].suggested_lane_idx);
        }
        auto chain_left = (is_adjacent_follow
                             ? (is_traveled_route ? bl_suggested_bound_adj_traveled_ : bl_suggested_bound_adj_remaining_)
                             : (is_traveled_route ? bl_suggested_bound_same_traveled_ : bl_suggested_bound_same_remaining_));
        auto chain_right = chain_left;  // same target chain and offset
        const float z_off = is_adjacent_follow ? 0.0f : VERTICAL_OFFSET_EPSILON;
        // left
        chain_left->addPoint(Ogre::Vector3(suggested_lane.left_boundary.point.x,
                                           suggested_lane.left_boundary.point.y,
                                           suggested_lane.left_boundary.point.z + z_off));
        chain_left->addPoint(Ogre::Vector3(following_lane.left_boundary.point.x,
                                           following_lane.left_boundary.point.y,
                                           following_lane.left_boundary.point.z + z_off));
        chain_left->finishLine();
        // right
        chain_right->addPoint(Ogre::Vector3(suggested_lane.right_boundary.point.x,
                                            suggested_lane.right_boundary.point.y,
                                            suggested_lane.right_boundary.point.z + z_off));
        chain_right->addPoint(Ogre::Vector3(following_lane.right_boundary.point.x,
                                            following_lane.right_boundary.point.y,
                                            following_lane.right_boundary.point.z + z_off));
        chain_right->finishLine();
      }
    }

    // display suggested lane regulatory elements
    if (show_suggested_lane_regulatory_elements && route_element.is_enriched) {
      const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
      for (const auto& index : suggested_lane.regulatory_element_idcs) {
        const auto& regulatory_element = route_element.regulatory_elements[index];
        Ogre::ColourValue color_reg_elem = color_suggested_lane_regulatory_elements;
        if (regulatory_element.type == route_planning_msgs::msg::RegulatoryElement::TYPE_TRAFFIC_LIGHT) {
          if (regulatory_element.meta_value == route_planning_msgs::msg::RegulatoryElement::META_VALUE_MOVEMENT_ALLOWED) {
            color_reg_elem = Ogre::ColourValue(0, 255, 0);
          } else if (regulatory_element.meta_value == route_planning_msgs::msg::RegulatoryElement::META_VALUE_MOVEMENT_RESTRICTED) {
            color_reg_elem = Ogre::ColourValue(255, 0, 0);
          }
        }
        std::vector<geometry_msgs::msg::Point> points(regulatory_element.reference_line.begin(), regulatory_element.reference_line.end());
        suggested_lane_regulatory_elements_.push_back(generateRenderLine(points, color_reg_elem, scale_suggested_lane_regulatory_elements, opacity, 2 * VERTICAL_OFFSET_EPSILON));
        if (show_suggested_lane_regulatory_elements_sign_positions) {
          for (const auto& position : regulatory_element.positions) {
            suggested_lane_regulatory_elements_sign_positions_.push_back(generateRenderPoint(position, color_reg_elem, 0.5));
            if (show_suggested_lane_regulatory_elements_timing_information) {
              std::string text = "no stamp";
              if (regulatory_element.has_validity_stamp) {
                double validity_stamp = rclcpp::Time(regulatory_element.validity_stamp).seconds();
                std::stringstream ss;
                ss << std::fixed << std::setprecision(2) << validity_stamp;
                text = ss.str();
              }
              auto timing_text = std::make_shared<rviz_rendering::MovableText>(text, "Liberation Sans", scale_suggested_lane_regulatory_elements_timing_information, color_suggested_lane_regulatory_elements_timing_information);
              Ogre::Vector3 text_position(position.x, position.y, position.z + 0.5);
              timing_text->setGlobalTranslation(text_position);
              scene_node_->attachObject(timing_text.get());
              suggested_lane_regulatory_elements_timing_information_.push_back(timing_text);
            }
          }
        } 
      }
    }

    // display adjacent lanes poses
    if (show_adjacent_lanes_reference_poses && route_element.is_enriched) {
      for (size_t j = 0; j < route_element.lane_elements.size(); ++j) {
        if (j != route_element.suggested_lane_idx) {
          const auto& adjacent_lane = route_element.lane_elements[j];
          adjacent_lanes_reference_poses_.push_back(generateRenderArrow(adjacent_lane.reference_pose, color_adjacent_lanes_reference_poses, scale_adjacent_lanes_reference_poses, opacity));
        }
      }
    }

    // display adjacent lanes reference line (batched)
    if (show_adjacent_lanes_reference_line && (i < msg->route_elements.size() - 1) && route_element.is_enriched && msg->route_elements[i + 1].is_enriched) {
      for (size_t j = 0; j < route_element.lane_elements.size(); ++j) {
        if (j != route_element.suggested_lane_idx) {
          const auto& adjacent_lane = route_element.lane_elements[j];
          if (auto result = route_planning_msgs::route_access::getFollowingLaneElement(adjacent_lane, msg->route_elements[i + 1])) {
            const auto& following_lane = *result;
            auto chain = is_traveled_route ? bl_adjacent_ref_traveled_ : bl_adjacent_ref_remaining_;
            chain->addPoint(Ogre::Vector3(adjacent_lane.reference_pose.position.x,
                                          adjacent_lane.reference_pose.position.y,
                                          adjacent_lane.reference_pose.position.z));
            chain->addPoint(Ogre::Vector3(following_lane.reference_pose.position.x,
                                          following_lane.reference_pose.position.y,
                                          following_lane.reference_pose.position.z));
            chain->finishLine();
          }
        }
      }
    }

    // display adjacent lanes boundary points
    if (show_adjacent_lanes_boundary_points && route_element.is_enriched) {
      for (size_t j = 0; j < route_element.lane_elements.size(); ++j) {
        if (j != route_element.suggested_lane_idx) {
          const auto& adjacent_lane = route_element.lane_elements[j];
          adjacent_lanes_boundary_points_.push_back(generateRenderPoint(adjacent_lane.left_boundary.point, color_adjacent_lanes_boundary_points, scale_adjacent_lanes_boundary_points, opacity));
          adjacent_lanes_boundary_points_.push_back(generateRenderPoint(adjacent_lane.right_boundary.point, color_adjacent_lanes_boundary_points, scale_adjacent_lanes_boundary_points, opacity));
        }
      }
    }

    // display adjacent lanes boundary lines (batched)
    if (show_adjacent_lanes_boundary_lines && (i < msg->route_elements.size() - 1) && route_element.is_enriched && msg->route_elements[i + 1].is_enriched) {
      for (size_t j = 0; j < route_element.lane_elements.size(); ++j) {
        if (j != route_element.suggested_lane_idx) {
          const auto& adjacent_lane = route_element.lane_elements[j];
          if (auto result = route_planning_msgs::route_access::getFollowingLaneElement(adjacent_lane, msg->route_elements[i + 1])) {
            const auto& following_lane = *result;
            auto chain = is_traveled_route ? bl_adjacent_bound_traveled_ : bl_adjacent_bound_remaining_;
            // left
            chain->addPoint(Ogre::Vector3(adjacent_lane.left_boundary.point.x,
                                          adjacent_lane.left_boundary.point.y,
                                          adjacent_lane.left_boundary.point.z));
            chain->addPoint(Ogre::Vector3(following_lane.left_boundary.point.x,
                                          following_lane.left_boundary.point.y,
                                          following_lane.left_boundary.point.z));
            chain->finishLine();
            // right
            chain->addPoint(Ogre::Vector3(adjacent_lane.right_boundary.point.x,
                                          adjacent_lane.right_boundary.point.y,
                                          adjacent_lane.right_boundary.point.z));
            chain->addPoint(Ogre::Vector3(following_lane.right_boundary.point.x,
                                          following_lane.right_boundary.point.y,
                                          following_lane.right_boundary.point.z));
            chain->finishLine();
          }
        }
      }
    }

    // display adjacent lane regulatory elements
    if (show_adjacent_lane_regulatory_elements && route_element.is_enriched) {
      for (size_t j = 0; j < route_element.lane_elements.size(); ++j) {
        if (j != route_element.suggested_lane_idx) {
          const auto& adjacent_lane = route_element.lane_elements[j];
          for (const auto& index : adjacent_lane.regulatory_element_idcs) {
            const auto& regulatory_element = route_element.regulatory_elements[index];
            Ogre::ColourValue color_reg_elem = color_adjacent_lane_regulatory_elements;
            if (regulatory_element.type == route_planning_msgs::msg::RegulatoryElement::TYPE_TRAFFIC_LIGHT) {
              if (regulatory_element.meta_value == route_planning_msgs::msg::RegulatoryElement::META_VALUE_MOVEMENT_ALLOWED) {
                color_reg_elem = Ogre::ColourValue(0, 255, 0);
              } else if (regulatory_element.meta_value == route_planning_msgs::msg::RegulatoryElement::META_VALUE_MOVEMENT_RESTRICTED) {
                color_reg_elem = Ogre::ColourValue(255, 0, 0);
              }
            }
            std::vector<geometry_msgs::msg::Point> points(regulatory_element.reference_line.begin(), regulatory_element.reference_line.end());
            adjacent_lane_regulatory_elements_.push_back(generateRenderLine(points, color_reg_elem, scale_adjacent_lane_regulatory_elements, opacity));
            if (show_adjacent_lane_regulatory_elements_sign_positions) {
              for (const auto& position : regulatory_element.positions) {
                adjacent_lane_regulatory_elements_sign_positions_.push_back(generateRenderPoint(position, color_reg_elem, 0.5));
                if (show_adjacent_lane_regulatory_elements_timing_information) {
                  std::string text = "no stamp";
                  if (regulatory_element.has_validity_stamp) {
                    double validity_stamp = rclcpp::Time(regulatory_element.validity_stamp).seconds();
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << validity_stamp;
                    text = ss.str();
                  }
                  auto timing_text = std::make_shared<rviz_rendering::MovableText>(text, "Liberation Sans", scale_adjacent_lane_regulatory_elements_timing_information, color_adjacent_lane_regulatory_elements_timing_information);
                  Ogre::Vector3 text_position(position.x, position.y, position.z + 0.5);
                  timing_text->setGlobalTranslation(text_position);
                  scene_node_->attachObject(timing_text.get());
                  adjacent_lane_regulatory_elements_timing_information_.push_back(timing_text);
                }
              }
            }
          }
        }
      }
    }

    // display drivable space points
    if (show_drivable_space_points && route_element.is_enriched) {
      drivable_space_points_.push_back(generateRenderPoint(route_element.left_boundary, color_drivable_space_points, scale_drivable_space_points, opacity));
      drivable_space_points_.push_back(generateRenderPoint(route_element.right_boundary, color_drivable_space_points, scale_drivable_space_points, opacity));
    }

    // display drivable space lines (batched)
    if (show_drivable_space_lines && (i < msg->route_elements.size() - 1) && route_element.is_enriched && msg->route_elements[i + 1].is_enriched) {
      auto chain = is_traveled_route ? bl_drivable_traveled_ : bl_drivable_remaining_;
      const float z_off = 3.0f * VERTICAL_OFFSET_EPSILON;
      // left
      chain->addPoint(Ogre::Vector3(route_element.left_boundary.x,
                                    route_element.left_boundary.y,
                                    route_element.left_boundary.z + z_off));
      chain->addPoint(Ogre::Vector3(msg->route_elements[i + 1].left_boundary.x,
                                    msg->route_elements[i + 1].left_boundary.y,
                                    msg->route_elements[i + 1].left_boundary.z + z_off));
      chain->finishLine();
      // right
      chain->addPoint(Ogre::Vector3(route_element.right_boundary.x,
                                    route_element.right_boundary.y,
                                    route_element.right_boundary.z + z_off));
      chain->addPoint(Ogre::Vector3(msg->route_elements[i + 1].right_boundary.x,
                                    msg->route_elements[i + 1].right_boundary.y,
                                    msg->route_elements[i + 1].right_boundary.z + z_off));
      chain->finishLine();
    }

    // display lane change lines (batched)
    if (show_lane_change && (i < msg->route_elements.size() - 1)) {
      if (route_element.will_change_suggested_lane) {
        const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
        const auto& next_suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(msg->route_elements[i + 1]);
        auto chain = is_traveled_route ? bl_lane_change_traveled_ : bl_lane_change_remaining_;
        const float z_off = VERTICAL_OFFSET_EPSILON;
        chain->addPoint(Ogre::Vector3(suggested_lane.reference_pose.position.x,
                                      suggested_lane.reference_pose.position.y,
                                      suggested_lane.reference_pose.position.z + z_off));
        chain->addPoint(Ogre::Vector3(next_suggested_lane.reference_pose.position.x,
                                      next_suggested_lane.reference_pose.position.y,
                                      next_suggested_lane.reference_pose.position.z + z_off));
        chain->finishLine();
      }
    }
  }

  // Update chain render properties (color and width) once per message
  // Colors
  const Ogre::ColourValue color_sugg_ref = rviz_common::properties::qtToOgre(color_property_suggested_lane_reference_line_->getColor());
  const Ogre::ColourValue color_adj_ref  = rviz_common::properties::qtToOgre(color_property_adjacent_lanes_reference_line_->getColor());
  const Ogre::ColourValue color_sugg_bound = rviz_common::properties::qtToOgre(color_property_suggested_lane_boundary_lines_->getColor());
  const Ogre::ColourValue color_adj_bound  = rviz_common::properties::qtToOgre(color_property_adjacent_lanes_boundary_lines_->getColor());
  const Ogre::ColourValue color_drivable   = rviz_common::properties::qtToOgre(color_property_drivable_space_lines_->getColor());
  const Ogre::ColourValue color_lane_change_prop = rviz_common::properties::qtToOgre(color_property_lane_change_->getColor());

  const float width_sugg_ref  = scale_property_suggested_lane_reference_line_->getFloat();
  const float width_adj_ref   = scale_property_adjacent_lanes_reference_line_->getFloat();
  const float width_sugg_bound= scale_property_suggested_lane_boundary_lines_->getFloat();
  const float width_adj_bound = scale_property_adjacent_lanes_boundary_lines_->getFloat();
  const float width_drivable  = scale_property_drivable_space_lines_->getFloat();
  const float width_lane_change= scale_property_lane_change_->getFloat();

  const float traveled_alpha = viz_traveled_route_->getBool() ? opacity_property_traveled_route_->getFloat() : 1.0f;

  // Set colors and widths
  bl_suggested_ref_same_remaining_->setColor(color_sugg_ref.r, color_sugg_ref.g, color_sugg_ref.b, 1.0f);
  bl_suggested_ref_same_traveled_->setColor(color_sugg_ref.r, color_sugg_ref.g, color_sugg_ref.b, traveled_alpha);
  bl_suggested_ref_adj_remaining_->setColor(color_adj_ref.r, color_adj_ref.g, color_adj_ref.b, 1.0f);
  bl_suggested_ref_adj_traveled_->setColor(color_adj_ref.r, color_adj_ref.g, color_adj_ref.b, traveled_alpha);
  bl_suggested_ref_same_remaining_->setLineWidth(width_sugg_ref);
  bl_suggested_ref_same_traveled_->setLineWidth(width_sugg_ref);
  bl_suggested_ref_adj_remaining_->setLineWidth(width_adj_ref);
  bl_suggested_ref_adj_traveled_->setLineWidth(width_adj_ref);

  bl_suggested_bound_same_remaining_->setColor(color_sugg_bound.r, color_sugg_bound.g, color_sugg_bound.b, 1.0f);
  bl_suggested_bound_same_traveled_->setColor(color_sugg_bound.r, color_sugg_bound.g, color_sugg_bound.b, traveled_alpha);
  bl_suggested_bound_adj_remaining_->setColor(color_adj_bound.r, color_adj_bound.g, color_adj_bound.b, 1.0f);
  bl_suggested_bound_adj_traveled_->setColor(color_adj_bound.r, color_adj_bound.g, color_adj_bound.b, traveled_alpha);
  bl_suggested_bound_same_remaining_->setLineWidth(scale_property_suggested_lane_boundary_lines_->getFloat());
  bl_suggested_bound_same_traveled_->setLineWidth(scale_property_suggested_lane_boundary_lines_->getFloat());
  bl_suggested_bound_adj_remaining_->setLineWidth(scale_property_adjacent_lanes_boundary_lines_->getFloat());
  bl_suggested_bound_adj_traveled_->setLineWidth(scale_property_adjacent_lanes_boundary_lines_->getFloat());

  bl_adjacent_ref_remaining_->setColor(color_adj_ref.r, color_adj_ref.g, color_adj_ref.b, 1.0f);
  bl_adjacent_ref_traveled_->setColor(color_adj_ref.r, color_adj_ref.g, color_adj_ref.b, traveled_alpha);
  bl_adjacent_ref_remaining_->setLineWidth(width_adj_ref);
  bl_adjacent_ref_traveled_->setLineWidth(width_adj_ref);

  bl_adjacent_bound_remaining_->setColor(color_adj_bound.r, color_adj_bound.g, color_adj_bound.b, 1.0f);
  bl_adjacent_bound_traveled_->setColor(color_adj_bound.r, color_adj_bound.g, color_adj_bound.b, traveled_alpha);
  bl_adjacent_bound_remaining_->setLineWidth(width_adj_bound);
  bl_adjacent_bound_traveled_->setLineWidth(width_adj_bound);

  bl_drivable_remaining_->setColor(color_drivable.r, color_drivable.g, color_drivable.b, 1.0f);
  bl_drivable_traveled_->setColor(color_drivable.r, color_drivable.g, color_drivable.b, traveled_alpha);
  bl_drivable_remaining_->setLineWidth(width_drivable);
  bl_drivable_traveled_->setLineWidth(width_drivable);

  bl_lane_change_remaining_->setColor(color_lane_change_prop.r, color_lane_change_prop.g, color_lane_change_prop.b, 1.0f);
  bl_lane_change_traveled_->setColor(color_lane_change_prop.r, color_lane_change_prop.g, color_lane_change_prop.b, traveled_alpha);
  bl_lane_change_remaining_->setLineWidth(width_lane_change);
  bl_lane_change_traveled_->setLineWidth(width_lane_change);

  // reset scene after timeout, if enabled
  if (enable_timeout_property_->getBool()) {
    timeout_timer_ = rviz_ros_node_.lock()->get_raw_node()->create_wall_timer(
      std::chrono::duration<float>(timeout_property_->getFloat()),
      std::bind(&RouteDisplay::timeoutTimerCallback, this)
    );
  }
}

std::shared_ptr<rviz_rendering::Arrow> RouteDisplay::generateRenderArrow(const geometry_msgs::msg::Pose& pose, const Ogre::ColourValue& color, const float scale, const float opacity) {
  std::shared_ptr<rviz_rendering::Arrow> arrow = std::make_shared<rviz_rendering::Arrow>(scene_manager_, scene_node_);
  Ogre::Vector3 pos(pose.position.x, pose.position.y, pose.position.z);
  arrow->setPosition(pos);
  tf2::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Vector3 unit_vector(1, 0, 0);
  tf2::Vector3 direction = tf2::quatRotate(quat, unit_vector);
  Ogre::Vector3 dir(direction.getX(), direction.getY(), direction.getZ());
  arrow->setDirection(dir);
  arrow->setColor(color.r, color.g, color.b, opacity);
  arrow->setScale(Ogre::Vector3(scale, scale, scale));
  return arrow;
}

std::shared_ptr<rviz_rendering::BillboardLine> RouteDisplay::generateRenderLine(const std::vector<geometry_msgs::msg::Point>& points, const Ogre::ColourValue& color, const float scale, const float opacity, const float vertical_offset) {
  std::shared_ptr<rviz_rendering::BillboardLine> billboard_line = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
  for (const auto& point : points) {
    Ogre::Vector3 pos(point.x, point.y, point.z + vertical_offset);
    billboard_line->addPoint(pos);
  }
  billboard_line->setColor(color.r, color.g, color.b, opacity);
  billboard_line->setLineWidth(scale);
  billboard_line->finishLine();
  return billboard_line;
}

std::shared_ptr<rviz_rendering::Shape> RouteDisplay::generateRenderPoint(const geometry_msgs::msg::Point& point, const Ogre::ColourValue& color, const float scale, const float opacity) {
  std::shared_ptr<rviz_rendering::Shape> sphere = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_);
  Ogre::Vector3 pos(point.x, point.y, point.z);
  sphere->setPosition(pos);
  sphere->setColor(color.r, color.g, color.b, opacity);
  sphere->setScale(Ogre::Vector3(scale, scale, scale));
  return sphere;
}

void RouteDisplay::timeoutTimerCallback() {
  timeout_timer_->cancel();
  this->reset();
}

}  // namespace displays
}  // namespace route_planning_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(route_planning_msgs::displays::RouteDisplay, rviz_common::Display)
