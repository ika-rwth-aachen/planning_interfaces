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

namespace route_planning_msgs {
namespace displays {

void RouteDisplay::onInitialize() {
  MFDClass::onInitialize();

  // destination
  viz_destination_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Destination", true, "Whether to display the destination arrow.", this);
  color_property_destination_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 0, 255), "Color to draw the destination arrow.", viz_destination_.get());
  scale_property_destination_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 2.0, "Scale of the destination arrow.", viz_destination_.get());

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
      "Scale", 0.2, "Scale of the reference and lane boundary points of the suggested lane.", viz_suggested_lane_boundary_points_.get());

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
      "Reference Positions", true, "Whether to display the sign positions of the regulatory elements of the suggested lane.", viz_suggested_lane_regulatory_elements_.get());

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
      "Reference Positions", true, "Whether to display the sign positions of the regulatory elements of adjacent lanes.", viz_adjacent_lane_regulatory_elements_.get());

  // driveable space
  viz_driveable_space_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Driveable Space", true, "Whether to display the reference and lane boundary points of the driveable space.", this);
  color_property_driveable_space_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 0, 0), "Color to draw reference and lane boundary points of the driveable space.", viz_driveable_space_.get());
  scale_property_driveable_space_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.2, "Scale of the reference and lane boundary points of the driveable space.", viz_driveable_space_.get());

  // lane change
  viz_lane_change_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Lane Change", true, "Whether to display the lane change lines.", this);
  color_property_lane_change_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(0, 255, 0), "Color to draw lane change lines.", viz_lane_change_.get());
  scale_property_lane_change_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.2, "Scale of the lane change lines.", viz_lane_change_.get());
}

void RouteDisplay::reset() {
  MFDClass::reset();
  destination_arrow_.reset();
  suggested_lane_reference_poses_.clear();
  suggested_lane_reference_line_.clear();
  suggested_lane_boundary_points_.clear();
  suggested_lane_boundary_lines_.clear();
  suggested_lane_regulatory_elements_.clear();
  suggested_lane_regulatory_elements_sign_positions_.clear();
  adjacent_lanes_reference_poses_.clear();
  adjacent_lanes_reference_line_.clear();
  adjacent_lanes_boundary_points_.clear();
  adjacent_lanes_boundary_lines_.clear();
  adjacent_lane_regulatory_elements_.clear();
  adjacent_lane_regulatory_elements_sign_positions_.clear();
  drivable_space_points_.clear();
  lane_change_lines_.clear();
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
  for (size_t i = 0; i < msg->traveled_route_elements.size(); ++i) {
    valid = valid && validateFloats(msg->traveled_route_elements[i]);
  }
  for (size_t i = 0; i < msg->remaining_route_elements.size(); ++i) {
    valid = valid && validateFloats(msg->remaining_route_elements[i]);
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

  // clear previous points in arrays
  reset();

  // display destination
  if (viz_destination_->getBool()) {
    geometry_msgs::msg::Pose destination;
    destination.position = msg->destination;
    Ogre::ColourValue destination_color = rviz_common::properties::qtToOgre(color_property_destination_->getColor());
    float destination_scale = scale_property_destination_->getFloat();
    destination_arrow_ = generateRenderArrow(destination, destination_color, destination_scale);
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
  bool show_adjacent_lanes_reference_poses = viz_adjacent_lanes_reference && viz_adjacent_lanes_reference_poses_->getBool();
  bool show_adjacent_lanes_reference_line = viz_adjacent_lanes_reference && viz_adjacent_lanes_reference_line_->getBool();
  bool show_adjacent_lanes_boundary_points = viz_adjacent_lanes_boundaries && viz_adjacent_lanes_boundary_points_->getBool();
  bool show_adjacent_lanes_boundary_lines = viz_adjacent_lanes_boundaries && viz_adjacent_lanes_boundary_lines_->getBool();
  bool show_adjacent_lane_regulatory_elements = viz_adjacent_lanes_->getBool() && viz_adjacent_lane_regulatory_elements_->getBool();
  bool show_adjacent_lane_regulatory_elements_sign_positions = show_adjacent_lane_regulatory_elements && viz_adjacent_lane_regulatory_elements_sign_positions_->getBool();
  bool show_drivable_space = viz_driveable_space_->getBool();
  bool show_lane_change = viz_lane_change_->getBool();

  Ogre::ColourValue color_suggested_lane_reference_poses = rviz_common::properties::qtToOgre(color_property_suggested_lane_reference_poses_->getColor());
  Ogre::ColourValue color_suggested_lane_reference_line = rviz_common::properties::qtToOgre(color_property_suggested_lane_reference_line_->getColor());
  Ogre::ColourValue color_suggested_lane_boundary_points = rviz_common::properties::qtToOgre(color_property_suggested_lane_boundary_points_->getColor());
  Ogre::ColourValue color_suggested_lane_boundary_lines = rviz_common::properties::qtToOgre(color_property_suggested_lane_boundary_lines_->getColor());
  Ogre::ColourValue color_suggested_lane_regulatory_elements = rviz_common::properties::qtToOgre(color_property_suggested_lane_regulatory_elements_->getColor());
  Ogre::ColourValue color_adjacent_lanes_reference_poses = rviz_common::properties::qtToOgre(color_property_adjacent_lanes_reference_poses_->getColor());
  Ogre::ColourValue color_adjacent_lanes_reference_line = rviz_common::properties::qtToOgre(color_property_adjacent_lanes_reference_line_->getColor());
  Ogre::ColourValue color_adjacent_lanes_boundary_points = rviz_common::properties::qtToOgre(color_property_adjacent_lanes_boundary_points_->getColor());
  Ogre::ColourValue color_adjacent_lanes_boundary_lines = rviz_common::properties::qtToOgre(color_property_adjacent_lanes_boundary_lines_->getColor());
  Ogre::ColourValue color_adjacent_lane_regulatory_elements = rviz_common::properties::qtToOgre(color_property_adjacent_lane_regulatory_elements_->getColor());
  Ogre::ColourValue color_driveable_space = rviz_common::properties::qtToOgre(color_property_driveable_space_->getColor());
  Ogre::ColourValue color_lane_change = rviz_common::properties::qtToOgre(color_property_lane_change_->getColor());

  float scale_suggested_lane_reference_poses = scale_property_suggested_lane_reference_poses_->getFloat();
  float scale_suggested_lane_reference_line = scale_property_suggested_lane_reference_line_->getFloat();
  float scale_suggested_lane_boundary_points = scale_property_suggested_lane_boundary_points_->getFloat();
  float scale_suggested_lane_boundary_lines = scale_property_suggested_lane_boundary_lines_->getFloat();
  float scale_suggested_lane_regulatory_elements = scale_property_suggested_lane_regulatory_elements_->getFloat();
  float scale_adjacent_lanes_reference_poses = scale_property_adjacent_lanes_reference_poses_->getFloat();
  float scale_adjacent_lanes_reference_line = scale_property_adjacent_lanes_reference_line_->getFloat();
  float scale_adjacent_lanes_boundary_points = scale_property_adjacent_lanes_boundary_points_->getFloat();
  float scale_adjacent_lanes_boundary_lines = scale_property_adjacent_lanes_boundary_lines_->getFloat();
  float scale_adjacent_lane_regulatory_elements = scale_property_adjacent_lane_regulatory_elements_->getFloat();
  float scale_driveable_space = scale_property_driveable_space_->getFloat();
  float scale_lane_change = scale_property_lane_change_->getFloat();

  // loop over remaining route elements
  for (size_t i = 0; i < msg->remaining_route_elements.size(); ++i) {
    const auto& route_element = msg->remaining_route_elements[i];
    // display suggested lane reference poses
    if (show_suggested_lane_reference_poses) {
      const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
      suggested_lane_reference_poses_.push_back(generateRenderArrow(suggested_lane.reference_pose, color_suggested_lane_reference_poses, scale_suggested_lane_reference_poses));
    }

    // display suggested lane reference line
    if (show_suggested_lane_reference_line && (i < msg->remaining_route_elements.size() - 1)) {
      const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
      if (auto result = route_planning_msgs::route_access::getFollowingLaneElement(suggested_lane, msg->remaining_route_elements[i + 1])) {
        const auto& following_lane = *result;
        std::vector<geometry_msgs::msg::Point> points = {suggested_lane.reference_pose.position, following_lane.reference_pose.position};
        suggested_lane_reference_line_.push_back(generateRenderLine(points, color_suggested_lane_reference_line, scale_suggested_lane_reference_line));
      }
    }

    // display suggested lane boundary points
    if (show_suggested_lane_boundary_points) {
      const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
      if (suggested_lane.has_left_boundary) {
        suggested_lane_boundary_points_.push_back(generateRenderPoint(suggested_lane.left_boundary.point, color_suggested_lane_boundary_points, scale_suggested_lane_boundary_points));
      }
      if (suggested_lane.has_right_boundary) {
        suggested_lane_boundary_points_.push_back(generateRenderPoint(suggested_lane.right_boundary.point, color_suggested_lane_boundary_points, scale_suggested_lane_boundary_points));
      }
    }

    // display suggested lane boundary lines
    if (show_suggested_lane_boundary_lines && (i < msg->remaining_route_elements.size() - 1)) {
      const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
      if (auto result = route_planning_msgs::route_access::getFollowingLaneElement(suggested_lane, msg->remaining_route_elements[i + 1])) {
        const auto& following_lane = *result;
        if (suggested_lane.has_left_boundary && following_lane.has_left_boundary) {
          std::vector<geometry_msgs::msg::Point> points = {suggested_lane.left_boundary.point, following_lane.left_boundary.point};
          suggested_lane_boundary_lines_.push_back(generateRenderLine(points, color_suggested_lane_boundary_lines, scale_suggested_lane_boundary_lines));
        }
        if (suggested_lane.has_right_boundary && following_lane.has_right_boundary) {
          std::vector<geometry_msgs::msg::Point> points = {suggested_lane.right_boundary.point, following_lane.right_boundary.point};
          suggested_lane_boundary_lines_.push_back(generateRenderLine(points, color_suggested_lane_boundary_lines, scale_suggested_lane_boundary_lines));
        }
      }
    }

    // display suggested lane regulatory elements
    if (show_suggested_lane_regulatory_elements) {
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
        suggested_lane_regulatory_elements_.push_back(generateRenderLine(points, color_reg_elem, scale_suggested_lane_regulatory_elements));
        if (show_suggested_lane_regulatory_elements_sign_positions) {
          for (const auto& position : regulatory_element.positions) {
            suggested_lane_regulatory_elements_sign_positions_.push_back(generateRenderPoint(position, color_reg_elem, 0.5));
          }
        }
      }
    }

    // display adjacent lanes poses
    if (show_adjacent_lanes_reference_poses) {
      for (size_t i = 0; i < route_element.lane_elements.size(); ++i) {
        if (i != route_element.suggested_lane_idx) {
          const auto& adjacent_lane = route_element.lane_elements[i];
          adjacent_lanes_reference_poses_.push_back(generateRenderArrow(adjacent_lane.reference_pose, color_adjacent_lanes_reference_poses, scale_adjacent_lanes_reference_poses));
        }
      }
    }

    // display adjacent lanes reference line
    if (show_adjacent_lanes_reference_line && (i < msg->remaining_route_elements.size() - 1)) {
      for (size_t i = 0; i < route_element.lane_elements.size(); ++i) {
        if (i != route_element.suggested_lane_idx) {
          const auto& adjacent_lane = route_element.lane_elements[i];
          if (auto result = route_planning_msgs::route_access::getFollowingLaneElement(adjacent_lane, msg->remaining_route_elements[i + 1])) {
            const auto& following_lane = *result;
            std::vector<geometry_msgs::msg::Point> points = {adjacent_lane.reference_pose.position, following_lane.reference_pose.position};
            adjacent_lanes_reference_line_.push_back(generateRenderLine(points, color_adjacent_lanes_reference_line, scale_adjacent_lanes_reference_line));
          }
        }
      }
    }

    // display adjacent lanes boundary points
    if (show_adjacent_lanes_boundary_points) {
      for (size_t i = 0; i < route_element.lane_elements.size(); ++i) {
        if (i != route_element.suggested_lane_idx) {
          const auto& adjacent_lane = route_element.lane_elements[i];
          if (adjacent_lane.has_left_boundary) {
            adjacent_lanes_boundary_points_.push_back(generateRenderPoint(adjacent_lane.left_boundary.point, color_adjacent_lanes_boundary_points, scale_adjacent_lanes_boundary_points));
          }
          if (adjacent_lane.has_right_boundary) {
            adjacent_lanes_boundary_points_.push_back(generateRenderPoint(adjacent_lane.right_boundary.point, color_adjacent_lanes_boundary_points, scale_adjacent_lanes_boundary_points));
          }
        }
      }
    }

    // display adjacent lanes boundary lines
    if (show_adjacent_lanes_boundary_lines && (i < msg->remaining_route_elements.size() - 1)) {
      for (size_t i = 0; i < route_element.lane_elements.size(); ++i) {
        if (i != route_element.suggested_lane_idx) {
          const auto& adjacent_lane = route_element.lane_elements[i];
          if (auto result = route_planning_msgs::route_access::getFollowingLaneElement(adjacent_lane, msg->remaining_route_elements[i + 1])) {
            const auto& following_lane = *result;
            if (adjacent_lane.has_left_boundary && following_lane.has_left_boundary) {
              std::vector<geometry_msgs::msg::Point> points = {adjacent_lane.left_boundary.point, following_lane.left_boundary.point};
              adjacent_lanes_boundary_lines_.push_back(generateRenderLine(points, color_adjacent_lanes_boundary_lines, scale_adjacent_lanes_boundary_lines));
            }
            if (adjacent_lane.has_right_boundary && following_lane.has_right_boundary) {
              std::vector<geometry_msgs::msg::Point> points = {adjacent_lane.right_boundary.point, following_lane.right_boundary.point};
              adjacent_lanes_boundary_lines_.push_back(generateRenderLine(points, color_adjacent_lanes_boundary_lines, scale_adjacent_lanes_boundary_lines));
            }
          }
        }
      }
    }

    // display adjacent lane regulatory elements
    if (show_adjacent_lane_regulatory_elements) {
      for (size_t i = 0; i < route_element.lane_elements.size(); ++i) {
        if (i != route_element.suggested_lane_idx) {
          const auto& adjacent_lane = route_element.lane_elements[i];
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
            adjacent_lane_regulatory_elements_.push_back(generateRenderLine(points, color_reg_elem, scale_adjacent_lane_regulatory_elements));
            if (show_adjacent_lane_regulatory_elements_sign_positions) {
              for (const auto& position : regulatory_element.positions) {
                adjacent_lane_regulatory_elements_sign_positions_.push_back(generateRenderPoint(position, color_reg_elem, 0.5));
              }
            }
          }
        }
      }
    }

    // display driveable space points
    if (show_drivable_space) {
      drivable_space_points_.push_back(generateRenderPoint(route_element.left_boundary, color_driveable_space, scale_driveable_space));
      drivable_space_points_.push_back(generateRenderPoint(route_element.right_boundary, color_driveable_space, scale_driveable_space));
    }

    // display lane change lines
    if (show_lane_change && (i < msg->remaining_route_elements.size() - 1)) {
      if (route_element.will_change_suggested_lane) {
        const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
        const auto& next_suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(msg->remaining_route_elements[i + 1]);
        std::vector<geometry_msgs::msg::Point> points = {suggested_lane.reference_pose.position, next_suggested_lane.reference_pose.position};
        lane_change_lines_.push_back(generateRenderLine(points, color_lane_change, scale_lane_change));
      }
    }
  }

  // loop over traveled elements
  for (size_t i = 0; i < msg->traveled_route_elements.size(); ++i) {
    const auto& route_element = msg->traveled_route_elements[i];
    // display suggested lane reference poses
    if (show_suggested_lane_reference_poses) {
      const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
      suggested_lane_reference_poses_.push_back(generateRenderArrow(suggested_lane.reference_pose, color_suggested_lane_reference_poses, scale_suggested_lane_reference_poses));
    }

    // display suggested lane reference line
    if (show_suggested_lane_reference_line && (i < msg->traveled_route_elements.size() - 1)) {
      const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
      if (auto result = route_planning_msgs::route_access::getFollowingLaneElement(suggested_lane, msg->traveled_route_elements[i + 1])) {
        const auto& following_lane = *result;
        std::vector<geometry_msgs::msg::Point> points = {suggested_lane.reference_pose.position, following_lane.reference_pose.position};
        suggested_lane_reference_line_.push_back(generateRenderLine(points, color_suggested_lane_reference_line, scale_suggested_lane_reference_line));
      }
    }

    // display suggested lane boundary points
    if (show_suggested_lane_boundary_points) {
      const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
      if (suggested_lane.has_left_boundary) {
        suggested_lane_boundary_points_.push_back(generateRenderPoint(suggested_lane.left_boundary.point, color_suggested_lane_boundary_points, scale_suggested_lane_boundary_points));
      }
      if (suggested_lane.has_right_boundary) {
        suggested_lane_boundary_points_.push_back(generateRenderPoint(suggested_lane.right_boundary.point, color_suggested_lane_boundary_points, scale_suggested_lane_boundary_points));
      }
    }

    // display suggested lane boundary lines
    if (show_suggested_lane_boundary_lines && (i < msg->traveled_route_elements.size() - 1)) {
      const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
      if (auto result = route_planning_msgs::route_access::getFollowingLaneElement(suggested_lane, msg->traveled_route_elements[i + 1])) {
        const auto& following_lane = *result;
        if (suggested_lane.has_left_boundary && following_lane.has_left_boundary) {
          std::vector<geometry_msgs::msg::Point> points = {suggested_lane.left_boundary.point, following_lane.left_boundary.point};
          suggested_lane_boundary_lines_.push_back(generateRenderLine(points, color_suggested_lane_boundary_lines, scale_suggested_lane_boundary_lines));
        }
        if (suggested_lane.has_right_boundary && following_lane.has_right_boundary) {
          std::vector<geometry_msgs::msg::Point> points = {suggested_lane.right_boundary.point, following_lane.right_boundary.point};
          suggested_lane_boundary_lines_.push_back(generateRenderLine(points, color_suggested_lane_boundary_lines, scale_suggested_lane_boundary_lines));
        }
      }
    }

    // display suggested lane regulatory elements
    if (show_suggested_lane_regulatory_elements) {
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
        suggested_lane_regulatory_elements_.push_back(generateRenderLine(points, color_reg_elem, scale_suggested_lane_regulatory_elements));
        if (show_suggested_lane_regulatory_elements_sign_positions) {
          for (const auto& position : regulatory_element.positions) {
            suggested_lane_regulatory_elements_sign_positions_.push_back(generateRenderPoint(position, color_reg_elem, 0.5));
          }
        }
      }
    }

    // display adjacent lanes poses
    if (show_adjacent_lanes_reference_poses) {
      for (size_t i = 0; i < route_element.lane_elements.size(); ++i) {
        if (i != route_element.suggested_lane_idx) {
          const auto& adjacent_lane = route_element.lane_elements[i];
          adjacent_lanes_reference_poses_.push_back(generateRenderArrow(adjacent_lane.reference_pose, color_adjacent_lanes_reference_poses, scale_adjacent_lanes_reference_poses));
        }
      }
    }

    // display adjacent lanes reference line
    if (show_adjacent_lanes_reference_line && (i < msg->traveled_route_elements.size() - 1)) {
      for (size_t i = 0; i < route_element.lane_elements.size(); ++i) {
        if (i != route_element.suggested_lane_idx) {
          const auto& adjacent_lane = route_element.lane_elements[i];
          if (auto result = route_planning_msgs::route_access::getFollowingLaneElement(adjacent_lane, msg->traveled_route_elements[i + 1])) {
            const auto& following_lane = *result;
            std::vector<geometry_msgs::msg::Point> points = {adjacent_lane.reference_pose.position, following_lane.reference_pose.position};
            adjacent_lanes_reference_line_.push_back(generateRenderLine(points, color_adjacent_lanes_reference_line, scale_adjacent_lanes_reference_line));
          }
        }
      }
    }

    // display adjacent lanes boundary points
    if (show_adjacent_lanes_boundary_points) {
      for (size_t i = 0; i < route_element.lane_elements.size(); ++i) {
        if (i != route_element.suggested_lane_idx) {
          const auto& adjacent_lane = route_element.lane_elements[i];
          if (adjacent_lane.has_left_boundary) {
            adjacent_lanes_boundary_points_.push_back(generateRenderPoint(adjacent_lane.left_boundary.point, color_adjacent_lanes_boundary_points, scale_adjacent_lanes_boundary_points));
          }
          if (adjacent_lane.has_right_boundary) {
            adjacent_lanes_boundary_points_.push_back(generateRenderPoint(adjacent_lane.right_boundary.point, color_adjacent_lanes_boundary_points, scale_adjacent_lanes_boundary_points));
          }
        }
      }
    }

    // display adjacent lanes boundary lines
    if (show_adjacent_lanes_boundary_lines && (i < msg->traveled_route_elements.size() - 1)) {
      for (size_t i = 0; i < route_element.lane_elements.size(); ++i) {
        if (i != route_element.suggested_lane_idx) {
          const auto& adjacent_lane = route_element.lane_elements[i];
          if (auto result = route_planning_msgs::route_access::getFollowingLaneElement(adjacent_lane, msg->traveled_route_elements[i + 1])) {
            const auto& following_lane = *result;
            if (adjacent_lane.has_left_boundary && following_lane.has_left_boundary) {
              std::vector<geometry_msgs::msg::Point> points = {adjacent_lane.left_boundary.point, following_lane.left_boundary.point};
              adjacent_lanes_boundary_lines_.push_back(generateRenderLine(points, color_adjacent_lanes_boundary_lines, scale_adjacent_lanes_boundary_lines));
            }
            if (adjacent_lane.has_right_boundary && following_lane.has_right_boundary) {
              std::vector<geometry_msgs::msg::Point> points = {adjacent_lane.right_boundary.point, following_lane.right_boundary.point};
              adjacent_lanes_boundary_lines_.push_back(generateRenderLine(points, color_adjacent_lanes_boundary_lines, scale_adjacent_lanes_boundary_lines));
            }
          }
        }
      }
    }

    // display adjacent lane regulatory elements
    if (show_adjacent_lane_regulatory_elements) {
      for (size_t i = 0; i < route_element.lane_elements.size(); ++i) {
        if (i != route_element.suggested_lane_idx) {
          const auto& adjacent_lane = route_element.lane_elements[i];
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
            adjacent_lane_regulatory_elements_.push_back(generateRenderLine(points, color_reg_elem, scale_adjacent_lane_regulatory_elements));
            if (show_adjacent_lane_regulatory_elements_sign_positions) {
              for (const auto& position : regulatory_element.positions) {
                adjacent_lane_regulatory_elements_sign_positions_.push_back(generateRenderPoint(position, color_reg_elem, 0.5));
              }
            }
          }
        }
      }
    }

    // display driveable space points
    if (show_drivable_space) {
      drivable_space_points_.push_back(generateRenderPoint(route_element.left_boundary, color_driveable_space, scale_driveable_space));
      drivable_space_points_.push_back(generateRenderPoint(route_element.right_boundary, color_driveable_space, scale_driveable_space));
    }

    // display lane change lines
    if (show_lane_change && (i < msg->traveled_route_elements.size() - 1)) {
      if (route_element.will_change_suggested_lane) {
        const auto& suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
        const auto& next_suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(msg->traveled_route_elements[i + 1]);
        std::vector<geometry_msgs::msg::Point> points = {suggested_lane.reference_pose.position, next_suggested_lane.reference_pose.position};
        lane_change_lines_.push_back(generateRenderLine(points, color_lane_change, scale_lane_change));
      }
    }
  }
}

std::shared_ptr<rviz_rendering::Arrow> RouteDisplay::generateRenderArrow(const geometry_msgs::msg::Pose& pose, const Ogre::ColourValue& color, const float scale) {
  std::shared_ptr<rviz_rendering::Arrow> arrow = std::make_shared<rviz_rendering::Arrow>(scene_manager_, scene_node_);
  Ogre::Vector3 pos(pose.position.x, pose.position.y, pose.position.z);
  arrow->setPosition(pos);
  tf2::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Vector3 unit_vector(1, 0, 0);
  tf2::Vector3 direction = tf2::quatRotate(quat, unit_vector);
  Ogre::Vector3 dir(direction.getX(), direction.getY(), direction.getZ());
  arrow->setDirection(dir);
  arrow->setColor(color);
  arrow->setScale(Ogre::Vector3(scale, scale, scale));
  return arrow;
}

std::shared_ptr<rviz_rendering::BillboardLine> RouteDisplay::generateRenderLine(const std::vector<geometry_msgs::msg::Point>& points, const Ogre::ColourValue& color, const float scale) {
  std::shared_ptr<rviz_rendering::BillboardLine> billboard_line = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
  for (const auto& point : points) {
    Ogre::Vector3 pos(point.x, point.y, point.z);
    billboard_line->addPoint(pos);
  }
  billboard_line->setColor(color.r, color.g, color.b, color.a);
  billboard_line->setLineWidth(scale);
  billboard_line->finishLine();
  return billboard_line;
}

std::shared_ptr<rviz_rendering::Shape> RouteDisplay::generateRenderPoint(const geometry_msgs::msg::Point& point, const Ogre::ColourValue& color, const float scale) {
  std::shared_ptr<rviz_rendering::Shape> cube = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cube, scene_manager_, scene_node_);
  Ogre::Vector3 pos(point.x, point.y, point.z);
  cube->setPosition(pos);
  cube->setColor(color);
  cube->setScale(Ogre::Vector3(scale, scale, scale));
  return cube;
}

}  // namespace displays
}  // namespace route_planning_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(route_planning_msgs::displays::RouteDisplay, rviz_common::Display)