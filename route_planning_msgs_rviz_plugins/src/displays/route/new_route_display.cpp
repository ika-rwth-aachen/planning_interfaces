#include "route_planning_msgs/displays/route/new_route_display.hpp"

#include <rviz_common/logging.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_common/properties/parse_color.hpp>

namespace route_planning_msgs {
namespace displays {

void NewRouteDisplay::onInitialize() {
  MFDClass::onInitialize();

  // destination
  viz_destination_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Destination", true, "Whether to display the destination arrow.", this, SLOT(updateStyle()));
  color_property_destination_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 0, 255), "Color to draw the destination arrow.", viz_destination_.get(), SLOT(updateStyle()));
  scale_property_destination_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.1, "Scale of the destination arrow.", viz_destination_.get(), SLOT(updateStyle()));

  // suggested lane
  viz_suggested_lane_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Suggested Lane", true, "Whether to display the suggested lane.", this, SLOT(updateStyle()));
  viz_suggested_lane_reference_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Reference Line", true, "Whether to display the reference line of the suggested lane.", viz_suggested_lane_.get(), SLOT(updateStyle()));
  viz_suggested_lane_boundaries_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Boundaries", true, "Whether to display the reference boundaries of the suggested lane.", viz_suggested_lane_.get(), SLOT(updateStyle()));

  viz_suggested_lane_reference_poses_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Poses", true, "Whether to display the reference poses of the suggested lane.", viz_suggested_lane_reference_.get(), SLOT(updateStyle()));
  color_property_suggested_lane_reference_poses_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(36, 64, 142), "Color to draw reference poses of the suggested lane.", viz_suggested_lane_reference_poses_.get(), SLOT(updateStyle()));
  scale_property_suggested_lane_reference_poses_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 1.0, "Scale of the reference poses of the suggested lane.", viz_suggested_lane_reference_poses_.get(), SLOT(updateStyle()));

  viz_suggested_lane_reference_line_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Line", true, "Whether to display the reference line of the suggested lane.", viz_suggested_lane_reference_.get(), SLOT(updateStyle()));
  color_property_suggested_lane_reference_line_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(36, 64, 142), "Color to draw reference line of the suggested lane.", viz_suggested_lane_reference_line_.get(), SLOT(updateStyle()));
  scale_property_suggested_lane_reference_line_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 1.0, "Scale of the reference line of the suggested lane.", viz_suggested_lane_reference_line_.get(), SLOT(updateStyle()));

  viz_suggested_lane_boundary_points_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Points", true, "Whether to display the reference and lane boundary points of the suggested lane.", viz_suggested_lane_boundaries_.get(), SLOT(updateStyle()));
  color_property_suggested_lane_boundary_points_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(0, 0, 255), "Color to draw reference and lane boundary points of the suggested lane.", viz_suggested_lane_boundary_points_.get(), SLOT(updateStyle()));
  scale_property_suggested_lane_boundary_points_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.1, "Scale of the reference and lane boundary points of the suggested lane.", viz_suggested_lane_boundary_points_.get(), SLOT(updateStyle()));

  viz_suggested_lane_boundary_lines_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Lines", true, "Whether to display the lane boundary lines of the suggested lane.", viz_suggested_lane_boundaries_.get(), SLOT(updateStyle()));
  color_property_suggested_lane_boundary_lines_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(0, 0, 255), "Color to draw lane boundary lines of the suggested lane.", viz_suggested_lane_boundary_lines_.get(), SLOT(updateStyle()));
  scale_property_suggested_lane_boundary_lines_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 1.0, "Scale of the lane boundary lines of the suggested lane.", viz_suggested_lane_boundary_lines_.get(), SLOT(updateStyle()));

  // adjacent lanes
  viz_adjacent_lanes_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Adjacent Lanes", true, "Whether to display the reference and lane boundary points of adjacent lanes.", this, SLOT(updateStyle()));
  viz_adjacent_lanes_reference_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Reference Line", true, "Whether to display the reference line of adjacent lanes.", viz_adjacent_lanes_.get(), SLOT(updateStyle()));
  viz_adjacent_lanes_boundaries_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Boundaries", true, "Whether to display the reference boundaries of adjacent lanes.", viz_adjacent_lanes_.get(), SLOT(updateStyle()));

  viz_adjacent_lanes_reference_poses_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Poses", true, "Whether to display the reference poses of adjacent lanes.", viz_adjacent_lanes_reference_.get(), SLOT(updateStyle()));
  color_property_adjacent_lanes_reference_poses_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 0, 0), "Color to draw reference poses of adjacent lanes.", viz_adjacent_lanes_reference_poses_.get(), SLOT(updateStyle()));
  scale_property_adjacent_lanes_reference_poses_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 1.0, "Scale of the reference poses of adjacent lanes.", viz_adjacent_lanes_reference_poses_.get(), SLOT(updateStyle()));
  
  viz_adjacent_lanes_reference_line_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Line", true, "Whether to display the reference line of adjacent lanes.", viz_adjacent_lanes_reference_.get(), SLOT(updateStyle()));
  color_property_adjacent_lanes_reference_line_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 0, 0), "Color to draw reference line of adjacent lanes.", viz_adjacent_lanes_reference_line_.get(), SLOT(updateStyle()));
  scale_property_adjacent_lanes_reference_line_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 1.0, "Scale of the reference line of adjacent lanes.", viz_adjacent_lanes_reference_line_.get(), SLOT(updateStyle()));

  viz_adjacent_lanes_boundary_points_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Points", true, "Whether to display the reference and lane boundary points of adjacent lanes.", viz_adjacent_lanes_boundaries_.get(), SLOT(updateStyle()));
  color_property_adjacent_lanes_boundary_points_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 0, 0), "Color to draw reference and lane boundary points of adjacent lanes.", viz_adjacent_lanes_boundary_points_.get(), SLOT(updateStyle()));
  scale_property_adjacent_lanes_boundary_points_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.1, "Scale of the reference and lane boundary points of adjacent lanes.", viz_adjacent_lanes_boundary_points_.get(), SLOT(updateStyle()));

  viz_adjacent_lanes_boundary_lines_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Lines", true, "Whether to display the lane boundary lines of adjacent lanes.", viz_adjacent_lanes_boundaries_.get(), SLOT(updateStyle()));
  color_property_adjacent_lanes_boundary_lines_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 0, 0), "Color to draw lane boundary lines of adjacent lanes.", viz_adjacent_lanes_boundary_lines_.get(), SLOT(updateStyle()));
  scale_property_adjacent_lanes_boundary_lines_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 1.0, "Scale of the lane boundary lines of adjacent lanes.", viz_adjacent_lanes_boundary_lines_.get(), SLOT(updateStyle()));

  // driveable space
  viz_driveable_space_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Driveable Space", true, "Whether to display the reference and lane boundary points of the driveable space.", this, SLOT(updateStyle()));
  color_property_driveable_space_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(0, 255, 0), "Color to draw reference and lane boundary points of the driveable space.", viz_driveable_space_.get(), SLOT(updateStyle()));
  scale_property_driveable_space_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 0.1, "Scale of the reference and lane boundary points of the driveable space.", viz_driveable_space_.get(), SLOT(updateStyle()));  

  // lane change
  viz_lane_change_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Lane Change", true, "Whether to display the lane change lines.", this, SLOT(updateStyle()));
  color_property_lane_change_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Color", QColor(255, 255, 0), "Color to draw lane change lines.", viz_lane_change_.get(), SLOT(updateStyle()));
  scale_property_lane_change_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Scale", 1.0, "Scale of the lane change lines.", viz_lane_change_.get(), SLOT(updateStyle()));

  updateStyle();
}

void NewRouteDisplay::reset() {
  MFDClass::reset();
  destination_arrow_.reset();
  suggested_lane_reference_poses_.clear();
  suggested_lane_reference_line_.clear();
  suggested_lane_boundary_points_.clear();
  suggested_lane_boundary_lines_.clear();
  adjacent_lanes_reference_poses_.clear();
  adjacent_lanes_reference_line_.clear();
  adjacent_lanes_boundary_points_.clear();
  adjacent_lanes_boundary_lines_.clear();
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
  
  // reset destination
  destination_arrow_.reset();
  // display destination
  if (viz_destination_->getBool()) {
    geometry_msgs::msg::Pose destination;
    destination.position = msg->destination;
    Ogre::ColourValue destination_color = rviz_common::properties::qtToOgre(color_property_destination_->getColor());
    float destination_scale = scale_property_destination_->getFloat();
    destination_arrow_ = generateRenderArrow(destination, destination_color, destination_scale);
  }

  // clear previous points
  suggested_lane_reference_poses_.clear();
  suggested_lane_reference_line_.clear();
  suggested_lane_boundary_points_.clear();
  suggested_lane_boundary_lines_.clear();
  adjacent_lanes_reference_poses_.clear();
  adjacent_lanes_reference_line_.clear();
  adjacent_lanes_boundary_points_.clear();
  adjacent_lanes_boundary_lines_.clear();
  drivable_space_points_.clear();
  lane_change_lines_.clear();

  // Get visualization settings once
  bool viz_suggested_lane_reference = viz_suggested_lane_->getBool() && viz_suggested_lane_reference_->getBool();
  bool viz_suggested_lane_boundaries = viz_suggested_lane_->getBool() && viz_suggested_lane_boundaries_->getBool();
  bool viz_adjacent_lanes_reference = viz_adjacent_lanes_->getBool() && viz_adjacent_lanes_reference_ ->getBool();
  bool viz_adjacent_lanes_boundaries = viz_adjacent_lanes_->getBool() && viz_adjacent_lanes_boundaries_->getBool();

  bool show_suggested_lane_reference_poses = viz_suggested_lane_reference && viz_suggested_lane_reference_poses_->getBool();
  bool show_suggested_lane_reference_line = viz_suggested_lane_reference && viz_suggested_lane_reference_line_->getBool();
  bool show_suggested_lane_boundary_points = viz_suggested_lane_boundaries && viz_suggested_lane_boundary_points_->getBool();
  bool show_suggested_lane_boundary_lines = viz_suggested_lane_boundaries && viz_suggested_lane_boundary_lines_->getBool();
  bool show_adjacent_lanes_reference_poses = viz_adjacent_lanes_reference && viz_adjacent_lanes_reference_poses_->getBool();
  bool show_adjacent_lanes_reference_line = viz_adjacent_lanes_reference && viz_adjacent_lanes_reference_line_->getBool();
  bool show_adjacent_lanes_boundary_points = viz_adjacent_lanes_boundaries && viz_adjacent_lanes_boundary_points_->getBool();
  bool show_adjacent_lanes_boundary_lines = viz_adjacent_lanes_boundaries && viz_adjacent_lanes_boundary_lines_->getBool();
  bool show_drivable_space = viz_driveable_space_->getBool();
  bool show_lane_change = viz_lane_change_->getBool();

  Ogre::ColourValue color_suggested_lane_reference_poses = rviz_common::properties::qtToOgre(color_property_suggested_lane_reference_poses_->getColor());
  Ogre::ColourValue color_suggested_lane_reference_line = rviz_common::properties::qtToOgre(color_property_suggested_lane_reference_line_->getColor());
  Ogre::ColourValue color_suggested_lane_boundary_points = rviz_common::properties::qtToOgre(color_property_suggested_lane_boundary_points_->getColor());
  Ogre::ColourValue color_suggested_lane_boundary_lines = rviz_common::properties::qtToOgre(color_property_suggested_lane_boundary_lines_->getColor());
  Ogre::ColourValue color_adjacent_lanes_reference_poses = rviz_common::properties::qtToOgre(color_property_adjacent_lanes_reference_poses_->getColor());
  Ogre::ColourValue color_adjacent_lanes_reference_line = rviz_common::properties::qtToOgre(color_property_adjacent_lanes_reference_line_->getColor());
  Ogre::ColourValue color_adjacent_lanes_boundary_points = rviz_common::properties::qtToOgre(color_property_adjacent_lanes_boundary_points_->getColor());
  Ogre::ColourValue color_adjacent_lanes_boundary_lines = rviz_common::properties::qtToOgre(color_property_adjacent_lanes_boundary_lines_->getColor());
  Ogre::ColourValue color_driveable_space = rviz_common::properties::qtToOgre(color_property_driveable_space_->getColor());
  Ogre::ColourValue color_lane_change = rviz_common::properties::qtToOgre(color_property_lane_change_->getColor());

  float scale_suggested_lane_reference_poses = scale_property_suggested_lane_reference_poses_->getFloat();
  float scale_suggested_lane_reference_line = scale_property_suggested_lane_reference_line_->getFloat();
  float scale_suggested_lane_boundary_points = scale_property_suggested_lane_boundary_points_->getFloat();
  float scale_suggested_lane_boundary_lines = scale_property_suggested_lane_boundary_lines_->getFloat();
  float scale_adjacent_lanes_reference_poses = scale_property_adjacent_lanes_reference_poses_->getFloat();
  float scale_adjacent_lanes_reference_line = scale_property_adjacent_lanes_reference_line_->getFloat();
  float scale_adjacent_lanes_boundary_points = scale_property_adjacent_lanes_boundary_points_->getFloat();
  float scale_adjacent_lanes_boundary_lines = scale_property_adjacent_lanes_boundary_lines_->getFloat();
  float scale_driveable_space = scale_property_driveable_space_->getFloat();
  float scale_lane_change = scale_property_lane_change_->getFloat();

  // loop over route elements
  for (size_t i = 0; i < msg->remaining_route_elements.size(); ++i) {
    const auto& route_element = msg->remaining_route_elements[i];
    // display suggested lane reference poses
    if (show_suggested_lane_reference_poses) {
      const auto& suggested_lane = route_planning_msgs::route_access::getCurrentLaneElement(route_element);
      suggested_lane_reference_poses_.push_back(generateRenderArrow(suggested_lane.reference_pose, color_suggested_lane_reference_poses, scale_suggested_lane_reference_poses));
    }

    // display suggested lane reference line
    if (show_suggested_lane_reference_line && (i < msg->remaining_route_elements.size() - 1)) {
      const auto& suggested_lane = route_planning_msgs::route_access::getCurrentLaneElement(route_element);
      const auto& next_suggested_lane = route_planning_msgs::route_access::getCurrentLaneElement(msg->remaining_route_elements[i + 1]);
      suggested_lane_reference_line_.push_back(generateRenderLine(suggested_lane.reference_pose.position, next_suggested_lane.reference_pose.position, color_suggested_lane_reference_line, scale_suggested_lane_reference_line));
    }

    // display suggested lane boundary points
    if (show_suggested_lane_boundary_points) {
      const auto& suggested_lane = route_planning_msgs::route_access::getCurrentLaneElement(route_element);
      if (suggested_lane.has_left_boundary) {
        suggested_lane_boundary_points_.push_back(generateRenderPoint(suggested_lane.left_boundary.point, color_suggested_lane_boundary_points, scale_suggested_lane_boundary_points));
      }
      if (suggested_lane.has_right_boundary) {
        suggested_lane_boundary_points_.push_back(generateRenderPoint(suggested_lane.right_boundary.point, color_suggested_lane_boundary_points, scale_suggested_lane_boundary_points));
      }
    }
    
    // display suggested lane boundary lines
    if (show_suggested_lane_boundary_lines && (i < msg->remaining_route_elements.size() - 1)) {
      const auto& suggested_lane = route_planning_msgs::route_access::getCurrentLaneElement(route_element);
      const auto& next_suggested_lane = route_planning_msgs::route_access::getCurrentLaneElement(msg->remaining_route_elements[i + 1]);
      if (suggested_lane.has_left_boundary && next_suggested_lane.has_left_boundary) {
        suggested_lane_boundary_lines_.push_back(generateRenderLine(suggested_lane.left_boundary.point, next_suggested_lane.left_boundary.point, color_suggested_lane_boundary_lines, scale_suggested_lane_boundary_lines));
      }
      if (suggested_lane.has_right_boundary && next_suggested_lane.has_right_boundary) {
        suggested_lane_boundary_lines_.push_back(generateRenderLine(suggested_lane.right_boundary.point, next_suggested_lane.right_boundary.point, color_suggested_lane_boundary_lines, scale_suggested_lane_boundary_lines));
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
          const auto& next_adjacent_lane = msg->remaining_route_elements[i + 1].lane_elements[i];
          adjacent_lanes_reference_line_.push_back(generateRenderLine(adjacent_lane.reference_pose.position, next_adjacent_lane.reference_pose.position, color_adjacent_lanes_reference_line, scale_adjacent_lanes_reference_line));
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
          const auto& next_adjacent_lane = msg->remaining_route_elements[i + 1].lane_elements[i];
          if (adjacent_lane.has_left_boundary && next_adjacent_lane.has_left_boundary) {
            adjacent_lanes_boundary_lines_.push_back(generateRenderLine(adjacent_lane.left_boundary.point, next_adjacent_lane.left_boundary.point, color_adjacent_lanes_boundary_lines, scale_adjacent_lanes_boundary_lines));
          }
          if (adjacent_lane.has_right_boundary && next_adjacent_lane.has_right_boundary) {
            adjacent_lanes_boundary_lines_.push_back(generateRenderLine(adjacent_lane.right_boundary.point, next_adjacent_lane.right_boundary.point, color_adjacent_lanes_boundary_lines, scale_adjacent_lanes_boundary_lines));
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
        const auto& suggested_lane = route_planning_msgs::route_access::getCurrentLaneElement(route_element);
        const auto& next_suggested_lane = route_planning_msgs::route_access::getCurrentLaneElement(msg->remaining_route_elements[i + 1]);
        lane_change_lines_.push_back(generateRenderLine(suggested_lane.reference_pose.position, next_suggested_lane.reference_pose.position, color_lane_change, scale_lane_change));
      }
    }
  }
}

std::shared_ptr<rviz_rendering::Arrow> NewRouteDisplay::generateRenderArrow(const geometry_msgs::msg::Pose& pose, const Ogre::ColourValue& color, const float scale) {
  std::shared_ptr<rviz_rendering::Arrow> arrow = std::make_shared<rviz_rendering::Arrow>(scene_manager_, scene_node_);
  Ogre::Vector3 pos(pose.position.x, pose.position.y, pose.position.z);
  Ogre::Quaternion orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  arrow->setPosition(pos);
  arrow->setOrientation(orientation);
  arrow->setColor(color);
  arrow->setScale(Ogre::Vector3(scale, scale, scale));
  return arrow;
}

std::shared_ptr<rviz_rendering::Line> NewRouteDisplay::generateRenderLine(const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& end, const Ogre::ColourValue& color, const float scale) {
  std::shared_ptr<rviz_rendering::Line> line = std::make_shared<rviz_rendering::Line>(scene_manager_, scene_node_);
  Ogre::Vector3 start_pos(start.x, start.y, start.z);
  Ogre::Vector3 end_pos(end.x, end.y, end.z);
  line->setPoints(start_pos, end_pos);
  line->setColor(color);
  line->setScale(Ogre::Vector3(scale, scale, scale));
  return line;
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
  // suggested lane reference poses
  if (viz_suggested_lane_->getBool() && viz_suggested_lane_reference_->getBool() && viz_suggested_lane_reference_poses_->getBool()) {
    Ogre::ColourValue color_suggested_lane_reference_poses = rviz_common::properties::qtToOgre(color_property_suggested_lane_reference_poses_->getColor());
    float scale_suggested_lane_reference_poses = scale_property_suggested_lane_reference_poses_->getFloat();
    for (auto& pose : suggested_lane_reference_poses_) {
      pose->setColor(color_suggested_lane_reference_poses);
      pose->setScale(Ogre::Vector3(scale_suggested_lane_reference_poses, scale_suggested_lane_reference_poses, scale_suggested_lane_reference_poses));
    }
  }

  // suggested lane boundary points
  if (viz_suggested_lane_->getBool() && viz_suggested_lane_boundaries_->getBool() && viz_suggested_lane_boundary_points_->getBool()) {
    Ogre::ColourValue color_suggested_lane_boundary_points = rviz_common::properties::qtToOgre(color_property_suggested_lane_boundary_points_->getColor());
    float scale_suggested_lane_boundary_points = scale_property_suggested_lane_boundary_points_->getFloat();
    for (auto& point : suggested_lane_boundary_points_) {
      point->setColor(color_suggested_lane_boundary_points);
      point->setScale(Ogre::Vector3(scale_suggested_lane_boundary_points, scale_suggested_lane_boundary_points, scale_suggested_lane_boundary_points));
    }
  }

  // adjacent lanes reference poses
  if (viz_adjacent_lanes_->getBool() && viz_adjacent_lanes_reference_->getBool() && viz_adjacent_lanes_reference_poses_->getBool()) {
    Ogre::ColourValue color_adjacent_lanes_reference_poses = rviz_common::properties::qtToOgre(color_property_adjacent_lanes_reference_poses_->getColor());
    float scale_adjacent_lanes_reference_poses = scale_property_adjacent_lanes_reference_poses_->getFloat();
    for (auto& pose : adjacent_lanes_reference_poses_) {
      pose->setColor(color_adjacent_lanes_reference_poses);
      pose->setScale(Ogre::Vector3(scale_adjacent_lanes_reference_poses, scale_adjacent_lanes_reference_poses, scale_adjacent_lanes_reference_poses));
    }
  }

  // adjacent lanes boundary points
  if (viz_adjacent_lanes_->getBool() && viz_adjacent_lanes_boundaries_->getBool() && viz_adjacent_lanes_boundary_points_->getBool()) {
    Ogre::ColourValue color_adjacent_lanes_boundary_points = rviz_common::properties::qtToOgre(color_property_adjacent_lanes_boundary_points_->getColor());
    float scale_adjacent_lanes_boundary_points = scale_property_adjacent_lanes_boundary_points_->getFloat();
    for (auto& point : adjacent_lanes_boundary_points_) {
      point->setColor(color_adjacent_lanes_boundary_points);
      point->setScale(Ogre::Vector3(scale_adjacent_lanes_boundary_points, scale_adjacent_lanes_boundary_points, scale_adjacent_lanes_boundary_points));
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