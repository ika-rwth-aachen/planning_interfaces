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

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_rendering/material_manager.hpp"

namespace route_planning_msgs {
namespace displays {

RouteDisplay::RouteDisplay() {
  color_property_target_ = new rviz_common::properties::ColorProperty(
      "Color Target", QColor(255, 0, 255), "Color to draw the target position.", this, SLOT(queueRender()));
  viz_sp_centerline_ =
      new rviz_common::properties::BoolProperty("Enable Route", true, "Visualize the shortes-path centerline.", this);
  color_property_traveled_route_ = new rviz_common::properties::ColorProperty(
      "Color Traveled Route", QColor(0, 150, 150), "Color to draw the traveled route.", viz_sp_centerline_,
      SLOT(queueRender()));
  color_property_remaining_route_ = new rviz_common::properties::ColorProperty(
      "Color Remaining Route", QColor(0, 255, 255), "Color to draw the traveled route.", viz_sp_centerline_,
      SLOT(queueRender()));
  viz_route_boundaries_ = new rviz_common::properties::BoolProperty("Enable Route Boundaries", true,
                                                                    "Visualize the route boundaries.", this);
  color_property_boundaries_ = new rviz_common::properties::ColorProperty("Color Route Boundaries", QColor(255, 255, 0),
                                                                          "Color to draw the boundaries of the route.",
                                                                          viz_route_boundaries_, SLOT(queueRender()));
  viz_driveable_space_ = new rviz_common::properties::BoolProperty("Enable Driveable Space", true,
                                                                   "Visualize the driveable-space boundaries.", this);
  color_property_driveable_space_ = new rviz_common::properties::ColorProperty(
      "Color Driveable-Space Boundaries", QColor(255, 165, 0), "Color to draw the boundaries of the driveable space.",
      viz_driveable_space_, SLOT(queueRender()));
  viz_lanes_ =
      new rviz_common::properties::BoolProperty("Enable Lanes", true, "Visualize lanes of the road-network.", this);
  viz_lane_separators_ = new rviz_common::properties::BoolProperty(
      "Enable Lane Separators", true, "Visualize separators of different lanes within the road-network.", viz_lanes_);
  viz_lane_centerline_ = new rviz_common::properties::BoolProperty(
      "Enable Lane Centerline", true, "Visualize centerline of lane within the road-network.", viz_lanes_);
  alpha_property_lane_ = new rviz_common::properties::FloatProperty(
      "Alpha Lane Separators", 1.0f, "Amount of transparency to apply to the lane separators.", viz_lanes_,
      SLOT(queueRender()));
  color_property_separators_allowed_ = new rviz_common::properties::ColorProperty(
      "Color Unrestricting Lane Separators", QColor(25, 255, 0),
      "Color to draw the lane separators that allow crossing.", viz_lane_separators_, SLOT(queueRender()));
  color_property_separators_restricted_ = new rviz_common::properties::ColorProperty(
      "Color Restricting Lane Separators", QColor(255, 0, 0),
      "Color to draw the lane separators that restrict crossing.", viz_lane_separators_, SLOT(queueRender()));
  color_property_lane_centerlines_ = new rviz_common::properties::ColorProperty(
      "Color Lane Centerlines", QColor(238, 130, 238), "Color to draw the lane centerlines.", viz_lane_centerline_,
      SLOT(queueRender()));
  viz_regelems_ = new rviz_common::properties::BoolProperty("Enable Regulatory Elements", true,
                                                            "Visualize regulatory elements.", this);
  color_property_regelems_ = new rviz_common::properties::ColorProperty(
      "Color Regulatory Elements", QColor(0, 255, 255), "Color to draw the regulatory elements.", viz_regelems_,
      SLOT(queueRender()));
  viz_cur_speed_limit_ = new rviz_common::properties::BoolProperty(
      "Enable Speed Limit Text", true, "Visualize the text indicating the current speed limit.", this);
  alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 1.0f, "Amount of transparency to apply.", this,
                                                               SLOT(queueRender()));

  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
  alpha_property_lane_->setMin(0);
  alpha_property_lane_->setMax(1);

  static int ds_count = 0;
  std::string material_name = "RouteMaterial" + std::to_string(ds_count++);
  material_traveled_route_ =
      rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name + "_traveled_route");
  material_remaining_route_ =
      rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name + "_remaining_route");
  material_boundaries_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name + "_boundaries");
  material_driveable_space_ =
      rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name + "_driveable_space");
  material_separators_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name + "_separators");
  material_regelems_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name + "_regelems");
}

RouteDisplay::~RouteDisplay() {
  if (initialized()) {
    delete target_arrow_;
    delete cur_speed_text_;
    scene_manager_->destroyManualObject(manual_object_);
  }
}

void RouteDisplay::onInitialize() {
  MFDClass::onInitialize();

  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);
  // Target Position Arrow
  Ogre::ColourValue invisible;
  invisible.a = 0.0;
  target_arrow_ =
      new rviz_rendering::Arrow(scene_manager_, scene_node_, target_arrow_shaft_length_, target_arrow_shaft_diameter_,
                                target_arrow_head_length_, target_arrow_head_diameter_);
  target_arrow_->setColor(invisible);
  // Current Speed Text
  cur_speed_text_ = new rviz_rendering::MovableText("Current Speed Limit not set!");
  cur_speed_text_->setColor(invisible);
  cur_speed_text_->setTextAlignment(rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);
  scene_node_->attachObject(cur_speed_text_);
}

void RouteDisplay::reset() {
  MFDClass::reset();
  Ogre::ColourValue invisible;
  invisible.a = 0.0;
  regelem_spheres_.clear();
  target_arrow_->setColor(invisible);
  cur_speed_text_->setColor(invisible);
  manual_object_->clear();
}

bool validateFloats(route_planning_msgs::msg::Route::ConstSharedPtr msg) {
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg->destination);
  valid = valid && rviz_common::validateFloats(msg->boundaries.left);
  valid = valid && rviz_common::validateFloats(msg->boundaries.right);
  valid = valid && rviz_common::validateFloats(msg->driveable_space.boundaries.left);
  valid = valid && rviz_common::validateFloats(msg->driveable_space.boundaries.right);
  valid = valid && rviz_common::validateFloats(msg->traveled_route);
  valid = valid && rviz_common::validateFloats(msg->remaining_route);
  for (size_t i = 0; i < msg->lanes.size(); ++i) {
    valid = valid && rviz_common::validateFloats(msg->lanes[i].left.line);
    valid = valid && rviz_common::validateFloats(msg->lanes[i].right.line);
  }
  for (size_t i = 0; i < msg->driveable_space.restricted_areas.size(); ++i) {
    valid = valid && rviz_common::validateFloats(msg->driveable_space.restricted_areas[i].points);
  }
  for (size_t i = 0; i < msg->regulatory_elements.size(); ++i) {
    valid = valid && rviz_common::validateFloats(msg->regulatory_elements[i].effect_line);
    for (size_t j = 0; j < msg->regulatory_elements[i].signal_positions.size(); ++j) {
      valid = valid && rviz_common::validateFloats(msg->regulatory_elements[i].signal_positions[j]);
    }
  }
  return valid;
}

void RouteDisplay::processMessage(route_planning_msgs::msg::Route::ConstSharedPtr msg) {
  if (!validateFloats(msg)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
              "Message contained invalid floating point values (nans or infs)");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }
  setTransformOk();

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  manual_object_->clear();

  Ogre::ColourValue color_target = rviz_common::properties::qtToOgre(color_property_target_->getColor());
  Ogre::ColourValue color_regelems = rviz_common::properties::qtToOgre(color_property_regelems_->getColor());
  Ogre::ColourValue color_grey_regelems;
  color_grey_regelems.r = 128.0;
  color_grey_regelems.g = 128.0;
  color_grey_regelems.b = 128.0;
  color_grey_regelems.a = color_regelems.a;

  color_target.a = alpha_property_->getFloat();
  color_regelems.a = alpha_property_->getFloat();

  rviz_rendering::MaterialManager::enableAlphaBlending(material_regelems_, color_regelems.a);

  // Target Position
  target_arrow_->setColor(color_target);
  Ogre::Vector3 base_pos(msg->destination.x, msg->destination.y,
                         msg->destination.z + target_arrow_shaft_length_ + target_arrow_head_length_);
  target_arrow_->setPosition(base_pos);

  // traveled/remaining route (centerline)
  if (viz_sp_centerline_->getBool()) {
    Ogre::ColourValue color_traveled_route =
        rviz_common::properties::qtToOgre(color_property_traveled_route_->getColor());
    Ogre::ColourValue color_remaining_route =
        rviz_common::properties::qtToOgre(color_property_remaining_route_->getColor());
    color_traveled_route.a = alpha_property_->getFloat();
    color_remaining_route.a = alpha_property_->getFloat();
    rviz_rendering::MaterialManager::enableAlphaBlending(material_traveled_route_, color_traveled_route.a);
    rviz_rendering::MaterialManager::enableAlphaBlending(material_remaining_route_, color_remaining_route.a);
    if (!msg->traveled_route.empty()) {
      manual_object_->estimateVertexCount(msg->traveled_route.size());
      manual_object_->begin(material_traveled_route_->getName(), Ogre::RenderOperation::OT_LINE_STRIP,
                            "rviz_rendering");
      for (const auto& point : msg->traveled_route) {
        manual_object_->position(point.x, point.y, 0.0);  // z is s
        manual_object_->colour(color_traveled_route);
      }
      manual_object_->end();
    }
    if (!msg->remaining_route.empty()) {
      manual_object_->estimateVertexCount(msg->remaining_route.size());
      manual_object_->begin(material_remaining_route_->getName(), Ogre::RenderOperation::OT_LINE_STRIP,
                            "rviz_rendering");
      for (const auto& point : msg->remaining_route) {
        manual_object_->position(point.x, point.y, 0.0);  // z is s
        manual_object_->colour(color_remaining_route);
      }
      manual_object_->end();
    }
  }

  if (viz_route_boundaries_->getBool()) {
    Ogre::ColourValue color_boundaries = rviz_common::properties::qtToOgre(color_property_boundaries_->getColor());
    color_boundaries.a = alpha_property_->getFloat();
    rviz_rendering::MaterialManager::enableAlphaBlending(material_boundaries_, color_boundaries.a);

    // Boundaries Left
    size_t num_points = msg->boundaries.left.size();
    if (num_points > 0) {
      manual_object_->estimateVertexCount(num_points);
      manual_object_->begin(material_boundaries_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
      for (uint32_t i = 0; i < num_points; i++) {
        const geometry_msgs::msg::Point& msg_point = msg->boundaries.left[i];
        manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
        manual_object_->colour(color_boundaries);
      }
      manual_object_->end();
    }

    // Boundaries right
    num_points = msg->boundaries.right.size();
    if (num_points > 0) {
      manual_object_->estimateVertexCount(num_points);
      manual_object_->begin(material_boundaries_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
      for (uint32_t i = 0; i < num_points; i++) {
        const geometry_msgs::msg::Point& msg_point = msg->boundaries.right[i];
        manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
        manual_object_->colour(color_boundaries);
      }
      manual_object_->end();
    }
  }

  if (viz_driveable_space_->getBool()) {
    Ogre::ColourValue color_ds = rviz_common::properties::qtToOgre(color_property_driveable_space_->getColor());
    color_ds.a = alpha_property_->getFloat();
    rviz_rendering::MaterialManager::enableAlphaBlending(material_driveable_space_, color_ds.a);

    // DS Boundaries Left
    size_t num_points = msg->driveable_space.boundaries.left.size();
    if (num_points > 0) {
      manual_object_->estimateVertexCount(num_points);
      manual_object_->begin(material_driveable_space_->getName(), Ogre::RenderOperation::OT_LINE_STRIP,
                            "rviz_rendering");
      for (uint32_t i = 0; i < num_points; i++) {
        const geometry_msgs::msg::Point& msg_point = msg->driveable_space.boundaries.left[i % num_points];
        manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
        manual_object_->colour(color_ds);
      }
      manual_object_->end();
    }

    // DS Boundaries right
    num_points = msg->driveable_space.boundaries.right.size();
    if (num_points > 0) {
      manual_object_->estimateVertexCount(num_points);
      manual_object_->begin(material_driveable_space_->getName(), Ogre::RenderOperation::OT_LINE_STRIP,
                            "rviz_rendering");
      for (uint32_t i = 0; i < num_points; i++) {
        const geometry_msgs::msg::Point& msg_point = msg->driveable_space.boundaries.right[i % num_points];
        manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
        manual_object_->colour(color_ds);
      }
      manual_object_->end();
    }

    // Restricted areas
    if (msg->driveable_space.restricted_areas.size() > 0) {
      for (size_t j = 0; j < msg->driveable_space.restricted_areas.size(); j++) {
        num_points = msg->driveable_space.restricted_areas[j].points.size();
        manual_object_->estimateVertexCount(num_points);
        manual_object_->begin(material_driveable_space_->getName(), Ogre::RenderOperation::OT_LINE_STRIP,
                              "rviz_rendering");
        for (uint32_t i = 0; i < num_points + 1; ++i) {
          const geometry_msgs::msg::Point32& msg_point =
              msg->driveable_space.restricted_areas[j].points[i % num_points];
          manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
          manual_object_->colour(color_ds);
        }
        manual_object_->end();
      }
    }
  }

  // Lanes
  if (viz_lanes_->getBool() && msg->lanes.size()) {
    Ogre::ColourValue color_separators_allowed =
        rviz_common::properties::qtToOgre(color_property_separators_allowed_->getColor());
    Ogre::ColourValue color_separators_restricted =
        rviz_common::properties::qtToOgre(color_property_separators_restricted_->getColor());
    Ogre::ColourValue color_lane_centerlines =
        rviz_common::properties::qtToOgre(color_property_lane_centerlines_->getColor());
    Ogre::ColourValue color_grey_separators = color_separators_allowed;  // Used for Lane Separators with type unknown
    color_grey_separators.r = 128.0;
    color_grey_separators.g = 128.0;
    color_grey_separators.b = 128.0;
    color_separators_allowed.a = alpha_property_lane_->getFloat();
    color_separators_restricted.a = alpha_property_lane_->getFloat();
    color_lane_centerlines.a = alpha_property_lane_->getFloat();
    rviz_rendering::MaterialManager::enableAlphaBlending(material_separators_, color_separators_allowed.a);
    for (uint32_t i = 0; i < msg->lanes.size(); i++) {
      if (viz_lane_separators_->getBool()) {
        // Left
        manual_object_->estimateVertexCount(msg->lanes[i].left.line.size());
        manual_object_->begin(material_separators_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
        if (msg->lanes[i].left.type == route_planning_msgs::msg::LaneSeparator::TYPE_CROSSING_RESTRICTED) {
          manual_object_->colour(color_separators_restricted);
        } else {
          if (msg->lanes[i].left.type == route_planning_msgs::msg::LaneSeparator::TYPE_UNKNOWN) {
            manual_object_->colour(color_grey_separators);
          } else {
            manual_object_->colour(color_separators_allowed);
          }
        }
        for (uint32_t j = 0; j < msg->lanes[i].left.line.size(); j++) {
          const geometry_msgs::msg::Point& msg_point = msg->lanes[i].left.line[j];
          manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
        }
        manual_object_->end();
        // Right
        manual_object_->estimateVertexCount(msg->lanes[i].right.line.size());
        manual_object_->begin(material_separators_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
        if (msg->lanes[i].right.type == route_planning_msgs::msg::LaneSeparator::TYPE_CROSSING_RESTRICTED) {
          manual_object_->colour(color_separators_restricted);
        } else {
          if (msg->lanes[i].right.type == route_planning_msgs::msg::LaneSeparator::TYPE_UNKNOWN) {
            manual_object_->colour(color_grey_separators);
          } else {
            manual_object_->colour(color_separators_allowed);
          }
        }
        for (uint32_t j = 0; j < msg->lanes[i].right.line.size(); j++) {
          const geometry_msgs::msg::Point& msg_point = msg->lanes[i].right.line[j];
          manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
        }
        manual_object_->end();
      }
      if (viz_lane_centerline_->getBool()) {
        manual_object_->begin(material_separators_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
        manual_object_->colour(color_lane_centerlines);
        for (uint32_t j = 0; j < msg->lanes[i].centerline.size(); j++) {
          const geometry_msgs::msg::Point& msg_point = msg->lanes[i].centerline[j];
          manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
        }
        manual_object_->end();
      }
    }
  }

  // Regulatory Elements
  // First delete all spheres in current vector
  regelem_spheres_.clear();
  if (viz_regelems_->getBool() && msg->regulatory_elements.size()) {
    for (uint32_t i = 0; i < msg->regulatory_elements.size(); i++) {
      // Define Color
      Ogre::ColourValue regelem_color;
      if (msg->regulatory_elements[i].type == route_planning_msgs::msg::RegulatoryElement::TYPE_UNKNOWN) {
        regelem_color = color_grey_regelems;
      } else if (msg->regulatory_elements[i].type == route_planning_msgs::msg::RegulatoryElement::TYPE_SPEED_LIMIT) {
        regelem_color = color_regelems;
      } else  // It's TL, Yield, Stop... so everything with an active or passive state
      {
        if (msg->regulatory_elements[i].value == route_planning_msgs::msg::RegulatoryElement::MOVEMENT_ALLOWED) {
          regelem_color.r = 0.0;
          regelem_color.g = 255.0;
          regelem_color.b = 0.0;
          regelem_color.a = color_regelems.a;
        } else if (msg->regulatory_elements[i].value ==
                   route_planning_msgs::msg::RegulatoryElement::MOVEMENT_RESTRICTED) {
          regelem_color.r = 255.0;
          regelem_color.g = 0.0;
          regelem_color.b = 0.0;
          regelem_color.a = color_regelems.a;
        } else {
          regelem_color = color_grey_regelems;
        }
      }
      // Render Effect Line
      manual_object_->estimateVertexCount(msg->regulatory_elements[i].effect_line.size());
      manual_object_->begin(material_separators_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
      manual_object_->colour(regelem_color);
      for (uint32_t j = 0; j < msg->regulatory_elements[i].effect_line.size(); j++) {
        const geometry_msgs::msg::Point& msg_point = msg->regulatory_elements[i].effect_line[j];
        manual_object_->position(msg_point.x, msg_point.y, 0.0);  // z is s
      }
      manual_object_->end();
      // Render Signal Positions
      for (uint32_t j = 0; j < msg->regulatory_elements[i].signal_positions.size(); j++) {
        std::shared_ptr<rviz_rendering::Shape> shape =
            std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_);
        shape->setColor(regelem_color);
        Ogre::Vector3 pos(msg->regulatory_elements[i].signal_positions[j].x,
                          msg->regulatory_elements[i].signal_positions[j].y,
                          msg->regulatory_elements[i].signal_positions[j].z);
        shape->setPosition(pos);
        Ogre::Vector3 scale(1.0, 1.0, 1.0);
        shape->setScale(scale);
        regelem_spheres_.push_back(shape);
        // Render Text with information about regulatory element type and value
        // To Do
      }
    }
  }

  // Current Speed Limit
  if (viz_cur_speed_limit_->getBool() && msg->remaining_route.size()) {
    Ogre::Vector3 pos(msg->remaining_route[0].x, msg->remaining_route[0].y, 0.0);  // z is s
    // Speed Limit to String
    std::string str = "Current Speed Limit:\n" + std::to_string(msg->current_speed_limit) + " km/h";
    cur_speed_text_->setCaption(str);
    // Maybe there is a bug in rviz_rendering::MovableText::setGlobalTranslation
    // Currently only the given y-Position is set
    // https://github.com/ros2/rviz/blob/1ac419472ed06cdd52842a8f964f953a75395245/rviz_rendering/src/rviz_rendering/objects/movable_text.cpp#L520
    // Shows that the global_translation-vector is mutliplied with Ogre::Vector3::UNIT_Y is this intended?
    // In the ROS1 implementation the translation-vector is added without any multiplication
    // See: https://github.com/ros-visualization/rviz/blob/ec7ab1b0183244c05fbd2d0d1b8d8f53d8f42f2b/src/rviz/ogre_helpers/movable_text.cpp#L506
    // I've opened an Issue here: https://github.com/ros2/rviz/issues/974
    cur_speed_text_->setGlobalTranslation(pos);
    cur_speed_text_->setColor(color_regelems);
  } else {
    Ogre::ColourValue invisible;
    invisible.a = 0.0;
    cur_speed_text_->setColor(invisible);
  }
}

}  // namespace displays
}  // namespace route_planning_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(route_planning_msgs::displays::RouteDisplay, rviz_common::Display)