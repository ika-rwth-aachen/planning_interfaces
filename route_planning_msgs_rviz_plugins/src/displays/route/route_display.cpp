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
#include <cmath>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_rendering/material_manager.hpp"

#include <rviz_common/logging.hpp>

namespace route_planning_msgs {
namespace displays {

RouteDisplay::RouteDisplay() {
  color_property_target_ = new rviz_common::properties::ColorProperty(
      "Color Target", QColor(255, 0, 255), "Color to draw the target position.", this, SLOT(queueRender()));
  viz_sp_centerline_ =
      new rviz_common::properties::BoolProperty("Enable Route", true, "Visualize the shortes-path centerline.", this);
  color_property_route_elements_ = new rviz_common::properties::ColorProperty(
      "Color Route", QColor(0, 255, 255), "Color to draw the route.", viz_sp_centerline_,
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
  material_route_elements_ =
      rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name + "_route_elements");
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
  lane_marker_spheres_.clear();
  target_arrow_->setColor(invisible);
  cur_speed_text_->setColor(invisible);
  manual_object_->clear();
}

// bool validateFloats(const route_planning_msgs::msg::RouteElement& msg) {
//   bool valid = true;
//   valid = valid && rviz_common::validateFloats(msg.left_boundary);
//   valid = valid && rviz_common::validateFloats(msg.right_boundary);
//   valid = valid && rviz_common::validateFloats(msg.s);
//   for (size_t i = 0; i < msg.lane_elements.size(); ++i) {
//     valid = valid && rviz_common::validateFloats(msg.lane_elements[i].reference_pose);
//     valid = valid && rviz_common::validateFloats(msg.lane_elements[i].left_boundary.point);
//     valid = valid && rviz_common::validateFloats(msg.lane_elements[i].right_boundary.point);
//     for (size_t j = 0; j < msg.lane_elements[i].regulatory_elements.size(); ++j) {
//       valid = valid && rviz_common::validateFloats(msg.lane_elements[i].regulatory_elements[j].effect_line);
//       valid = valid && rviz_common::validateFloats(msg.lane_elements[i].regulatory_elements[j].sign_positions);
//     }
//   }
//   return valid;
// }

// bool validateFloats(const route_planning_msgs::msg::Route::ConstSharedPtr msg) {
//   bool valid = true;
//   valid = valid && rviz_common::validateFloats(msg->destination);
//   for (size_t i = 0; i < msg->traveled_route_elements.size(); ++i) {
//     valid = valid && validateFloats(msg->traveled_route_elements[i]);
//   }
//   for (size_t i = 0; i < msg->remaining_route_elements.size(); ++i) {
//     valid = valid && validateFloats(msg->remaining_route_elements[i]);
//   }
//   return valid;
// }

void RouteDisplay::processMessage(const route_planning_msgs::msg::Route::ConstSharedPtr msg) {
  // if (!validateFloats(msg)) {
  //   setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
  //             "Message contained invalid floating point values (nans or infs)");
  //   return;
  // }

  lane_marker_spheres_.clear();

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

  // route
  Ogre::ColourValue color_route_elements =
      rviz_common::properties::qtToOgre(color_property_route_elements_->getColor());
  color_route_elements.a = alpha_property_->getFloat();
  rviz_rendering::MaterialManager::enableAlphaBlending(material_route_elements_, color_route_elements.a);
  if (viz_sp_centerline_->getBool()) {
    if (!msg->remaining_route_elements.empty()) {
      // manual_object_->estimateVertexCount(msg->route_elements.size());
      // manual_object_->begin(material_route_elements_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
      // for (const auto& route_element : msg->route_elements) {
      //   double x = route_element.lane_elements[route_element.suggested_lane_idx].reference_pose.position.x;
      //   double y = route_element.lane_elements[route_element.suggested_lane_idx].reference_pose.position.y;
      //   double z = route_element.lane_elements[route_element.suggested_lane_idx].reference_pose.position.z;
      //   manual_object_->position(x, y, z);
      //   manual_object_->colour(color_route_elements);

      //   // draw centerline markers
      //   std::shared_ptr<rviz_rendering::Shape> marker =
      //     std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_);
      //   marker->setColor(color_regelems);
      //   marker->setPosition(Ogre::Vector3(x, y, z));
      //   marker->setScale(Ogre::Vector3(0.4, 0.4, 0.4));
      //   lane_marker_spheres_.push_back(marker);
      // }
      // manual_object_->end();
    }
  }

  // drivable space
  if (viz_driveable_space_->getBool()) {
    Ogre::ColourValue color_ds = rviz_common::properties::qtToOgre(color_property_driveable_space_->getColor());
    color_ds.a = alpha_property_->getFloat();
    rviz_rendering::MaterialManager::enableAlphaBlending(material_driveable_space_, color_ds.a);

    // left
    if (!msg->remaining_route_elements.empty()) {
      manual_object_->estimateVertexCount(msg->remaining_route_elements.size());
      manual_object_->begin(material_driveable_space_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
      for (const auto& route_element : msg->remaining_route_elements) {
        manual_object_->position(route_element.left_boundary.x, route_element.left_boundary.y, route_element.left_boundary.z);
        manual_object_->colour(color_ds);
      }
      manual_object_->end();
    }

    // right
    if (!msg->remaining_route_elements.empty()) {
      manual_object_->estimateVertexCount(msg->remaining_route_elements.size());
      manual_object_->begin(material_driveable_space_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
      for (const auto& route_element : msg->remaining_route_elements) {
        manual_object_->position(route_element.right_boundary.x, route_element.right_boundary.y, route_element.right_boundary.z);
        manual_object_->colour(color_ds);
      }
      manual_object_->end();
    }
  }

  // lane bounds
  if (viz_route_boundaries_->getBool()) {
    Ogre::ColourValue color_boundaries = rviz_common::properties::qtToOgre(color_property_boundaries_->getColor());
    color_boundaries.a = alpha_property_->getFloat();
    rviz_rendering::MaterialManager::enableAlphaBlending(material_boundaries_, color_boundaries.a);



    if (!msg->remaining_route_elements.empty()) {

      // draw markers
      for (size_t r = 0; r < msg->remaining_route_elements.size(); ++r) {
        const auto& route_element = msg->remaining_route_elements[r];
        for (size_t l = 0; l < route_element.lane_elements.size(); ++l) {
          const auto& lane_element = route_element.lane_elements[l];

          // draw left lane boundary marker
          if (lane_element.has_left_boundary) {
            std::shared_ptr<rviz_rendering::Shape> left_marker;
            if (l == route_element.suggested_lane_idx) {
              left_marker =
                std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_);
              left_marker->setColor(color_regelems);
              left_marker->setScale(Ogre::Vector3(0.4, 0.4, 0.4));
            } else {
              left_marker =
                std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cylinder, scene_manager_, scene_node_);
              left_marker->setColor(color_grey_regelems);
              left_marker->setScale(Ogre::Vector3(0.2, 0.5, 0.2));
              left_marker->setOrientation(Ogre::Quaternion(Ogre::Radian(Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_X));
            }
            left_marker->setPosition(Ogre::Vector3(lane_element.left_boundary.point.x, lane_element.left_boundary.point.y, lane_element.left_boundary.point.z));
            lane_marker_spheres_.push_back(left_marker);
          }

          // draw centerline marker
          std::shared_ptr<rviz_rendering::Shape> center_marker;
          if (l == route_element.suggested_lane_idx) {
            center_marker =
              std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_);
            center_marker->setColor(color_regelems);
            center_marker->setScale(Ogre::Vector3(0.4, 0.4, 0.4));
          } else {
            center_marker =
              std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cylinder, scene_manager_, scene_node_);
            center_marker->setColor(color_grey_regelems);
            center_marker->setScale(Ogre::Vector3(0.2, 0.5, 0.2));
            center_marker->setOrientation(Ogre::Quaternion(Ogre::Radian(Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_X));
          }
          center_marker->setPosition(Ogre::Vector3(lane_element.reference_pose.position.x, lane_element.reference_pose.position.y, lane_element.reference_pose.position.z));
          lane_marker_spheres_.push_back(center_marker);

          // draw right lane boundary marker
          if (lane_element.has_right_boundary) {
            std::shared_ptr<rviz_rendering::Shape> right_marker;
            if (l == route_element.suggested_lane_idx) {
              right_marker =
                std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_);
              right_marker->setColor(color_regelems);
              right_marker->setScale(Ogre::Vector3(0.4, 0.4, 0.4));
            } else {
              right_marker =
                std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cylinder, scene_manager_, scene_node_);
              right_marker->setColor(color_grey_regelems);
              right_marker->setScale(Ogre::Vector3(0.2, 0.5, 0.2));
              right_marker->setOrientation(Ogre::Quaternion(Ogre::Radian(Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_X));
            }
            right_marker->setPosition(Ogre::Vector3(lane_element.right_boundary.point.x, lane_element.right_boundary.point.y, lane_element.right_boundary.point.z));
            lane_marker_spheres_.push_back(right_marker);
          }
        }
      }

      // draw lines from RouteElem to next
      for (size_t r = 0; r < msg->remaining_route_elements.size() - 1; ++r) {
        const auto& route_element = msg->remaining_route_elements[r];
        const auto& next_route_element = msg->remaining_route_elements[r + 1];

        for (size_t l = 0; l < route_element.lane_elements.size(); ++l) {

          const auto& lane_element = route_element.lane_elements[l];

          int following_idx = lane_element.following_lane_idx;
          RVIZ_COMMON_LOG_INFO_STREAM("RouteElement " << r << " | LaneElement " << l << " | Has Following " << lane_element.has_following_lane_idx << " | Following LaneElement " << following_idx);

          // find following lane element in next route element
          if (lane_element.has_following_lane_idx && lane_element.following_lane_idx < next_route_element.lane_elements.size()) {
            const auto& next_lane_element = next_route_element.lane_elements[following_idx];

            // draw left lane boundary
            if (lane_element.has_left_boundary && next_lane_element.has_left_boundary) {
              manual_object_->estimateVertexCount(2);
              manual_object_->begin(material_boundaries_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
              manual_object_->colour(color_boundaries);
              manual_object_->position(lane_element.left_boundary.point.x, lane_element.left_boundary.point.y, lane_element.left_boundary.point.z);
              manual_object_->position(next_lane_element.left_boundary.point.x, next_lane_element.left_boundary.point.y, next_lane_element.left_boundary.point.z);
              manual_object_->end();
            }

            // draw centerline
            manual_object_->estimateVertexCount(2);
            manual_object_->begin(material_boundaries_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
            if (l == route_element.suggested_lane_idx) {
              manual_object_->colour(color_route_elements);
            } else {
              manual_object_->colour(color_boundaries);
            }
            manual_object_->position(lane_element.reference_pose.position.x, lane_element.reference_pose.position.y, lane_element.reference_pose.position.z);
            manual_object_->position(next_lane_element.reference_pose.position.x, next_lane_element.reference_pose.position.y, next_lane_element.reference_pose.position.z);
            manual_object_->end();

            // draw right lane boundary
            if (lane_element.has_right_boundary && next_lane_element.has_right_boundary) {
              manual_object_->estimateVertexCount(2);
              manual_object_->begin(material_boundaries_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
              manual_object_->colour(color_boundaries);
              manual_object_->position(lane_element.right_boundary.point.x, lane_element.right_boundary.point.y, lane_element.right_boundary.point.z);
              manual_object_->position(next_lane_element.right_boundary.point.x, next_lane_element.right_boundary.point.y, next_lane_element.right_boundary.point.z);
              manual_object_->end();
            }
          }
        }
      }
    }











    // // left
    // if (!msg->route_elements.empty()) {
    //   manual_object_->estimateVertexCount(msg->route_elements.size());
    //   manual_object_->begin(material_boundaries_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
    //   for (const auto& route_element : msg->route_elements) {
    //     route_planning_msgs::msg::LaneElement lane_element = route_element.lane_elements[route_element.suggested_lane_idx];
    //     manual_object_->position(lane_element.left_boundary.x, lane_element.left_boundary.y, lane_element.left_boundary.z);
    //     manual_object_->colour(color_boundaries);
    //   }
    //   manual_object_->end();
    // }

    // // right
    // if (!msg->route_elements.empty()) {
    //   manual_object_->estimateVertexCount(msg->route_elements.size());
    //   manual_object_->begin(material_boundaries_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
    //   for (const auto& route_element : msg->route_elements) {
    //     route_planning_msgs::msg::LaneElement lane_element = route_element.lane_elements[route_element.suggested_lane_idx];
    //     manual_object_->position(lane_element.right_boundary.x, lane_element.right_boundary.y, lane_element.right_boundary.z);
    //     manual_object_->colour(color_boundaries);
    //   }
    //   manual_object_->end();
    // }

    // Ogre::ColourValue color_traveled_route = rviz_common::properties::qtToOgre(color_property_traveled_route_->getColor());
    // color_traveled_route.a = alpha_property_->getFloat();
    // rviz_rendering::MaterialManager::enableAlphaBlending(material_traveled_route_, color_traveled_route.a);
    // if (!msg->route_elements.empty()) {
    //   for (const auto& route_element : msg->route_elements) {
    //     manual_object_->estimateVertexCount(route_element.lane_elements.size() + 1);
    //     manual_object_->begin(material_traveled_route_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
    //     for (const auto& lane_element : route_element.lane_elements) {
    //       manual_object_->position(lane_element.left_boundary.x, lane_element.left_boundary.y, lane_element.left_boundary.z);
    //       manual_object_->colour(color_traveled_route);
    //     }
    //     if (route_element.lane_elements.size() > 0) {
    //       manual_object_->position(route_element.lane_elements.back().right_boundary.x, route_element.lane_elements.back().right_boundary.y, route_element.lane_elements.back().right_boundary.z);
    //       manual_object_->colour(color_traveled_route);
    //     }
    //     manual_object_->end();
    //   }
    // }




  }

  // if (viz_driveable_space_->getBool()) {
  //   Ogre::ColourValue color_ds = rviz_common::properties::qtToOgre(color_property_driveable_space_->getColor());
  //   color_ds.a = alpha_property_->getFloat();
  //   rviz_rendering::MaterialManager::enableAlphaBlending(material_driveable_space_, color_ds.a);

  //   // DS Boundaries Left
  //   size_t num_points = msg->driveable_space.boundaries.left.size();
  //   if (num_points > 0) {
  //     manual_object_->estimateVertexCount(num_points);
  //     manual_object_->begin(material_driveable_space_->getName(), Ogre::RenderOperation::OT_LINE_STRIP,
  //                           "rviz_rendering");
  //     for (uint32_t i = 0; i < num_points; i++) {
  //       const geometry_msgs::msg::Point& msg_point = msg->driveable_space.boundaries.left[i % num_points];
  //       manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
  //       manual_object_->colour(color_ds);
  //     }
  //     manual_object_->end();
  //   }

  //   // DS Boundaries right
  //   num_points = msg->driveable_space.boundaries.right.size();
  //   if (num_points > 0) {
  //     manual_object_->estimateVertexCount(num_points);
  //     manual_object_->begin(material_driveable_space_->getName(), Ogre::RenderOperation::OT_LINE_STRIP,
  //                           "rviz_rendering");
  //     for (uint32_t i = 0; i < num_points; i++) {
  //       const geometry_msgs::msg::Point& msg_point = msg->driveable_space.boundaries.right[i % num_points];
  //       manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
  //       manual_object_->colour(color_ds);
  //     }
  //     manual_object_->end();
  //   }

  //   // Restricted areas
  //   if (msg->driveable_space.restricted_areas.size() > 0) {
  //     for (size_t j = 0; j < msg->driveable_space.restricted_areas.size(); j++) {
  //       num_points = msg->driveable_space.restricted_areas[j].points.size();
  //       manual_object_->estimateVertexCount(num_points);
  //       manual_object_->begin(material_driveable_space_->getName(), Ogre::RenderOperation::OT_LINE_STRIP,
  //                             "rviz_rendering");
  //       for (uint32_t i = 0; i < num_points + 1; ++i) {
  //         const geometry_msgs::msg::Point32& msg_point =
  //             msg->driveable_space.restricted_areas[j].points[i % num_points];
  //         manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
  //         manual_object_->colour(color_ds);
  //       }
  //       manual_object_->end();
  //     }
  //   }
  // }

  // // Lanes
  // if (viz_lanes_->getBool() && msg->lanes.size()) {
  //   Ogre::ColourValue color_separators_allowed =
  //       rviz_common::properties::qtToOgre(color_property_separators_allowed_->getColor());
  //   Ogre::ColourValue color_separators_restricted =
  //       rviz_common::properties::qtToOgre(color_property_separators_restricted_->getColor());
  //   Ogre::ColourValue color_lane_centerlines =
  //       rviz_common::properties::qtToOgre(color_property_lane_centerlines_->getColor());
  //   Ogre::ColourValue color_grey_separators = color_separators_allowed;  // Used for Lane Separators with type unknown
  //   color_grey_separators.r = 128.0;
  //   color_grey_separators.g = 128.0;
  //   color_grey_separators.b = 128.0;
  //   color_separators_allowed.a = alpha_property_lane_->getFloat();
  //   color_separators_restricted.a = alpha_property_lane_->getFloat();
  //   color_lane_centerlines.a = alpha_property_lane_->getFloat();
  //   rviz_rendering::MaterialManager::enableAlphaBlending(material_separators_, color_separators_allowed.a);
  //   for (uint32_t i = 0; i < msg->lanes.size(); i++) {
  //     if (viz_lane_separators_->getBool()) {
  //       // Left
  //       manual_object_->estimateVertexCount(msg->lanes[i].left.line.size());
  //       manual_object_->begin(material_separators_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
  //       if (msg->lanes[i].left.type == route_planning_msgs::msg::LaneSeparator::TYPE_CROSSING_RESTRICTED) {
  //         manual_object_->colour(color_separators_restricted);
  //       } else {
  //         if (msg->lanes[i].left.type == route_planning_msgs::msg::LaneSeparator::TYPE_UNKNOWN) {
  //           manual_object_->colour(color_grey_separators);
  //         } else {
  //           manual_object_->colour(color_separators_allowed);
  //         }
  //       }
  //       for (uint32_t j = 0; j < msg->lanes[i].left.line.size(); j++) {
  //         const geometry_msgs::msg::Point& msg_point = msg->lanes[i].left.line[j];
  //         manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
  //       }
  //       manual_object_->end();
  //       // Right
  //       manual_object_->estimateVertexCount(msg->lanes[i].right.line.size());
  //       manual_object_->begin(material_separators_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
  //       if (msg->lanes[i].right.type == route_planning_msgs::msg::LaneSeparator::TYPE_CROSSING_RESTRICTED) {
  //         manual_object_->colour(color_separators_restricted);
  //       } else {
  //         if (msg->lanes[i].right.type == route_planning_msgs::msg::LaneSeparator::TYPE_UNKNOWN) {
  //           manual_object_->colour(color_grey_separators);
  //         } else {
  //           manual_object_->colour(color_separators_allowed);
  //         }
  //       }
  //       for (uint32_t j = 0; j < msg->lanes[i].right.line.size(); j++) {
  //         const geometry_msgs::msg::Point& msg_point = msg->lanes[i].right.line[j];
  //         manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
  //       }
  //       manual_object_->end();
  //     }
  //     if (viz_lane_centerline_->getBool()) {
  //       manual_object_->begin(material_separators_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
  //       manual_object_->colour(color_lane_centerlines);
  //       for (uint32_t j = 0; j < msg->lanes[i].centerline.size(); j++) {
  //         const geometry_msgs::msg::Point& msg_point = msg->lanes[i].centerline[j];
  //         manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
  //       }
  //       manual_object_->end();
  //     }
  //   }
  // }

  // Regulatory Elements
  // First delete all spheres in current vector
  // regelem_spheres_.clear();
  // if (viz_regelems_->getBool() && msg->regulatory_elements.size()) {
  //   for (uint32_t i = 0; i < msg->regulatory_elements.size(); i++) {
  //     // Define Color
  //     Ogre::ColourValue regelem_color;
  //     if (msg->regulatory_elements[i].type == route_planning_msgs::msg::RegulatoryElement::TYPE_UNKNOWN) {
  //       regelem_color = color_grey_regelems;
  //     } else if (msg->regulatory_elements[i].type == route_planning_msgs::msg::RegulatoryElement::TYPE_SPEED_LIMIT) {
  //       regelem_color = color_regelems;
  //     } else  // It's TL, Yield, Stop... so everything with an active or passive state
  //     {
  //       if (msg->regulatory_elements[i].value == route_planning_msgs::msg::RegulatoryElement::MOVEMENT_ALLOWED) {
  //         regelem_color.r = 0.0;
  //         regelem_color.g = 255.0;
  //         regelem_color.b = 0.0;
  //         regelem_color.a = color_regelems.a;
  //       } else if (msg->regulatory_elements[i].value ==
  //                  route_planning_msgs::msg::RegulatoryElement::MOVEMENT_RESTRICTED) {
  //         regelem_color.r = 255.0;
  //         regelem_color.g = 0.0;
  //         regelem_color.b = 0.0;
  //         regelem_color.a = color_regelems.a;
  //       } else {
  //         regelem_color = color_grey_regelems;
  //       }
  //     }
  //     // Render Effect Line
  //     manual_object_->estimateVertexCount(msg->regulatory_elements[i].effect_line.size());
  //     manual_object_->begin(material_separators_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
  //     manual_object_->colour(regelem_color);
  //     for (uint32_t j = 0; j < msg->regulatory_elements[i].effect_line.size(); j++) {
  //       const geometry_msgs::msg::Point& msg_point = msg->regulatory_elements[i].effect_line[j];
  //       manual_object_->position(msg_point.x, msg_point.y, 0.0);  // z is s
  //     }
  //     manual_object_->end();
  //     // Render Signal Positions
  //     for (uint32_t j = 0; j < msg->regulatory_elements[i].sign_positions.size(); j++) {
  //       std::shared_ptr<rviz_rendering::Shape> shape =
  //           std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_);
  //       shape->setColor(regelem_color);
  //       Ogre::Vector3 pos(msg->regulatory_elements[i].sign_positions[j].x,
  //                         msg->regulatory_elements[i].sign_positions[j].y,
  //                         msg->regulatory_elements[i].sign_positions[j].z);
  //       shape->setPosition(pos);
  //       Ogre::Vector3 scale(1.0, 1.0, 1.0);
  //       shape->setScale(scale);
  //       regelem_spheres_.push_back(shape);
  //       // Render Text with information about regulatory element type and value
  //       // To Do
  //     }
  //   }
  // }

  // Current Speed Limit
//   if (viz_cur_speed_limit_->getBool() && msg->route_elements.size()) {
//     Ogre::Vector3 pos(msg->route_elements[0].x, msg->route_elements[0].y, 0.0);  // z is s
//     // Speed Limit to String
//     std::string str = "Current Speed Limit:\n" + std::to_string(msg->current_speed_limit) + " km/h";
//     cur_speed_text_->setCaption(str);
//     // Maybe there is a bug in rviz_rendering::MovableText::setGlobalTranslation
//     // Currently only the given y-Position is set
//     // https://github.com/ros2/rviz/blob/1ac419472ed06cdd52842a8f964f953a75395245/rviz_rendering/src/rviz_rendering/objects/movable_text.cpp#L520
//     // Shows that the global_translation-vector is mutliplied with Ogre::Vector3::UNIT_Y is this intended?
//     // In the ROS1 implementation the translation-vector is added without any multiplication
//     // See: https://github.com/ros-visualization/rviz/blob/ec7ab1b0183244c05fbd2d0d1b8d8f53d8f42f2b/src/rviz/ogre_helpers/movable_text.cpp#L506
//     // I've opened an Issue here: https://github.com/ros2/rviz/issues/974
//     cur_speed_text_->setGlobalTranslation(pos);
//     cur_speed_text_->setColor(color_regelems);
//   } else {
//     Ogre::ColourValue invisible;
//     invisible.a = 0.0;
//     cur_speed_text_->setColor(invisible);
//   }
}

}  // namespace displays
}  // namespace route_planning_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(route_planning_msgs::displays::RouteDisplay, rviz_common::Display)