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

#include "trajectory_planning_msgs/displays/trajectory_display.hpp"

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <QString>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_rendering/material_manager.hpp"

namespace trajectory_planning_msgs {
namespace displays {

TrajectoryDisplay::TrajectoryDisplay() {
  vel_trj_ = nullptr;
  time_trj_ = nullptr;
  acc_trj_ = nullptr;
  s_trj_ = nullptr;
  viz_vel_ = new rviz_common::properties::BoolProperty("Show velocity trajectory", true,
                                                       "Visualize velocity trajectory.", this);
  color_property_vel_ = new rviz_common::properties::ColorProperty(
      "Color velocity trajectory/samples", QColor(0, 255, 0), "Color to draw the velocity trajectory/sample-points.",
      viz_vel_, SLOT(queueRender()));
  viz_vel_points_ = new rviz_common::properties::BoolProperty("Show velocity samples", false,
                                                              "Visualize velocity sample points.", viz_vel_);
  viz_yaw_arrows_ = new rviz_common::properties::BoolProperty(
      "Show yaw arrows", false, "Visualize arrows indicating the vehicle yaw-angle.", viz_vel_);
  scale_property_yaw_arrows_ = new rviz_common::properties::FloatProperty("Scale", 0.5f, "Scale of yaw arrows.",
                                                                          viz_yaw_arrows_, SLOT(queueRender()));
  color_property_yaw_ = new rviz_common::properties::ColorProperty(
      "Color yaw arrows", QColor(255, 85, 0), "Color to draw the arrows indicatind the vehicel yaw.", viz_yaw_arrows_,
      SLOT(queueRender()));
  viz_time_ =
      new rviz_common::properties::BoolProperty("Show time trajectory", false, "Visualize time trajectory.", this);
  color_property_time_ = new rviz_common::properties::ColorProperty("Color time trajectory/samples", QColor(0, 0, 255),
                                                                    "Color to draw the time trajectory/sample-points.",
                                                                    viz_time_, SLOT(queueRender()));
  viz_time_points_ =
      new rviz_common::properties::BoolProperty("Show time samples", false, "Visualize time sample points.", viz_time_);
  viz_acc_ = new rviz_common::properties::BoolProperty("Show acceleration trajectory", false,
                                                       "Visualize acceleration trajectory.", this);
  color_property_acc_ = new rviz_common::properties::ColorProperty(
      "Color acceleration trajectory/samples", QColor(255, 255, 0),
      "Color to draw the acceleration trajectory/sample-points.", viz_acc_, SLOT(queueRender()));
  viz_acc_points_ = new rviz_common::properties::BoolProperty("Show acceleration samples", false,
                                                              "Visualize acceleration sample points.", viz_acc_);
  viz_s_ = new rviz_common::properties::BoolProperty("Show distance trajectory", false,
                                                     "Visualize distance trajectory.", this);
  color_property_s_ = new rviz_common::properties::ColorProperty("Color distance trajectory/samples", QColor(255, 0, 0),
                                                                 "Color to draw the distance trajectory/sample-points.",
                                                                 viz_s_, SLOT(queueRender()));
  viz_s_points_ = new rviz_common::properties::BoolProperty("Show distance samples", false,
                                                            "Visualize distance sample points.", viz_s_);
  size_property_points_ = new rviz_common::properties::FloatProperty("Point-Size", 0.5f, "Radius of sample points.",
                                                                     this, SLOT(queueRender()));
  alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 1.0f, "Amount of transparency to apply.", this,
                                                               SLOT(queueRender()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  static int tra_count = 0;
  std::string material_name = "TrajectoryMaterial" + std::to_string(tra_count++);
  material_vel_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name + "_vel");
  material_time_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name + "_time");
  material_acc_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name + "_acc");
  material_s_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name + "_s");

  // timeout properties
  enable_timeout_property_ = new rviz_common::properties::BoolProperty("Timeout", true, "Remove renderings after timeout if no new msgs have been received", this);
  timeout_property_ = new rviz_common::properties::FloatProperty("Duration", 1.0, "Timeout duration in seconds (wall time)", enable_timeout_property_);
}

TrajectoryDisplay::~TrajectoryDisplay() {
  if (timeout_timer_) {
    timeout_timer_->cancel();
  }
  timeout_timer_.reset();

  if (initialized()) {
    if (vel_trj_) {
      scene_manager_->destroyManualObject(vel_trj_);
      vel_trj_ = nullptr;
    }
    if (time_trj_) {
      scene_manager_->destroyManualObject(time_trj_);
      time_trj_ = nullptr;
    }
    if (acc_trj_) {
      scene_manager_->destroyManualObject(acc_trj_);
      acc_trj_ = nullptr;
    }
    if (s_trj_) {
      scene_manager_->destroyManualObject(s_trj_);
      s_trj_ = nullptr;
    }
  }
}

void TrajectoryDisplay::onInitialize() {
  MFDClass::onInitialize();

  vel_trj_ = scene_manager_->createManualObject();
  vel_trj_->setDynamic(true);
  scene_node_->attachObject(vel_trj_);

  time_trj_ = scene_manager_->createManualObject();
  time_trj_->setDynamic(true);
  scene_node_->attachObject(time_trj_);

  acc_trj_ = scene_manager_->createManualObject();
  acc_trj_->setDynamic(true);
  scene_node_->attachObject(acc_trj_);

  s_trj_ = scene_manager_->createManualObject();
  s_trj_->setDynamic(true);
  scene_node_->attachObject(s_trj_);
}

void TrajectoryDisplay::reset() {
  MFDClass::reset();
  vel_trj_->clear();
  time_trj_->clear();
  acc_trj_->clear();
  s_trj_->clear();
  vel_point_spheres_.clear();
  time_point_spheres_.clear();
  acc_point_spheres_.clear();
  s_point_spheres_.clear();
  yaw_arrows_.clear();
}

bool validateFloats(trajectory_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg->states);
  return valid;
}

void TrajectoryDisplay::processMessage(trajectory_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
  if (!validateFloats(msg)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Message",
              "Message contained invalid floating point values (nans or infs)");
    return;
  }
  // sanity check trajectory
  // trajectory_planning_msgs::trajectory_access::sanityCheckTrajectory(*msg); <- throws exception

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }
  setTransformOk();

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  // clear previous visualization
  reset();

  Ogre::ColourValue color_vel = rviz_common::properties::qtToOgre(color_property_vel_->getColor());
  Ogre::ColourValue color_yaw = rviz_common::properties::qtToOgre(color_property_yaw_->getColor());
  Ogre::ColourValue color_time = rviz_common::properties::qtToOgre(color_property_time_->getColor());
  Ogre::ColourValue color_acc = rviz_common::properties::qtToOgre(color_property_acc_->getColor());
  Ogre::ColourValue color_s = rviz_common::properties::qtToOgre(color_property_s_->getColor());
  color_vel.a = alpha_property_->getFloat();
  color_yaw.a = alpha_property_->getFloat();
  color_time.a = alpha_property_->getFloat();
  color_acc.a = alpha_property_->getFloat();
  color_s.a = alpha_property_->getFloat();
  rviz_rendering::MaterialManager::enableAlphaBlending(material_vel_, color_vel.a);
  rviz_rendering::MaterialManager::enableAlphaBlending(material_time_, color_time.a);
  rviz_rendering::MaterialManager::enableAlphaBlending(material_acc_, color_acc.a);
  rviz_rendering::MaterialManager::enableAlphaBlending(material_s_, color_s.a);

  size_t num_points = trajectory_planning_msgs::trajectory_access::getSamplePointSize(*msg);
  if (num_points > 0) {
    if (viz_vel_->getBool()) {
      vel_trj_->estimateVertexCount(num_points);
      vel_trj_->begin(material_vel_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
      vel_trj_->colour(color_vel);
    }
    if (viz_time_->getBool()) {
      time_trj_->estimateVertexCount(num_points);
      time_trj_->begin(material_time_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
      time_trj_->colour(color_time);
    }
    if (viz_acc_->getBool()) {
      if (msg->type_id == trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID || msg->type_id == trajectory_planning_msgs::msg::DRIVABLERWS::TYPE_ID) {
        acc_trj_->estimateVertexCount(num_points);
        acc_trj_->begin(material_acc_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
        acc_trj_->colour(color_acc);
      } else {
        setStatus(rviz_common::properties::StatusProperty::Warn, "Message",
                  "Message containing ID " + QString::number(msg->type_id) + " is not supported. Unable to visualize acceleration trajectory!");
      }
    }
    if (viz_s_->getBool()) {
      if (msg->type_id == trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID || msg->type_id == trajectory_planning_msgs::msg::DRIVABLERWS::TYPE_ID) {
        s_trj_->estimateVertexCount(num_points);
        s_trj_->begin(material_s_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
        s_trj_->colour(color_s);
      } else {
        setStatus(
            rviz_common::properties::StatusProperty::Warn, "Message",
            "Message containing ID " + QString::number(msg->type_id) + " is not supported. Unable to visualize distance trajectory!");
      }
    }
    float point_size = size_property_points_->getFloat();
    float arrow_scale = scale_property_yaw_arrows_->getFloat();
    Ogre::Vector3 scale(point_size, point_size, point_size);
    for (uint32_t i = 0; i < num_points; i++) {
      if (viz_vel_->getBool()) {
        vel_trj_->position(trajectory_planning_msgs::trajectory_access::getX(*msg, i),
                           trajectory_planning_msgs::trajectory_access::getY(*msg, i),
                           trajectory_planning_msgs::trajectory_access::getV(*msg, i));
      }
      if (viz_vel_points_->getBool()) {
        std::shared_ptr<rviz_rendering::Shape> shape =
            std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_);
        shape->setColor(color_vel);
        Ogre::Vector3 pos(trajectory_planning_msgs::trajectory_access::getX(*msg, i),
                          trajectory_planning_msgs::trajectory_access::getY(*msg, i),
                          trajectory_planning_msgs::trajectory_access::getV(*msg, i));
        shape->setPosition(pos);
        shape->setScale(scale);
        vel_point_spheres_.push_back(shape);
      }
      if (viz_yaw_arrows_->getBool()) {
        Ogre::Vector3 pos(trajectory_planning_msgs::trajectory_access::getX(*msg, i),
                          trajectory_planning_msgs::trajectory_access::getY(*msg, i),
                          trajectory_planning_msgs::trajectory_access::getV(*msg, i));
        std::shared_ptr<rviz_rendering::Arrow> arrow =
            std::make_shared<rviz_rendering::Arrow>(scene_manager_, scene_node_);
        arrow->setColor(color_yaw);
        arrow->setPosition(pos);
        // Use tf to identify direction vector
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, trajectory_planning_msgs::trajectory_access::getTheta(*msg, i));
        tf2::Vector3 unit_vector(1, 0, 0);
        tf2::Vector3 direction = tf2::quatRotate(quaternion, unit_vector);
        // Convert to Ogre::Vector3
        Ogre::Vector3 dir(direction.getX(), direction.getY(), direction.getZ());
        arrow->setDirection(dir);
        arrow->setScale(Ogre::Vector3(arrow_scale, arrow_scale, arrow_scale));
        yaw_arrows_.push_back(arrow);
      }
      if (viz_time_->getBool())
        time_trj_->position(trajectory_planning_msgs::trajectory_access::getX(*msg, i),
                            trajectory_planning_msgs::trajectory_access::getY(*msg, i),
                            trajectory_planning_msgs::trajectory_access::getT(*msg, i));
      if (viz_time_points_->getBool()) {
        std::shared_ptr<rviz_rendering::Shape> shape =
            std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_);
        shape->setColor(color_time);
        Ogre::Vector3 pos(trajectory_planning_msgs::trajectory_access::getX(*msg, i),
                          trajectory_planning_msgs::trajectory_access::getY(*msg, i),
                          trajectory_planning_msgs::trajectory_access::getT(*msg, i));
        shape->setPosition(pos);
        shape->setScale(scale);
        time_point_spheres_.push_back(shape);
      }
      if (viz_acc_->getBool()) {
        if (msg->type_id == trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID || msg->type_id == trajectory_planning_msgs::msg::DRIVABLERWS::TYPE_ID) {
          acc_trj_->position(trajectory_planning_msgs::trajectory_access::getX(*msg, i),
                             trajectory_planning_msgs::trajectory_access::getY(*msg, i),
                             trajectory_planning_msgs::trajectory_access::getA(*msg, i));
        } else {
          setStatus(rviz_common::properties::StatusProperty::Warn, "Message",
                    "Message containing ID " + QString::number(msg->type_id) + " is not supported. Unable to visualize acceleration trajectory!");
        }
      }
      if (viz_acc_points_->getBool()) {
        if (msg->type_id == trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID || msg->type_id == trajectory_planning_msgs::msg::DRIVABLERWS::TYPE_ID) {
          std::shared_ptr<rviz_rendering::Shape> shape =
              std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_);
          shape->setColor(color_acc);
          Ogre::Vector3 pos(trajectory_planning_msgs::trajectory_access::getX(*msg, i),
                            trajectory_planning_msgs::trajectory_access::getY(*msg, i),
                            trajectory_planning_msgs::trajectory_access::getA(*msg, i));
          shape->setPosition(pos);
          shape->setScale(scale);
          acc_point_spheres_.push_back(shape);
        } else {
          setStatus(rviz_common::properties::StatusProperty::Warn, "Message",
                    "Message containing ID " + QString::number(msg->type_id) + " is not supported. Unable to visualize acceleration "
                    "samples!");
        }
      }
      if (viz_s_->getBool() ) {
        if (msg->type_id == trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID || msg->type_id == trajectory_planning_msgs::msg::DRIVABLERWS::TYPE_ID) {
          s_trj_->position(trajectory_planning_msgs::trajectory_access::getX(*msg, i),
                           trajectory_planning_msgs::trajectory_access::getY(*msg, i),
                           trajectory_planning_msgs::trajectory_access::getS(*msg, i));
        } else {
          setStatus(rviz_common::properties::StatusProperty::Warn, "Message",
                    "Message containing ID " + QString::number(msg->type_id) + " is not supported. Unable to visualize distance trajectory!");
        }
      }
      if (viz_s_points_->getBool()) {
        if (msg->type_id == trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID || msg->type_id == trajectory_planning_msgs::msg::DRIVABLERWS::TYPE_ID) {
          std::shared_ptr<rviz_rendering::Shape> shape =
              std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_);
          shape->setColor(color_s);
          Ogre::Vector3 pos(trajectory_planning_msgs::trajectory_access::getX(*msg, i),
                            trajectory_planning_msgs::trajectory_access::getY(*msg, i),
                            trajectory_planning_msgs::trajectory_access::getS(*msg, i));
          shape->setPosition(pos);
          shape->setScale(scale);
          s_point_spheres_.push_back(shape);
        } else {
          setStatus(
              rviz_common::properties::StatusProperty::Warn, "Message",
              "Message containing ID " + QString::number(msg->type_id) + " is not supported. Unable to visualize distance samples!");
        }
      }
    }
    if (viz_vel_->getBool()) vel_trj_->end();
    if (viz_time_->getBool()) time_trj_->end();
    if (viz_acc_->getBool() && (msg->type_id == trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID || msg->type_id == trajectory_planning_msgs::msg::DRIVABLERWS::TYPE_ID)) acc_trj_->end();
    if (viz_s_->getBool() && (msg->type_id == trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID || msg->type_id == trajectory_planning_msgs::DRIVABLERWS::TYPE_ID))  s_trj_->end();

  } else {
    setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "Message contains no points");
  }

  // reset scene after timeout, if enabled
  if (enable_timeout_property_->getBool()) {
    timeout_timer_ = rviz_ros_node_.lock()->get_raw_node()->create_wall_timer(
      std::chrono::duration<float>(timeout_property_->getFloat()),
      std::bind(&TrajectoryDisplay::timeoutTimerCallback, this)
    );
  }
}

void TrajectoryDisplay::timeoutTimerCallback() {
  timeout_timer_->cancel();
  this->reset();
}

}  // namespace displays
}  // namespace trajectory_planning_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(trajectory_planning_msgs::displays::TrajectoryDisplay, rviz_common::Display)