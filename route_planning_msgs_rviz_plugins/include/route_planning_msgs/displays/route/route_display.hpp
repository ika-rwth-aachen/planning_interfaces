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

#pragma once

#include <route_planning_msgs/msg/lane_element.hpp>
#include <route_planning_msgs/msg/regulatory_element.hpp>
#include <route_planning_msgs/msg/route.hpp>
#include <route_planning_msgs/msg/route_element.hpp>

#include <rviz_common/message_filter_display.hpp>

#include "rviz_default_plugins/visibility_control.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/movable_text.hpp"
#include "rviz_rendering/objects/shape.hpp"

namespace Ogre {
class ManualObject;
}

namespace rviz_common {
namespace properties {
class ColorProperty;
class FloatProperty;
}  // namespace properties
}  // namespace rviz_common

namespace route_planning_msgs {
namespace displays {

/**
 * \class RouteDisplay
 * \brief Displays a route_planning_msgs::Route message
 */
class RouteDisplay : public rviz_common::MessageFilterDisplay<route_planning_msgs::msg::Route> {
  Q_OBJECT

 public:
  RouteDisplay();
  ~RouteDisplay() override;


  void reset() override;

 protected:
  void processMessage(const route_planning_msgs::msg::Route::ConstSharedPtr msg) override;

  void onInitialize() override;

  Ogre::ManualObject *manual_object_;
  Ogre::MaterialPtr material_route_elements_, material_boundaries_,
      material_driveable_space_, material_separators_, material_regelems_;
  rviz_rendering::Arrow *target_arrow_;
  rviz_rendering::MovableText *cur_speed_text_;
  double target_arrow_shaft_length_ = 2.0;
  double target_arrow_shaft_diameter_ = 0.5;
  double target_arrow_head_length_ = 1.0;
  double target_arrow_head_diameter_ = 1.0;
  std::vector<std::shared_ptr<rviz_rendering::Shape>> regelem_spheres_;
  std::vector<std::shared_ptr<rviz_rendering::Shape>> lane_marker_spheres_;

  rviz_common::properties::BoolProperty *viz_route_boundaries_, *viz_driveable_space_, *viz_sp_centerline_, *viz_lanes_,
      *viz_lane_separators_, *viz_lane_centerline_, *viz_regelems_, *viz_cur_speed_limit_;
  rviz_common::properties::ColorProperty *color_property_route_elements_,
      *color_property_target_, *color_property_boundaries_, *color_property_driveable_space_,
      *color_property_separators_allowed_, *color_property_separators_restricted_, *color_property_lane_centerlines_,
      *color_property_regelems_;
  rviz_common::properties::FloatProperty *alpha_property_, *alpha_property_lane_;
};

}  // namespace displays
}  // namespace route_planning_msgs
