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

#include <route_planning_msgs/msg/route.hpp>
#include <route_planning_msgs_utils/route_access.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/bool_property.hpp>

#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/shape.hpp>

namespace route_planning_msgs {
namespace displays {

constexpr float ARROW_SHAFT_LENGTH = 2.0f;
constexpr float ARROW_SHAFT_DIAMETER = 0.5f;
constexpr float ARROW_HEAD_LENGTH = 1.0f;
constexpr float ARROW_HEAD_DIAMETER = 1.0f;

constexpr float VERTICAL_OFFSET_EPSILON = 0.05f; // to avoid overlapping with other lines

/**
 * \class RouteDisplay
 * \brief Displays a route_planning_msgs::Route message
 */
class RouteDisplay : public rviz_common::MessageFilterDisplay<route_planning_msgs::msg::Route> {
  Q_OBJECT

  public:
    void reset() override;

 protected:
  void onInitialize() override;
  void processMessage(const route_planning_msgs::msg::Route::ConstSharedPtr msg) override;

  std::shared_ptr<rviz_rendering::Arrow> generateRenderArrow(const geometry_msgs::msg::Pose& pose, const Ogre::ColourValue& color, const float scale, const float opacity = 1.0);
  std::shared_ptr<rviz_rendering::Shape> generateRenderPoint(const geometry_msgs::msg::Point& point, const Ogre::ColourValue& color, const float scale, const float opacity = 1.0);
  std::shared_ptr<rviz_rendering::BillboardLine> generateRenderLine(const std::vector<geometry_msgs::msg::Point>& points, const Ogre::ColourValue& color, const float scale, const float opacity = 1.0, const float vertical_offset = 0.0);

  // destination
  std::shared_ptr<rviz_rendering::Arrow> destination_arrow_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_destination_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_destination_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_destination_;

  // traveled route
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_traveled_route_;
  std::unique_ptr<rviz_common::properties::FloatProperty> opacity_property_traveled_route_;

  // suggested lane
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_reference_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_boundaries_;

  std::vector<std::shared_ptr<rviz_rendering::Arrow>> suggested_lane_reference_poses_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_reference_poses_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_suggested_lane_reference_poses_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_suggested_lane_reference_poses_;

  std::vector<std::shared_ptr<rviz_rendering::BillboardLine>> suggested_lane_reference_line_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_reference_line_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_suggested_lane_reference_line_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_suggested_lane_reference_line_;

  std::vector<std::shared_ptr<rviz_rendering::Shape>> suggested_lane_boundary_points_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_boundary_points_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_suggested_lane_boundary_points_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_suggested_lane_boundary_points_;

  std::vector<std::shared_ptr<rviz_rendering::BillboardLine>> suggested_lane_boundary_lines_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_boundary_lines_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_suggested_lane_boundary_lines_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_suggested_lane_boundary_lines_;

  std::vector<std::shared_ptr<rviz_rendering::BillboardLine>> suggested_lane_regulatory_elements_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_regulatory_elements_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_suggested_lane_regulatory_elements_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_suggested_lane_regulatory_elements_;
  std::vector<std::shared_ptr<rviz_rendering::Shape>> suggested_lane_regulatory_elements_sign_positions_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_regulatory_elements_sign_positions_;

  // adjacent lanes
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lanes_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lanes_reference_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lanes_boundaries_;

  std::vector<std::shared_ptr<rviz_rendering::Arrow>> adjacent_lanes_reference_poses_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lanes_reference_poses_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_adjacent_lanes_reference_poses_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_adjacent_lanes_reference_poses_;

  std::vector<std::shared_ptr<rviz_rendering::BillboardLine>> adjacent_lanes_reference_line_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lanes_reference_line_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_adjacent_lanes_reference_line_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_adjacent_lanes_reference_line_;

  std::vector<std::shared_ptr<rviz_rendering::Shape>> adjacent_lanes_boundary_points_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lanes_boundary_points_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_adjacent_lanes_boundary_points_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_adjacent_lanes_boundary_points_;

  std::vector<std::shared_ptr<rviz_rendering::BillboardLine>> adjacent_lanes_boundary_lines_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lanes_boundary_lines_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_adjacent_lanes_boundary_lines_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_adjacent_lanes_boundary_lines_;

  std::vector<std::shared_ptr<rviz_rendering::BillboardLine>> adjacent_lane_regulatory_elements_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lane_regulatory_elements_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_adjacent_lane_regulatory_elements_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_adjacent_lane_regulatory_elements_;
  std::vector<std::shared_ptr<rviz_rendering::Shape>> adjacent_lane_regulatory_elements_sign_positions_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lane_regulatory_elements_sign_positions_;

  // drivable space
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_drivable_space_;

  std::vector<std::shared_ptr<rviz_rendering::Shape>> drivable_space_points_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_drivable_space_points_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_drivable_space_points_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_drivable_space_points_;

  std::vector<std::shared_ptr<rviz_rendering::BillboardLine>> drivable_space_lines_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_drivable_space_lines_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_drivable_space_lines_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_drivable_space_lines_;

  // lane change
  std::vector<std::shared_ptr<rviz_rendering::BillboardLine>> lane_change_lines_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_lane_change_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_lane_change_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_lane_change_;
};

}  // namespace displays
}  // namespace route_planning_msgs
