#pragma once

#include <route_planning_msgs/msg/route.hpp>
#include <route_planning_msgs_utils/route_access.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/bool_property.hpp>

#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/line.hpp>
#include <rviz_rendering/objects/shape.hpp>

namespace route_planning_msgs {
namespace displays {

/**
 * \class NewRouteDisplay
 * \brief Displays a route_planning_msgs::Route message
 */
class NewRouteDisplay : public rviz_common::MessageFilterDisplay<route_planning_msgs::msg::Route> {
  Q_OBJECT

  public:
    void reset() override;

private Q_SLOTS:
  void updateStyle();

 protected:
  void onInitialize() override;
  void processMessage(const route_planning_msgs::msg::Route::ConstSharedPtr msg) override;

  std::shared_ptr<rviz_rendering::Arrow> generateRenderArrow(const geometry_msgs::msg::Pose& pose, const Ogre::ColourValue& color, const float scale);
  std::shared_ptr<rviz_rendering::Shape> generateRenderPoint(const geometry_msgs::msg::Point& point, const Ogre::ColourValue& color, const float scale);
  std::shared_ptr<rviz_rendering::Line> generateRenderLine(const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& end, const Ogre::ColourValue& color, const float scale);

  // destination
  std::shared_ptr<rviz_rendering::Arrow> destination_arrow_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_destination_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_destination_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_destination_;

  // suggested lane
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_reference_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_boundaries_;

  std::vector<std::shared_ptr<rviz_rendering::Arrow>> suggested_lane_reference_poses_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_reference_poses_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_suggested_lane_reference_poses_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_suggested_lane_reference_poses_;

  std::vector<std::shared_ptr<rviz_rendering::Line>> suggested_lane_reference_line_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_reference_line_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_suggested_lane_reference_line_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_suggested_lane_reference_line_;

  std::vector<std::shared_ptr<rviz_rendering::Shape>> suggested_lane_boundary_points_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_boundary_points_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_suggested_lane_boundary_points_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_suggested_lane_boundary_points_;

  std::vector<std::shared_ptr<rviz_rendering::Line>> suggested_lane_boundary_lines_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_boundary_lines_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_suggested_lane_boundary_lines_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_suggested_lane_boundary_lines_;

  // adjacent lanes
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lanes_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lanes_reference_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lanes_boundaries_;

  std::vector<std::shared_ptr<rviz_rendering::Arrow>> adjacent_lanes_reference_poses_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lanes_reference_poses_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_adjacent_lanes_reference_poses_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_adjacent_lanes_reference_poses_;

  std::vector<std::shared_ptr<rviz_rendering::Line>> adjacent_lanes_reference_line_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lanes_reference_line_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_adjacent_lanes_reference_line_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_adjacent_lanes_reference_line_;

  std::vector<std::shared_ptr<rviz_rendering::Shape>> adjacent_lanes_boundary_points_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lanes_boundary_points_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_adjacent_lanes_boundary_points_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_adjacent_lanes_boundary_points_;

  std::vector<std::shared_ptr<rviz_rendering::Line>> adjacent_lanes_boundary_lines_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_adjacent_lanes_boundary_lines_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_adjacent_lanes_boundary_lines_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_adjacent_lanes_boundary_lines_;

  // driveable space
  std::vector<std::shared_ptr<rviz_rendering::Shape>> drivable_space_points_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_driveable_space_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_driveable_space_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_driveable_space_;

  std::vector<std::shared_ptr<rviz_rendering::Line>> lane_change_lines_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_lane_change_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_lane_change_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_lane_change_;
};

}  // namespace displays
}  // namespace route_planning_msgs
