#pragma once

#include <route_planning_msgs/msg/route.hpp>
#include <route_planning_msgs_utils/route_access.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/bool_property.hpp>

#include <rviz_rendering/objects/shape.hpp>

namespace route_planning_msgs {
namespace displays {

/**
 * \class NewRouteDisplay
 * \brief Displays a route_planning_msgs::Route message
 */
class NewRouteDisplay : public rviz_common::MessageFilterDisplay<route_planning_msgs::msg::Route> {
  Q_OBJECT

private Q_SLOTS:
  void updateStyle();

 protected:
  void onInitialize() override;
  void processMessage(const route_planning_msgs::msg::Route::ConstSharedPtr msg) override;

  void displaySuggestedLanePoints(const std::vector<route_planning_msgs::msg::RouteElement>& route_elements, const Ogre::ColourValue& color, float scale);

  std::vector<std::shared_ptr<rviz_rendering::Shape>> suggested_lane_points_;
  std::unique_ptr<rviz_common::properties::BoolProperty> viz_suggested_lane_;
  std::unique_ptr<rviz_common::properties::FloatProperty> scale_property_suggested_lane_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_suggested_lane_;

};

}  // namespace displays
}  // namespace route_planning_msgs
