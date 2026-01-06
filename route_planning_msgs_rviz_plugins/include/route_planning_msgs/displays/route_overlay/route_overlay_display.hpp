#ifndef ROUTE_PLANNING_MSGS__DISPLAYS__ROUTE_OVERLAY__ROUTE_OVERLAY_DISPLAY_HPP_
#define ROUTE_PLANNING_MSGS__DISPLAYS__ROUTE_OVERLAY__ROUTE_OVERLAY_DISPLAY_HPP_

#include <rviz_common/ros_topic_display.hpp>
#include <rviz_2d_overlay_plugins/overlay_utils.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <route_planning_msgs/msg/route.hpp>
#include <QImage>
#include <QPainter>
#include <QColor>
#include <QFont>
#include <QPixmap>

namespace route_planning_msgs
{
namespace displays
{

class RouteOverlay 
  : public rviz_common::RosTopicDisplay<route_planning_msgs::msg::Route>
{
  Q_OBJECT

public:
  RouteOverlay();
  ~RouteOverlay() override = default;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void processMessage(route_planning_msgs::msg::Route::ConstSharedPtr msg) override;
  void update(float wall_dt, float ros_dt) override;
  
  void renderOverlay();

protected Q_SLOTS:
  void updateWidth();
  void updateHeight();
  void updateLeft();
  void updateTop();
  void updateBackgroundColor();
  void updateBackgroundAlpha();
  void updateLookaheadDistance();

private:
  rviz_2d_overlay_plugins::OverlayObject::SharedPtr overlay_;
  
  rviz_common::properties::IntProperty* width_property_;
  rviz_common::properties::IntProperty* height_property_;
  rviz_common::properties::IntProperty* left_property_;
  rviz_common::properties::IntProperty* top_property_;
  rviz_common::properties::ColorProperty* bg_color_property_;
  rviz_common::properties::FloatProperty* bg_alpha_property_;
  rviz_common::properties::FloatProperty* lookahead_distance_property_;
  
  int width_;
  int height_;
  int left_;
  int top_;
  QColor bg_color_;
  float bg_alpha_;
  float lookahead_distance_; // meters
  
  // PNG icons
  QPixmap icon_distance_;
  QPixmap icon_time_;
  QPixmap icon_speed_limit_;
  
  // Route data
  uint8_t current_speed_limit_;      // km/h
  uint8_t traffic_light_state_;      // 0=unknown, 1=green, 2=red
  bool has_traffic_light_;
  bool has_validity_stamp_;          // whether validity_stamp is set
  double remaining_distance_;        // meters
  double estimated_time_;            // seconds
  double traffic_light_time_remaining_; // seconds until signal change
  bool update_required_;
};

}  // namespace displays
}  // namespace route_planning_msgs

#endif  // ROUTE_PLANNING_MSGS__DISPLAYS__ROUTE_OVERLAY__ROUTE_OVERLAY_DISPLAY_HPP_
