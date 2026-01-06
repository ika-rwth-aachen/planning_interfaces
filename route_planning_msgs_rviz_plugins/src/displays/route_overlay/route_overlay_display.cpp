#include "route_planning_msgs/displays/route_overlay/route_overlay_display.hpp"
#include <rviz_rendering/render_system.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QPainter>
#include <QFontMetrics>

namespace route_planning_msgs
{
namespace displays
{

RouteOverlay::RouteOverlay()
  : width_(135)
  , height_(125)
  , left_(10)
  , top_(10)
  , bg_color_(20, 20, 20)
  , bg_alpha_(0.9)
  , lookahead_distance_(100.0)
  , current_speed_limit_(0)
  , traffic_light_state_(0)
  , has_traffic_light_(false)
  , has_validity_stamp_(false)
  , remaining_distance_(0.0)
  , estimated_time_(0.0)
  , traffic_light_time_remaining_(0.0)
  , update_required_(false)
{
  width_property_ = new rviz_common::properties::IntProperty(
    "Width", width_, "Width of the overlay", this, SLOT(updateWidth()));
  width_property_->setMin(50);
  
  height_property_ = new rviz_common::properties::IntProperty(
    "Height", height_, "Height of the overlay", this, SLOT(updateHeight()));
  height_property_->setMin(50);
  
  left_property_ = new rviz_common::properties::IntProperty(
    "Left", left_, "Left position of the overlay", this, SLOT(updateLeft()));
  left_property_->setMin(0);
  
  top_property_ = new rviz_common::properties::IntProperty(
    "Top", top_, "Top position of the overlay", this, SLOT(updateTop()));
  top_property_->setMin(0);
  
  bg_color_property_ = new rviz_common::properties::ColorProperty(
    "Background Color", bg_color_, "Background color", this, SLOT(updateBackgroundColor()));
  
  bg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Background Alpha", bg_alpha_, "Background transparency", this, SLOT(updateBackgroundAlpha()));
  bg_alpha_property_->setMin(0.0);
  bg_alpha_property_->setMax(1.0);
  
  lookahead_distance_property_ = new rviz_common::properties::FloatProperty(
    "Lookahead Distance", lookahead_distance_, "Distance in meters to search for traffic lights", this, SLOT(updateLookaheadDistance()));
  lookahead_distance_property_->setMin(10.0);
  lookahead_distance_property_->setMax(500.0);
}

void RouteOverlay::onInitialize()
{
  rviz_common::RosTopicDisplay<route_planning_msgs::msg::Route>::onInitialize();
  
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  
  static int count = 0;
  std::string overlay_name = "RouteOverlayDisplayOverlay" + std::to_string(count++);
  overlay_ = std::make_shared<rviz_2d_overlay_plugins::OverlayObject>(overlay_name);
  
  overlay_->updateTextureSize(width_, height_);
  overlay_->setDimensions(width_, height_);
  overlay_->setPosition(left_, top_, rviz_2d_overlay_plugins::HorizontalAlignment::RIGHT, rviz_2d_overlay_plugins::VerticalAlignment::TOP);
  
  QString package_path = QString::fromStdString(ament_index_cpp::get_package_share_directory("route_planning_msgs_rviz_plugins"));
  icon_distance_.load(package_path + "/assets/route_distance.png");
  icon_time_.load(package_path + "/assets/route_time.png");
  icon_speed_limit_.load(package_path + "/assets/route_speed_limit.png");
  
  update_required_ = true;
}

void RouteOverlay::onEnable()
{
  if (overlay_) {
    overlay_->show();
  }
  update_required_ = true;
}

void RouteOverlay::onDisable()
{
  if (overlay_) {
    overlay_->hide();
  }
}

void RouteOverlay::processMessage(route_planning_msgs::msg::Route::ConstSharedPtr msg)
{
  try {
    current_speed_limit_ = 0;
    has_traffic_light_ = false;
    has_validity_stamp_ = false;
    traffic_light_state_ = 0;
    remaining_distance_ = 0.0;
    traffic_light_time_remaining_ = 0.0;
    
    if (!msg) {
      return;
    }
    
    if (msg->route_elements.empty()) {
      return;
    }
    
    // Get current route element index with bounds checking
    uint64_t current_idx = msg->current_route_element_idx;
    uint64_t destination_idx = msg->destination_route_element_idx;
    
    // Validate indices
    if (current_idx >= msg->route_elements.size()) {
      return;
    }
    
    // Clamp destination to valid range
    if (destination_idx >= msg->route_elements.size()) {
      destination_idx = msg->route_elements.size() - 1;
    }
    
    // Get speed limit from current route element
    const auto& current_element = msg->route_elements[current_idx];
    
    if (!current_element.lane_elements.empty()) {
      current_speed_limit_ = current_element.lane_elements[0].speed_limit;
    }
    
    // Get current time
    auto current_time = this->context_->getClock()->now();
    
    // Search for traffic light in lookahead distance
    double accumulated_distance = 0.0;
    for (uint64_t i = current_idx; i < msg->route_elements.size() && i <= destination_idx; ++i) {
      const auto& element = msg->route_elements[i];
      accumulated_distance += element.s;
      
      if (accumulated_distance > lookahead_distance_) {
        break;
      }
      
      // Check regulatory elements for traffic light
      for (const auto& reg_elem : element.regulatory_elements) {
        if (reg_elem.type == route_planning_msgs::msg::RegulatoryElement::TYPE_TRAFFIC_LIGHT) {
          has_traffic_light_ = true;
          // meta_value: 1=green/allowed, 2=red/restricted
          if (reg_elem.meta_value == route_planning_msgs::msg::RegulatoryElement::META_VALUE_MOVEMENT_ALLOWED) {
            traffic_light_state_ = 1; // green
          } else if (reg_elem.meta_value == route_planning_msgs::msg::RegulatoryElement::META_VALUE_MOVEMENT_RESTRICTED) {
            traffic_light_state_ = 2; // red
          }
          
          // Calculate time until signal change if validity_stamp is set
          has_validity_stamp_ = reg_elem.has_validity_stamp;
          if (reg_elem.has_validity_stamp) {
            rclcpp::Time validity_time(reg_elem.validity_stamp);
            rclcpp::Duration time_diff = validity_time - current_time;
            traffic_light_time_remaining_ = time_diff.seconds();
          }
          break;
        }
      }
      
      if (has_traffic_light_) break;
    }
    
    // Calculate remaining distance to destination
    // s is cumulative distance, so take difference between destination and current
    if (destination_idx < msg->route_elements.size() && current_idx < msg->route_elements.size()) {
      remaining_distance_ = msg->route_elements[destination_idx].s - msg->route_elements[current_idx].s;
    }
    
    // Estimate time to destination (assuming average speed of 50 km/h if no speed limit)
    double avg_speed_mps = (current_speed_limit_ > 0 ? current_speed_limit_ : 50.0) / 3.6; // km/h to m/s
    if (avg_speed_mps > 0) {
      // multiply by 2.5 since average speed over route is typically lower than speed limit
      estimated_time_ = 2.5 * remaining_distance_ / avg_speed_mps;
    }
    
    update_required_ = true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("RouteOverlay"), "Error processing route message: %s", e.what());
  }
}

void RouteOverlay::update(float wall_dt, float ros_dt)
{
  rviz_common::RosTopicDisplay<route_planning_msgs::msg::Route>::update(wall_dt, ros_dt);
  
  if (update_required_) {
    renderOverlay();
    update_required_ = false;
  }
}

void RouteOverlay::renderOverlay()
{
  if (!overlay_) {
    return;
  }
  
  if (!overlay_->isVisible()) {
    return;
  }
  
  // Dynamic height: 3 lines (85px) without traffic light, 4 lines (105px) with traffic light
  int display_height = has_traffic_light_ ? 105 : 85;
  
  overlay_->updateTextureSize(width_, display_height);
  overlay_->setDimensions(width_, display_height);
  
  rviz_2d_overlay_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage image = buffer.getQImage(width_, display_height);
  image.fill(Qt::transparent);
  
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::TextAntialiasing, true);
  
  // Background
  QColor bg_with_alpha = bg_color_;
  bg_with_alpha.setAlphaF(bg_alpha_);
  painter.setPen(QPen(QColor(80, 80, 80), 1));
  painter.setBrush(bg_with_alpha);
  painter.drawRoundedRect(0, 0, width_, display_height, 15, 15);
  
  QFont valueFont("Arial", 11, QFont::Bold);
  QFontMetrics fm(valueFont);
  
  int x_icon = 10;
  int x_value = 35;
  int x_unit = 85; // Fixed position for units (wider to accommodate 3-digit numbers)
  int line_height = 25;
  
  painter.setPen(QColor(220, 220, 220));
  
  // SECTION 1: Distance
  int y = 22;
  painter.drawPixmap(x_icon, y - 16, 20, 20, icon_distance_);
  painter.setFont(valueFont);
  if (remaining_distance_ >= 1000.0) {
    QString value = QString::number(remaining_distance_ / 1000.0, 'f', 1);
    painter.drawText(x_value, y, value);
    painter.drawText(x_unit, y, "km");
  } else {
    QString value = QString::number(static_cast<int>(remaining_distance_));
    painter.drawText(x_value, y, value);
    painter.drawText(x_unit, y, "m");
  }
  y += line_height;
  
  // SECTION 2: Time
  painter.drawPixmap(x_icon, y - 16, 20, 20, icon_time_);
  painter.setFont(valueFont);
  QString time_value = QString::number(estimated_time_, 'f', 1);
  painter.drawText(x_value, y, time_value);
  painter.drawText(x_unit, y, "s");
  y += line_height;
  
  // SECTION 3: Speed limit
  painter.drawPixmap(x_icon, y - 16, 20, 20, icon_speed_limit_);
  painter.setFont(valueFont);
  QString speed_value = QString::number(current_speed_limit_);
  painter.drawText(x_value, y, speed_value);
  painter.drawText(x_unit, y, "km/h");
  y += line_height;
  
  // SECTION 4: Traffic light state (only circle and time, no icon)
  if (has_traffic_light_) {
    if (traffic_light_state_ == 1) {
      // Green
      painter.setBrush(QColor(0, 255, 0));
      painter.setPen(Qt::NoPen);
      painter.drawEllipse(x_icon + 2, y - 18, 16, 16);
      
      painter.setPen(QColor(220, 220, 220));
      painter.setFont(valueFont);
      if (has_validity_stamp_) {
        QString value = QString::number(traffic_light_time_remaining_, 'f', 1);
        painter.drawText(x_value, y - 4, value);
        painter.drawText(x_unit, y - 4, "s");
      } else {
        painter.drawText(x_value, y - 6, "–");
      }
    } else if (traffic_light_state_ == 2) {
      // Red
      painter.setBrush(QColor(255, 0, 0));
      painter.setPen(Qt::NoPen);
      painter.drawEllipse(x_icon + 2, y - 18, 16, 16);
      
      painter.setPen(QColor(220, 220, 220));
      painter.setFont(valueFont);
      if (has_validity_stamp_) {
        QString value = QString::number(traffic_light_time_remaining_, 'f', 1);
        painter.drawText(x_value, y - 4, value);
        painter.drawText(x_unit, y - 4, "s");
      } else {
        painter.drawText(x_value, y - 6, "–");
      }
    }
  }
  
  painter.end();
}

void RouteOverlay::updateWidth()
{
  width_ = width_property_->getInt();
  if (overlay_) {
    overlay_->setDimensions(width_, height_);
  }
  update_required_ = true;
}

void RouteOverlay::updateHeight()
{
  height_ = height_property_->getInt();
  if (overlay_) {
    overlay_->setDimensions(width_, height_);
  }
  update_required_ = true;
}

void RouteOverlay::updateLeft()
{
  left_ = left_property_->getInt();
  if (overlay_) {
    overlay_->setPosition(left_, top_, rviz_2d_overlay_plugins::HorizontalAlignment::RIGHT, rviz_2d_overlay_plugins::VerticalAlignment::TOP);
  }
  update_required_ = true;
}

void RouteOverlay::updateTop()
{
  top_ = top_property_->getInt();
  if (overlay_) {
    overlay_->setPosition(left_, top_, rviz_2d_overlay_plugins::HorizontalAlignment::RIGHT, rviz_2d_overlay_plugins::VerticalAlignment::TOP);
  }
  update_required_ = true;
}

void RouteOverlay::updateBackgroundColor()
{
  bg_color_ = bg_color_property_->getColor();
  update_required_ = true;
}

void RouteOverlay::updateBackgroundAlpha()
{
  bg_alpha_ = bg_alpha_property_->getFloat();
  update_required_ = true;
}

void RouteOverlay::updateLookaheadDistance()
{
  lookahead_distance_ = lookahead_distance_property_->getFloat();
  update_required_ = true;
}

}  // namespace displays
}  // namespace route_planning_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(route_planning_msgs::displays::RouteOverlay, rviz_common::Display)
