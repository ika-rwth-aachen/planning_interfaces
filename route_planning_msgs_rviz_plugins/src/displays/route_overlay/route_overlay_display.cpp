#include "route_planning_msgs/displays/route_overlay/route_overlay_display.hpp"
#include <QPainter>
#include <QFontMetrics>
#include <QGuiApplication>
#include <QScreen>
#include <rviz_rendering/render_system.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace route_planning_msgs
{
namespace displays
{

RouteOverlay::RouteOverlay()
  : width_(135)
  , height_(125)
  , right_(10)
  , top_(10)
  , bg_color_(20, 20, 20)
  , bg_alpha_(0.9)
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
  
  right_property_ = new rviz_common::properties::IntProperty(
    "Right", right_, "Right position of the overlay", this, SLOT(updateRight()));
  right_property_->setMin(0);

  top_property_ = new rviz_common::properties::IntProperty(
    "Top", top_, "Top position of the overlay", this, SLOT(updateTop()));
  top_property_->setMin(0);
  
  bg_color_property_ = new rviz_common::properties::ColorProperty(
    "Background Color", bg_color_, "Background color", this, SLOT(updateBackgroundColor()));
  
  bg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Background Alpha", bg_alpha_, "Background transparency", this, SLOT(updateBackgroundAlpha()));
  bg_alpha_property_->setMin(0.0);
  bg_alpha_property_->setMax(1.0);
  
}

void RouteOverlay::onInitialize()
{
  rviz_common::RosTopicDisplay<route_planning_msgs::msg::Route>::onInitialize();
  
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  
  static int count = 0;
  std::string overlay_name = "RouteOverlayDisplayOverlay" + std::to_string(count++);
  overlay_ = std::make_shared<rviz_2d_overlay_plugins::OverlayObject>(overlay_name);
  
  // DPI-based scaling
  QScreen* screen = QGuiApplication::primaryScreen();
  qreal dpi = screen ? screen->logicalDotsPerInch() : 96.0;
  double overlay_width_mm = 135.0 * 25.4 / dpi;
  double overlay_height_mm = 125.0 * 25.4 / dpi;
  width_ = static_cast<int>(overlay_width_mm * dpi / 25.4);
  height_ = static_cast<int>(overlay_height_mm * dpi / 25.4);
  overlay_->updateTextureSize(width_, height_);
  overlay_->setDimensions(width_, height_);
  overlay_->setPosition(right_, top_, rviz_2d_overlay_plugins::HorizontalAlignment::RIGHT, rviz_2d_overlay_plugins::VerticalAlignment::TOP);
  
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
  current_speed_limit_ = 0;
  has_traffic_light_ = false;
  has_validity_stamp_ = false;
  traffic_light_state_ = 0;
  remaining_distance_ = 0.0;
  traffic_light_time_remaining_ = 0.0;

  if (!msg || msg->route_elements.empty())
    return;

  uint64_t current_idx = msg->current_route_element_idx;
  uint64_t destination_idx = msg->destination_route_element_idx;
  if (current_idx >= msg->route_elements.size())
    return;

  current_speed_limit_ = route_planning_msgs::route_access::getSuggestedLaneElement(msg->route_elements[current_idx]).speed_limit;

  auto current_time = this->context_->getClock()->now();

  // Use getRemainingRouteElements to iterate all remaining RouteElements from the current position
  struct TLInfo {
    int state;
    bool has_validity_stamp;
    double time_remaining;
  };
  std::vector<TLInfo> traffic_lights;
  auto remaining_elements = route_planning_msgs::route_access::getRemainingRouteElements(*msg);
  for (const auto& element : remaining_elements) {
    if (!element.is_enriched)
      break;
    const auto regs = route_planning_msgs::route_access::getRegulatoryElementsOfSuggestedLane(element);
    for (const auto& reg_elem : regs) {
      if (reg_elem.type == route_planning_msgs::msg::RegulatoryElement::TYPE_TRAFFIC_LIGHT) {
        int state = (reg_elem.meta_value == route_planning_msgs::msg::RegulatoryElement::META_VALUE_MOVEMENT_ALLOWED) ? 1 :
                    (reg_elem.meta_value == route_planning_msgs::msg::RegulatoryElement::META_VALUE_MOVEMENT_RESTRICTED) ? 2 : 0;
        double time_remaining = 0.0;
        if (reg_elem.has_validity_stamp) {
          rclcpp::Time validity_time(reg_elem.validity_stamp);
          rclcpp::Duration time_diff = validity_time - current_time;
          time_remaining = time_diff.seconds();
        }
        traffic_lights.push_back({state, reg_elem.has_validity_stamp, time_remaining});
      }
    }
  }

  // Prefer the first traffic light with a known state (not unknown)
  auto it = std::find_if(traffic_lights.begin(), traffic_lights.end(), [](const TLInfo& tl) { return tl.state != 0; });
  TLInfo* best = nullptr;
  if (it != traffic_lights.end()) {
    best = &(*it);
  } else if (!traffic_lights.empty()) {
    best = &traffic_lights.front(); // fallback: show the first (even if unknown)
  }

  if (best) {
    has_traffic_light_ = true;
    traffic_light_state_ = best->state;
    has_validity_stamp_ = best->has_validity_stamp;
    traffic_light_time_remaining_ = best->time_remaining;
  } else {
    has_traffic_light_ = false;
    traffic_light_state_ = 0;
    has_validity_stamp_ = false;
    traffic_light_time_remaining_ = 0.0;
  }

  if (destination_idx < msg->route_elements.size() && current_idx < msg->route_elements.size())
    remaining_distance_ = msg->route_elements[destination_idx].s - msg->route_elements[current_idx].s;

  double avg_speed_mps = (current_speed_limit_ > 0 ? current_speed_limit_ : 50.0) / 3.6;
  estimated_time_ = (avg_speed_mps > 0) ? 2.5 * remaining_distance_ / avg_speed_mps : 0.0;

  update_required_ = true;
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
  
  // DPI-based scaling for all elements
  int display_height = has_traffic_light_ ? static_cast<int>(height_ * 0.84) : static_cast<int>(height_ * 0.68);
  overlay_->updateTextureSize(width_, display_height);
  overlay_->setDimensions(width_, display_height);
  rviz_2d_overlay_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage image = buffer.getQImage(width_, display_height);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::TextAntialiasing, true);
  QColor bg_with_alpha = bg_color_;
  bg_with_alpha.setAlphaF(bg_alpha_);
  painter.setPen(QPen(QColor(80, 80, 80), 1));
  painter.setBrush(bg_with_alpha);
  int radius = static_cast<int>(height_ * 0.12);
  painter.drawRoundedRect(0, 0, width_, display_height, radius, radius);
  QFont valueFont("Arial", static_cast<int>(height_ * 0.09), QFont::Bold);
  QFontMetrics fm(valueFont);
  int x_icon = static_cast<int>(height_ * 0.08);
  int icon_size = static_cast<int>(height_ * 0.16);
  int x_value = static_cast<int>(height_ * 0.28);
  int x_unit = static_cast<int>(height_ * 0.68);
  int line_height = static_cast<int>(height_ * 0.20);
  painter.setPen(QColor(220, 220, 220));
  
  // Section 1: Remaining Distance
  int y = static_cast<int>(height_ * 0.18);
  painter.drawPixmap(x_icon, y - icon_size * 0.8, icon_size, icon_size, icon_distance_);
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

  // Section 2: Estimated Time to Destination
  painter.drawPixmap(x_icon, y - icon_size * 0.8, icon_size, icon_size, icon_time_);
  painter.setFont(valueFont);
  QString time_value = QString::number(estimated_time_, 'f', 1);
  painter.drawText(x_value, y, time_value);
  painter.drawText(x_unit, y, "s");
  y += line_height;

  // Section 3: Speed Limit
  painter.drawPixmap(x_icon, y - icon_size * 0.8, icon_size, icon_size, icon_speed_limit_);
  painter.setFont(valueFont);
  QString speed_value = QString::number(current_speed_limit_);
  painter.drawText(x_value, y, speed_value);
  painter.drawText(x_unit, y, "km/h");
  y += line_height;

  // Section 4: Traffic Light State
  if (has_traffic_light_) {
    int circle_size = static_cast<int>(height_ * 0.13);
    int circle_x = x_icon + icon_size / 4 - static_cast<int>(height_ * 0.03);
    int circle_y = (y - icon_size * 0.8) + (icon_size - circle_size) / 2 - static_cast<int>(height_ * 0.01);
    int value_y = y;
    if (traffic_light_state_ == 1) {
      // Green
      painter.setBrush(QColor(0, 255, 0));
      painter.setPen(Qt::NoPen);
      painter.drawEllipse(circle_x, circle_y, circle_size, circle_size);
      painter.setPen(QColor(220, 220, 220));
      painter.setFont(valueFont);
      if (has_validity_stamp_) {
        QString value = QString::number(traffic_light_time_remaining_, 'f', 1);
        painter.drawText(x_value, value_y - 1, value);
        painter.drawText(x_unit, value_y - 1, "s");
      } else {
        painter.drawText(x_value, value_y - 3, "–");
      }
    } else if (traffic_light_state_ == 2) {
      // Red
      painter.setBrush(QColor(255, 0, 0));
      painter.setPen(Qt::NoPen);
      painter.drawEllipse(circle_x, circle_y, circle_size, circle_size);
      painter.setPen(QColor(220, 220, 220));
      painter.setFont(valueFont);
      if (has_validity_stamp_) {
        QString value = QString::number(traffic_light_time_remaining_, 'f', 1);
        painter.drawText(x_value, value_y - 1, value);
        painter.drawText(x_unit, value_y - 1, "s");
      } else {
        painter.drawText(x_value, value_y - 3, "–");
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

void RouteOverlay::updateRight()
{
  right_ = right_property_->getInt();
  if (overlay_) {
    overlay_->setPosition(right_, top_, rviz_2d_overlay_plugins::HorizontalAlignment::RIGHT, rviz_2d_overlay_plugins::VerticalAlignment::TOP);
  }
  update_required_ = true;
}

void RouteOverlay::updateTop()
{
  top_ = top_property_->getInt();
  if (overlay_) {
    overlay_->setPosition(right_, top_, rviz_2d_overlay_plugins::HorizontalAlignment::RIGHT, rviz_2d_overlay_plugins::VerticalAlignment::TOP);
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


}  // namespace displays
}  // namespace route_planning_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(route_planning_msgs::displays::RouteOverlay, rviz_common::Display)
