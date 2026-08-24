// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <memory>

#include <route_planning_msgs/msg/reference_line.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

namespace route_planning_msgs::displays {

class ReferenceLineDisplay : public rviz_common::MessageFilterDisplay<
                                 route_planning_msgs::msg::ReferenceLine> {
  Q_OBJECT

protected:
  void onInitialize() override;
  void processMessage(
      const route_planning_msgs::msg::ReferenceLine::ConstSharedPtr msg)
      override;
  void reset() override;

private:
  std::shared_ptr<rviz_rendering::BillboardLine> line_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_;
  std::unique_ptr<rviz_common::properties::FloatProperty> line_width_property_;
};

} // namespace route_planning_msgs::displays
