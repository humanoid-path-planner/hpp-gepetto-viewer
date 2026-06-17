#pragma once
#include <hpp_gepetto_viewer/msg/hpp_waypoint.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_default_plugins/displays/interactive_markers/interactive_marker_display.hpp>
#include <string>
#include <visualization_msgs/msg/interactive_marker_init.hpp>
#include <visualization_msgs/msg/interactive_marker_update.hpp>

#include "WaypointProperty.hpp"

namespace hpp {

namespace displays {
class WaypointDisplay
    : public rviz_default_plugins::displays::InteractiveMarkerDisplay {
  Q_OBJECT
 public:
  WaypointDisplay() = default;
  ~WaypointDisplay() = default;

  void onInitialize() override;

 private:
  void onWaypointEnabledChanged(const std::string& name, bool enabled);
  void onUpdateMessage(
      const visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr msg);

  std::map<std::string, std::unique_ptr<WaypointProperty>> waypoint_properties_;

  rviz_common::properties::Property* group_property_{nullptr};

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node_ptr_;

  rclcpp::Subscription<
      visualization_msgs::msg::InteractiveMarkerUpdate>::SharedPtr update_sub_;
  rclcpp::Publisher<hpp_gepetto_viewer::msg::HppWaypoint>::SharedPtr
      waypoint_visiblilty_pub_;
};

}  // namespace displays
}  // namespace hpp
