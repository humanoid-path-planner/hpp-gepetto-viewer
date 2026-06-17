#pragma once

#include <OgreVector3.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <hpp_gepetto_viewer/msg/hpp_waypoint.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/selection_manager.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/tool.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>

#include "interactiveWaypoint.hpp"

namespace hpp {
namespace tool {

class Waypoint : public rviz_common::Tool {
  Q_OBJECT
 public:
  Waypoint();
  ~Waypoint();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

 private:
  std::map<std::string, std::unique_ptr<InteractiveWaypoint>>
      interactive_waypoints_;
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node_ptr_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  interactive_markers::MenuHandler::EntryHandle edit_menu_handle_;
  interactive_markers::MenuHandler::EntryHandle delete_menu_handle_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      waypoint_sub_;
  rclcpp::Subscription<hpp_gepetto_viewer::msg::HppWaypoint>::SharedPtr
      waypoint_visibility_sub_;
  int waypoint_count_ = 0;

  void onWaypointVisibilityReceived(
      const hpp_gepetto_viewer::msg::HppWaypoint::SharedPtr msg);
  void createInteractiveWaypoint(const geometry_msgs::msg::PoseStamped& pos);
  void processFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&
          feedback);
  void onWaypointReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

}  // namespace tool
}  // namespace hpp
