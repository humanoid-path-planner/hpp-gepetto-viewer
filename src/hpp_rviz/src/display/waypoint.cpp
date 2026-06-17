#include "../../include/hpp/display/waypoint.hpp"

#include <QMetaObject>
#include <QStringList>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/property.hpp>

namespace hpp {
namespace displays {

void WaypointDisplay::onInitialize() {
  rviz_default_plugins::displays::InteractiveMarkerDisplay::onInitialize();

  group_property_ = new rviz_common::properties::Property(
      "Waypoints", QVariant(), "Waypoints description", this);

  const std::string server_topic = "/hpp_waypoint_server";
  setTopic(QString::fromStdString(server_topic), "");

  auto* show_axes = findProperty("Show Axes");
  if (show_axes) {
    show_axes->setValue(true);
    show_axes->hide();
  }

  // Cacher d'autres options du parent
  auto* show_visual_aids = findProperty("Show Visual Aids");
  if (show_visual_aids) show_visual_aids->hide();

  auto* enable_transparency = findProperty("Enable Transparency");
  if (enable_transparency) enable_transparency->hide();

  node_ptr_ = context_->getRosNodeAbstraction();
  auto node_lock = node_ptr_.lock();
  if (!node_lock) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Waypoint",
              "Unable to access ROS node.");
    return;
  }
  auto node = node_lock->get_raw_node();

  auto update_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
  update_sub_ = node->create_subscription<
      visualization_msgs::msg::InteractiveMarkerUpdate>(
      server_topic + "/update", update_qos,
      std::bind(&WaypointDisplay::onUpdateMessage, this,
                std::placeholders::_1));

  waypoint_visiblilty_pub_ =
      node->create_publisher<hpp_gepetto_viewer::msg::HppWaypoint>(
          server_topic + "/waypoint_visibility", 10);

  waypoint_visiblilty_pub_ =
      node->create_publisher<hpp_gepetto_viewer::msg::HppWaypoint>(
          server_topic + "/waypoint_visibility", 10);

  setStatus(rviz_common::properties::StatusProperty::Ok, "Waypoint",
            "Listening for interactive marker updates.");
}

void WaypointDisplay::onWaypointEnabledChanged(const std::string& name,
                                               bool enabled) {
  RCUTILS_LOG_INFO("Waypoint '%s' enabled changed to %s", name.c_str(),
                   enabled ? "true" : "false");
  auto it = waypoint_properties_.find(name);
  if (it == waypoint_properties_.end()) return;
  hpp_gepetto_viewer::msg::HppWaypoint msg;
  msg.name = name;
  msg.enable = enabled;
  waypoint_visiblilty_pub_->publish(msg);
}

void WaypointDisplay::onUpdateMessage(
    const visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr msg) {
  QMetaObject::invokeMethod(
      this,
      [this, msg]() {
        for (const auto& marker : msg->markers) {
          if (waypoint_properties_.count(marker.name)) {
            continue;
          }
          auto wp = std::make_unique<WaypointProperty>(
              QString::fromStdString(marker.name),
              QString::fromStdString("Waypoint " + marker.name),
              marker.header.frame_id,
              Ogre::Vector3((float)marker.pose.position.x,
                            (float)marker.pose.position.y,
                            (float)marker.pose.position.z),
              Ogre::Quaternion((float)marker.pose.orientation.w,
                               (float)marker.pose.orientation.x,
                               (float)marker.pose.orientation.y,
                               (float)marker.pose.orientation.z),
              group_property_);

          connect(wp.get(), &WaypointProperty::enabledChanged, this,
                  &WaypointDisplay::onWaypointEnabledChanged);

          waypoint_properties_.emplace(marker.name, std::move(wp));
        }

        for (const auto& erased_name : msg->erases) {
          auto it = waypoint_properties_.find(erased_name);
          if (it != waypoint_properties_.end()) {
            it->second->removeFromParent();
            waypoint_properties_.erase(it);
          }
        }

        for (const auto& marker : msg->poses) {
          auto it = waypoint_properties_.find(marker.name);
          if (it != waypoint_properties_.end()) {
            it->second->setPosition(Ogre::Vector3(
                (float)marker.pose.position.x, (float)marker.pose.position.y,
                (float)marker.pose.position.z));
            it->second->setOrientation(
                Ogre::Quaternion((float)marker.pose.orientation.w,
                                 (float)marker.pose.orientation.x,
                                 (float)marker.pose.orientation.y,
                                 (float)marker.pose.orientation.z));
          }
        }
      },
      Qt::QueuedConnection);
}

}  // namespace displays
}  // namespace hpp

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hpp::displays::WaypointDisplay, rviz_common::Display)
