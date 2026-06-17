#pragma once
#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/selection_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <string>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace hpp {

class InteractiveWaypoint : public visualization_msgs::msg::InteractiveMarker {
 public:
  InteractiveWaypoint() = default;
  InteractiveWaypoint(const Ogre::Vector3& pos, const Ogre::Quaternion& orient,
                      const std::string& name = "waypoint",
                      const std::string& description = "hpp waypoint") {
    this->header.frame_id = "world";
    this->scale = 0.2;
    this->name = name;
    this->description = description;

    this->pose.position.x = pos.x;
    this->pose.position.y = pos.y;
    this->pose.position.z = pos.z;
    this->pose.orientation.x = orient.x;
    this->pose.orientation.y = orient.y;
    this->pose.orientation.z = orient.z;
    this->pose.orientation.w = orient.w;

    visualization_msgs::msg::InteractiveMarkerControl visual_control;
    visual_control.always_visible = true;

    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    visual_control.markers.push_back(marker);
    this->controls.push_back(visual_control);

    addAxisControl(
        "rotate_x", 0.70710678, 0.0, 0.0, 0.70710678,
        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS);
    addAxisControl(
        "move_x", 0.70710678, 0.0, 0.0, 0.70710678,
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS);
    addAxisControl(
        "rotate_y", 0.0, 0.70710678, 0.0, 0.70710678,
        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS);
    addAxisControl(
        "move_y", 0.0, 0.70710678, 0.0, 0.70710678,
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS);
    addAxisControl(
        "rotate_z", 0.0, 0.0, 0.70710678, 0.70710678,
        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS);
    addAxisControl(
        "move_z", 0.0, 0.0, 0.70710678, 0.70710678,
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS);
  }

  bool isActivated() const { return activated_; }
  Ogre::Vector3 getPosition() const {
    return Ogre::Vector3(this->pose.position.x, this->pose.position.y,
                         this->pose.position.z);
  }
  Ogre::Quaternion getOrientation() const {
    return Ogre::Quaternion(this->pose.orientation.w, this->pose.orientation.x,
                            this->pose.orientation.y, this->pose.orientation.z);
  }

  void setPosition(const Ogre::Vector3& pos, const Ogre::Quaternion& orient) {
    this->pose.position.x = pos.x;
    this->pose.position.y = pos.y;
    this->pose.position.z = pos.z;
    this->pose.orientation.x = orient.x;
    this->pose.orientation.y = orient.y;
    this->pose.orientation.z = orient.z;
    this->pose.orientation.w = orient.w;
  }

 private:
  bool activated_ = false;
  void addAxisControl(const std::string& name, double ox, double oy, double oz,
                      double ow, uint8_t interaction_mode) {
    visualization_msgs::msg::InteractiveMarkerControl control;

    control.name = name;
    control.orientation.x = ox;
    control.orientation.y = oy;
    control.orientation.z = oz;
    control.orientation.w = ow;
    control.orientation_mode =
        visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    control.interaction_mode = interaction_mode;
    this->controls.push_back(control);
  };
};

}  // namespace hpp
