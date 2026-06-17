#pragma once

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/quaternion_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/vector_property.hpp>

namespace hpp {

class WaypointProperty : public QObject {
  Q_OBJECT
 public:
  WaypointProperty(
      const QString& name, const QString& description,
      const std::string& parent_frame = "world",
      const Ogre::Vector3& position = Ogre::Vector3::ZERO,
      const Ogre::Quaternion& orientation = Ogre::Quaternion::IDENTITY,
      rviz_common::properties::Property* parent = nullptr) {
    group_property_ = new rviz_common::properties::BoolProperty(
        name, true, description, parent);
    name_ = name.toStdString();

    parent_property_ = new rviz_common::properties::StringProperty(
        "Parent", QString::fromStdString(parent_frame),
        "Parent frame of this waypoint. (Not editable)", group_property_);
    parent_property_->setReadOnly(true);

    position_property_ = new rviz_common::properties::VectorProperty(
        "Position", position,
        "Position of this waypoint in its parent frame. (Not editable)",
        group_property_);
    position_property_->setReadOnly(true);

    orientation_property_ = new rviz_common::properties::QuaternionProperty(
        "Orientation", orientation,
        "Orientation of this waypoint in its parent frame. (Not editable)",
        group_property_);
    orientation_property_->setReadOnly(true);

    connect(group_property_, &rviz_common::properties::Property::changed, this,
            [this]() {
              emit enabledChanged(name_.c_str(), group_property_->getBool());
            });
  };
  ~WaypointProperty() = default;

  void setPosition(const Ogre::Vector3& position) {
    position_property_->setVector(position);
  }
  void setOrientation(const Ogre::Quaternion& orientation) {
    orientation_property_->setQuaternion(orientation);
  }
  void enable(bool visible) { group_property_->setValue(visible); }
  void removeFromParent() {
    if (group_property_ && group_property_->getParent()) {
      group_property_->getParent()->takeChild(group_property_);
    }
  }

 signals:
  void enabledChanged(const std::string& name, bool enabled);

 private:
  std::string name_;
  rviz_common::properties::BoolProperty* group_property_{nullptr};
  rviz_common::properties::StringProperty* parent_property_{nullptr};
  rviz_common::properties::VectorProperty* position_property_{nullptr};
  rviz_common::properties::QuaternionProperty* orientation_property_{nullptr};
};
}  // namespace hpp
