#include "../../include/hpp/panel/trajectory_slider.hpp"

namespace hpp {
namespace panel {

void TrajectorySlider::onInitialize() {
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction();

  rclcpp::Node::SharedPtr node = node_ptr_.lock()->get_raw_node();

  time_pub_ = node->create_publisher<hpp_gepetto_viewer::msg::PathInfo>(
      "/hpp/trajectory_time", 10);

  length_sub_ = node->create_subscription<hpp_gepetto_viewer::msg::PathInfo>(
      "/hpp/pathInfo", 10,
      [this](const hpp_gepetto_viewer::msg::PathInfo::SharedPtr msg) {
        onPathInfoReceive(msg);
      });

  scene_obj_sub_ = node->create_subscription<
      hpp_gepetto_viewer::msg::HppVectorConfiguration>(
      "/hpp/scene_objects", 10,
      [this](const hpp_gepetto_viewer::msg::HppVectorConfiguration::SharedPtr
                 msg) { onSceneObjReceive(msg); });

  target_frame_pub_ = node->create_publisher<hpp_gepetto_viewer::msg::PathInfo>(
      "/hpp/target_frame", 10);

  joint_value_pub_ =
      node->create_publisher<hpp_gepetto_viewer::msg::PinocchioJoint>(
          "/hpp/pinocchio_joints", 10);
}

void TrajectorySlider::onJointValueChanged(std::string name, double value) {
  hpp_gepetto_viewer::msg::PinocchioJoint msg;
  msg.name = name;
  msg.values.push_back(static_cast<float>(value));
  msg.type = "JOINT";
  msg.min = 0.0;
  msg.max = 0.0;
  joint_value_pub_->publish(msg);
}

void TrajectorySlider::onFreeFlyerValueChanged(std::string name, double value,
                                               int index) {
  freeflyerValues_[name][index] = value;

  /// QUATERNION NORMALIZATION
  if (index >= 3) {
    int primary_index = index - 3;
    std::vector<double> q = {
        freeflyerValues_[name][3], freeflyerValues_[name][4],
        freeflyerValues_[name][5], freeflyerValues_[name][6]};

    double rest = 1.0 - (q[primary_index] * q[primary_index]);
    if (rest < 0.0) rest = 0.0;

    double norm = 0.0;
    for (int i = 0; i < 4; ++i) {
      if (i == primary_index) continue;
      norm += q[i] * q[i];
    }
    norm = std::sqrt(norm);

    if (norm < 1e-9) {
      for (int i = 0; i < 4; ++i) {
        if (i == primary_index) continue;
        q[i] = (i == 0) ? std::sqrt(rest) : 0.0;
        break;
      }
    } else {
      double scale = std::sqrt(rest) / norm;
      for (int i = 0; i < 4; ++i) {
        if (i == primary_index) continue;
        q[i] *= scale;
      }
    }

    freeflyerValues_[name][3] = q[0];
    freeflyerValues_[name][4] = q[1];
    freeflyerValues_[name][5] = q[2];
    freeflyerValues_[name][6] = q[3];

    QTreeWidgetItem* grpFF = objPosInSceneTree_[name];
    for (int i = 0; i < 4; ++i) {
      if (i == primary_index) continue;
      QTreeWidgetItem* child = grpFF->child(3 + i);
      if (!child) continue;
      DoubleSlider* slider =
          child->data(1, Qt::UserRole).value<DoubleSlider*>();
      if (slider) {
        slider->blockSignals(true);
        slider->setValue(q[i]);
        slider->blockSignals(false);
      }
    }
  }

  hpp_gepetto_viewer::msg::PinocchioJoint msg;
  msg.name = name;
  msg.values = {static_cast<float>(freeflyerValues_[name][0]),
                static_cast<float>(freeflyerValues_[name][1]),
                static_cast<float>(freeflyerValues_[name][2]),
                static_cast<float>(freeflyerValues_[name][3]),
                static_cast<float>(freeflyerValues_[name][4]),
                static_cast<float>(freeflyerValues_[name][5]),
                static_cast<float>(freeflyerValues_[name][6])};
  msg.type = "FREE_FLYER";
  msg.min = 0.0;
  msg.max = 0.0;
  joint_value_pub_->publish(msg);
}

void TrajectorySlider::onPathInfoReceive(
    const hpp_gepetto_viewer::msg::PathInfo::SharedPtr msg) {
  path_length_ = msg->path_length;
  current_time_ = 0.0;
  is_playing_ = false;
  play_timer_->stop();

  // Active le slider
  slider_->setEnabled(path_length_ > 0.0);
  toogle_play_button_->setEnabled(path_length_ > 0.0);
  toogle_play_button_->setText("▶ Play");

  {
    QSignalBlocker blocker(slider_);
    slider_->setValue(0);
    t_spinbox_->setMaximum(path_length_);

    target_frame_selector_->clear();
    for (const auto& frameName : msg->frame_names) {
      target_frame_selector_->addItem(frameName.c_str());
    }
  }

  info_label_->setText(
      QString("Path length: %1 s").arg(path_length_, 0, 'f', 3));
  updateTimeDisplay();
  publishTime();
}

void TrajectorySlider::publishTime() {
  hpp_gepetto_viewer::msg::PathInfo msg;
  msg.current_time = current_time_;
  time_pub_->publish(msg);
}

void TrajectorySlider::updateTimeDisplay() {
  time_label_->setText(QString("t = %1 s / %2 s")
                           .arg(current_time_, 0, 'f', 3)
                           .arg(path_length_, 0, 'f', 3));

  {
    QSignalBlocker blocker(t_spinbox_);
    t_spinbox_->setValue(current_time_);
  }
}

QTreeWidgetItem* TrajectorySlider::getOrCreateNamespaceItem(
    QTreeWidgetItem* parent, const QString& ns) {
  // Cherche si le groupe existe déjà
  for (int i = 0; i < parent->childCount(); ++i) {
    if (parent->child(i)->text(0) == ns) return parent->child(i);
  }
  // Sinon on le crée
  auto* grp = new QTreeWidgetItem(parent, {ns});
  grp->setExpanded(true);
  return grp;
}

void TrajectorySlider::onSceneObjReceive(
    const hpp_gepetto_viewer::msg::HppVectorConfiguration::SharedPtr msg) {
  std::string vectorConfigurationInfo = "[";

  for (float val : msg->hpp_vector) {
    vectorConfigurationInfo += std::to_string(val) + ", ";
  }
  vectorConfigurationInfo.pop_back();
  vectorConfigurationInfo.pop_back();
  vectorConfigurationInfo += "]";
  hpp_vector_configuration_edit_->setPlainText(
      QString::fromStdString(vectorConfigurationInfo));

  for (const auto& joint : msg->joints) {
    std::string name = joint.name;
    float min = joint.min;
    float max = joint.max;

    // Convertir la string "[x, y, z, ...]" en vector<double>
    std::vector<double> pos = joint.values;

    // === PREMIÈRE FOIS : Création ===
    if (pos.size() == 7) {
      if (objPosInSceneTree_.find(name) == objPosInSceneTree_.end()) {
        QString qname = QString::fromStdString(name);
        QStringList parts = qname.split('/');
        QTreeWidgetItem* parent = grpJoints_;

        for (int i = 0; i < parts.size() - 1; ++i)
          parent = getOrCreateNamespaceItem(parent, parts[i]);

        // Groupe freeflyer (nœud parent, pas de slider)
        QString leafName = parts.last();
        auto* grpFF = new QTreeWidgetItem(parent, {leafName});
        grpFF->setExpanded(true);

        addFreeFlyerSliderItem(grpFF, name, min, max, pos);
        objPosInSceneTree_[name] = grpFF;
      } else {
        QTreeWidgetItem* grpFF = objPosInSceneTree_[name];
        for (int i = 0; i < 7; ++i) {
          QTreeWidgetItem* child = grpFF->child(i);
          if (!child) continue;
          DoubleSlider* slider =
              child->data(1, Qt::UserRole).value<DoubleSlider*>();
          if (slider) {
            slider->blockSignals(true);
            slider->setValue(pos[i]);
            slider->blockSignals(false);
          }
          freeflyerValues_[name][i] = pos[i];
        }
      }

    } else if (pos.size() == 1) {
      // JOINT
      if (objPosInSceneTree_.find(name) == objPosInSceneTree_.end()) {
        QString qname = QString::fromStdString(name);
        QStringList parts = qname.split('/');

        QTreeWidgetItem* parent = grpJoints_;

        for (int i = 0; i < parts.size() - 1; ++i) {
          parent = getOrCreateNamespaceItem(parent, parts[i]);
        }
        auto* item =
            addJointSliderItem(parent, name, parts.last(), min, max, pos[0]);
        objPosInSceneTree_[name] = item;
      } else {
        // Mise à jour
        QTreeWidgetItem* item = objPosInSceneTree_[name];
        DoubleSlider* slider =
            item->data(1, Qt::UserRole).value<DoubleSlider*>();

        if (slider) {
          slider->blockSignals(true);
          slider->setRange(min, max);
          slider->setValue(pos[0]);
          slider->blockSignals(false);
        }
      }
    }
  }
}

}  // namespace panel
}  // namespace hpp
