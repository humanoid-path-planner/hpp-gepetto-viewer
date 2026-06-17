#include "../../include/hpp/panel/trajectory_slider.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <std_msgs/msg/float64.hpp>

namespace hpp {
namespace panel {

TrajectorySlider::TrajectorySlider(QWidget* parent) : Panel(parent) {
  initializeUi();

  connect(slider_, &QSlider::valueChanged, this, &TrajectorySlider::onSlide);
  connect(toogle_play_button_, &QPushButton::clicked, this,
          &TrajectorySlider::onTogglePlay);
  connect(play_timer_, &QTimer::timeout, this, &TrajectorySlider::onTimer);
  connect(speed_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          [this](double value) { speed_ = value; });
  connect(t_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &TrajectorySlider::onTValueChanged);
  connect(target_frame_selector_, &QComboBox::currentTextChanged, this,
          &TrajectorySlider::onComboChanged);
  connect(copy_button_, &QPushButton::clicked, [this]() {
    QApplication::clipboard()->setText(
        hpp_vector_configuration_edit_->toPlainText());
    copy_button_->setText("Copied");
    QTimer::singleShot(1000, this, [this]() { copy_button_->setText("Copy"); });
  });
}

TrajectorySlider::~TrajectorySlider() = default;

void TrajectorySlider::onSlide(int value) {
  if (path_length_ <= 0.0) return;

  // Mappe 0..1000 → 0..path_length_
  current_time_ = (value / 1000.0) * path_length_;
  updateTimeDisplay();
  publishTime();
}

void TrajectorySlider::onTValueChanged(double value) {
  if (path_length_ <= 0.0) return;

  current_time_ = value;
  {
    QSignalBlocker blocker(slider_);
    slider_->setValue(
        static_cast<int>((current_time_ / path_length_) * 1000.0));
  }
  updateTimeDisplay();
  publishTime();
}

void TrajectorySlider::onTogglePlay() {
  is_playing_ = !is_playing_;

  if (is_playing_) {
    if (current_time_ >= path_length_) {
      current_time_ = 0.0;
      QSignalBlocker blocker(slider_);
      slider_->setValue(0);
    }
    toogle_play_button_->setText("⏸ Pause");
    play_timer_->start();
  } else {
    toogle_play_button_->setText("▶ Play");
    play_timer_->stop();
  }
}

void TrajectorySlider::onTimer() {
  if (!is_playing_ || path_length_ <= 0.0) return;

  // Advance by one timer tick: dt = 1 / trajectoryFps
  double delta = (1 / trajectoryFps);
  current_time_ += delta * speed_;

  if (current_time_ >= path_length_) {
    current_time_ = path_length_;
    is_playing_ = false;
    play_timer_->stop();
    toogle_play_button_->setText("▶ Play");
  }

  // Sync slider
  {
    QSignalBlocker blocker(slider_);
    slider_->setValue(
        static_cast<int>((current_time_ / path_length_) * 1000.0));
  }

  updateTimeDisplay();
  publishTime();
}

void TrajectorySlider::onPlay() {}
void TrajectorySlider::onPause() {}

void TrajectorySlider::onComboChanged(const QString& text) {
  hpp_gepetto_viewer::msg::PathInfo msg;
  msg.target_frame = text.toStdString();
  target_frame_pub_->publish(msg);
}
}  // namespace panel
}  // namespace hpp

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hpp::panel::TrajectorySlider, rviz_common::Panel)
