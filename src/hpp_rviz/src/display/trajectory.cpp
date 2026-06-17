
#include "../../include/hpp/display/trajectory.hpp"

namespace hpp {
namespace displays {

void TrajectoryDisplay::onInitialize() {
  rviz_default_plugins::displays::PathDisplay::onInitialize();

  auto* window_manager = context_->getWindowManager();
  if (window_manager) {
    hpp::panel::TrajectorySlider* hpp_panel =
        new hpp::panel::TrajectorySlider();
    hpp_panel->initialize(context_);
    window_manager->addPane("HPP Trajectory Control", hpp_panel);
  }

  setTopic("/hpp_path", "");
}
}  // namespace displays
}  // namespace hpp

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hpp::displays::TrajectoryDisplay, rviz_common::Display)
