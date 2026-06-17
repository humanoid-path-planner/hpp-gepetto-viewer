#ifndef HPP_PLUGINS__TRAJECTORY__POINT_DISPLAY_HPP_
#define HPP_PLUGINS__TRAJECTORY__POINT_DISPLAY_HPP_

#include <memory>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_default_plugins/displays/path/path_display.hpp>

#include "../panel/trajectory_slider.hpp"

namespace hpp {
namespace displays {
class TrajectoryDisplay : public rviz_default_plugins::displays::PathDisplay {
  Q_OBJECT
 public:
  TrajectoryDisplay() = default;
  ~TrajectoryDisplay() = default;

  void onInitialize() override;

 private:
};
}  // namespace displays
}  // namespace hpp
#endif
