# HPP-gepetto-viewer

[![Building Status](https://travis-ci.org/humanoid-path-planner/hpp-gepetto-viewer.svg?branch=master)](https://travis-ci.org/humanoid-path-planner/hpp-gepetto-viewer)
[![Pipeline status](https://gitlab.laas.fr/humanoid-path-planner/hpp-gepetto-viewer/badges/master/pipeline.svg)](https://gitlab.laas.fr/humanoid-path-planner/hpp-gepetto-viewer/commits/master)
[![Coverage report](https://gitlab.laas.fr/humanoid-path-planner/hpp-gepetto-viewer/badges/master/coverage.svg?job=doc-coverage)](https://gepettoweb.laas.fr/doc/humanoid-path-planner/hpp-gepetto-viewer/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/humanoid-path-planner/hpp-gepetto-viewer/master.svg)](https://results.pre-commit.ci/latest/github/humanoid-path-planner/hpp-gepetto-viewer)

## Compare trajectories in Viser

Load paths with descriptive names, then plot the same robot frame along each:

```python
from pyhpp_viser import Viewer

viewer = Viewer(robot)
viewer.loadPath(path, "Planned")
viewer.loadPath(optimized_path, "Optimized")
viewer.plotFrameTrajectories("robot/tool_frame")  # Use a frame from your model.
```

In the **Trajectory** tab, choose a frame and use **Plot All Paths**, or choose
a source **Path** and use **Plot Frame**. **Plot Selected** uses the frame selected
in the scene. Trajectories receive distinct colors from a cycling palette.

Each entry under **Trajectories** identifies its path and frame. Expand it to
adjust its RGB color, line width, visibility, or scene label. **Select in Player**
selects that entry's source path and resets playback to its start; use **Play**
in the **Path Player** tab to animate it. Hover over the color control or player
button for an identity hint. Viser's line API does not expose 3D hover events;
**Show Label** identifies a trajectory directly in the scene.

For explicit colors or a subset of loaded paths:

```python
viewer.plotFrameTrajectory("robot/tool_frame", path="Planned", color=(230, 90, 40))
viewer.plotFrameTrajectories("robot/tool_frame", paths=["Planned", "Optimized"])
viewer.clearTrajectories()
```

Colors accept RGB values in 0–255 or 0–1. Replotting a path/frame replaces its
trajectory and preserves its color; an explicit `name` creates a separately named
trace. Path objects passed directly to plotting are loaded automatically.
Replacing a loaded path under the same name removes its old trajectories.
