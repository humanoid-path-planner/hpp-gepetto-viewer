import logging
import os
import time
import warnings
from dataclasses import dataclass, field

# Suppress GTK warnings
os.environ.setdefault("GTK_MODULES", "")
os.environ.setdefault("NO_AT_BRIDGE", "1")

try:
    import hppfcl
except ImportError:
    raise ImportError("hppfcl not found, but it is currently required by this viewer.")

import threading

import numpy as np
import pinocchio as pin
from pinocchio.visualize import BaseVisualizer

try:
    import trimesh  # Required by viser
    import viser

    # Suppress viser verbose logging
    logging.getLogger("viser").setLevel(logging.WARNING)
except ImportError:
    import_viser_succeed = False
else:
    import_viser_succeed = True


MESH_TYPES = (hppfcl.BVHModelBase, hppfcl.HeightFieldOBBRSS, hppfcl.HeightFieldAABB)

_FRAME_TYPE_GROUPS = {
    pin.FrameType.OP_FRAME: "op_frames",
    pin.FrameType.JOINT: "joints",
    pin.FrameType.FIXED_JOINT: "fixed_joints",
    pin.FrameType.BODY: "bodies",
    pin.FrameType.SENSOR: "sensors",
}


@dataclass
class _PathPlayerState:
    current: object = None
    paths: dict = field(default_factory=dict)
    counter: int = 0
    playing: bool = False
    thread: object = None
    update_lock: bool = False


@dataclass
class _DisplayState:
    collisions: bool = False
    contact_surfaces: bool = False
    frames: bool = False
    visuals: bool = True


@dataclass
class _PlaybackUpdateRateState:
    visuals_fps: float = 30.0
    collisions_fps: float = 30.0
    frames_fps: float = 30.0
    contact_surfaces_fps: float = 30.0
    last_updates: dict = field(default_factory=dict)


@dataclass
class _SelectionState:
    node_name: str | None = None
    frames: list = field(default_factory=list)
    geom_name: str | None = None
    geom_type: str | None = None
    frame_id: int | None = None


@dataclass
class _FrameBatchState:
    group: str
    handle: object
    frame_ids: list
    frame_names: list
    positions: np.ndarray
    wxyzs: np.ndarray
    scales: np.ndarray


@dataclass
class _GeometryFrameState:
    geom_id: int
    geometry_object: object
    frames: tuple
    is_static: bool = False
    initialized: bool = False
    last_position: object = None
    last_rotation: object = None


@dataclass
class _BatchedGeometryEntry:
    node_name: str


@dataclass
class _BatchedGeometryState:
    name: str
    handle: object
    entries: list
    geom_ids: list
    mesh_scales: np.ndarray
    positions: np.ndarray
    wxyzs: np.ndarray
    is_static: bool = False
    initialized: bool = False


@dataclass
class _ProfilerEntry:
    total: float = 0.0
    count: int = 0
    max: float = 0.0
    unit: str = "ms"


@dataclass
class _ProfilerState:
    enabled: bool = False
    print_every: int = 0
    reset_after_print: bool = False
    animation_frames: int = 0
    entries: dict = field(default_factory=dict)


class Viewer(BaseVisualizer):
    """A Pinocchio visualizer using Viser with Gepetto-GUI style hierarchy."""

    _shared_server = None

    def __init__(self, robot, problem=None):
        if not import_viser_succeed:
            msg = (
                "Error while importing the viewer client.\n"
                "Check whether viser and its dependencies are properly installed.\n"
                "Required packages: viser, trimesh\n"
                "Install with: pip install --user viser trimesh"
            )
            raise ImportError(msg)

        model = robot.model()
        collision_model = None
        if callable(robot.geomModel):
            collision_model = robot.geomModel()
        visual_model = None
        if callable(robot.visualModel):
            visual_model = robot.visualModel()

        super().__init__(
            model,
            collision_model,
            visual_model,
            copy_models=False,
            data=None,
            collision_data=None,
            visual_data=None,
        )
        self.viser_frames = {}
        self._display = _DisplayState()
        self._playback_update_rates = _PlaybackUpdateRateState()
        self._path_player = _PathPlayerState()
        self._selection = _SelectionState()
        self._node_to_geom_info = {}
        self.viewerRootNodeName = None
        self.framesRootNodeName = None
        self.framesRootFrame = None
        self._frame_type_roots = {}
        self._frame_batches = {}
        self._frame_filter_pattern = ""
        self._geometry_frames = {}  # {base geometry path: [viser handles]}
        self._visual_geometry_frames = []
        self._visual_batched_geometry_frames = []
        self._visual_display_handles = []
        self._collision_geometry_frames = []
        self._profiler = _ProfilerState()
        self._viewer_initialized = False
        self.start_qt_viewer = False
        self._react_graph_viewer_port = 6789
        self._react_graph_viewer_host = "localhost"
        self._web_socket_bridge_port = 8765
        self._web_socket_bridge_host = "localhost"

        self.problem = problem
        self.graph = None
        if problem is not None:
            try:
                self.graph = problem.constraintGraph()
            except AttributeError:
                pass
        self._robot = robot
        self._config_queue = None
        self._contact_surface_frames = {}
        self._contact_surface_joints = {}
        self._contact_surfaces_root = None
        self._graph_thread = None
        self._last_config = None

    def __call__(self, q):
        """Allow calling viewer as v(q) for compatibility with Gepetto-GUI."""
        if not self._viewer_initialized:
            if hasattr(self, "viewer") and self.viewer is not None:
                self.loadViewerModel()
            else:
                self.initViewer(loadModel=True)
        self.display(q)

    def getGeometryObjectNodeName(
        self, geometry_object, geometry_type, create_groups=False
    ):
        """
        Find the node corresponding to a GeometryObject (Gepetto-GUI style)
        """
        type_str = (
            "collision" if geometry_type == pin.GeometryType.COLLISION else "visual"
        )
        names = geometry_object.name.split("/")

        if len(names) == 1:
            names = [self.viewerRootNodeName.split("/")[-1], names[0]]

        names = [*names, type_str]
        res = self.viewerRootNodeName
        for n in names:
            res += "/" + n

        if create_groups:
            self._create_hierarchy_nodes(names)

        return res

    def _create_hierarchy_nodes(self, names):
        """
        Create intermediate scene nodes to build the hierarchical structure
        """
        frame_path = self.viewerRootNodeName

        if frame_path not in self.viser_frames:
            self.viser_frames[frame_path] = self.viewer.scene.add_frame(
                frame_path, show_axes=False
            )

        for n in names[:-1]:
            frame_path = frame_path + "/" + n

            if frame_path not in self.viser_frames:
                self.viser_frames[frame_path] = self.viewer.scene.add_frame(
                    frame_path, show_axes=False
                )

    def _get_visual_batch_parent(self, geometry_object):
        """Return the hierarchy prefix used to place a visual batch."""
        names = geometry_object.name.split("/")
        if len(names) == 1:
            parent_names = [self.viewerRootNodeName.split("/")[-1]]
        else:
            parent_names = names[:-1]

        parent_path = self.viewerRootNodeName
        for name in parent_names:
            parent_path += "/" + name
        return parent_path, parent_names

    def _is_geometry_static(self, geometry_object):
        """Return whether a geometry is attached to the universe joint."""
        return getattr(geometry_object, "parentJoint", None) == 0

    def start(self, host="localhost", port="8000", open=True, new_server=False):
        """Start the viewer, load the robot model, and open the browser.

        This is the recommended way to initialize the viewer:
            viewer = Viewer(robot)
            viewer.start()

        Args:
            host: Server hostname (default: localhost)
            port: Server port (default: 8000)
            open: Open browser automatically (default: True)
            new_server: Force a new ViserServer on a new port (default: False)
        """
        self.initViewer(
            open=open, loadModel=True, host=host, port=port, new_server=new_server
        )

    @staticmethod
    def _reset_gui(gui):
        """Reset viser GUI controls, including tab groups."""
        root_container = getattr(gui, "_container_handle_from_uuid", {}).get("root")
        if root_container is None or not hasattr(root_container, "_children"):
            gui.reset()
            return

        while root_container._children:
            handle = next(iter(root_container._children.values()))
            tab_handles = getattr(handle, "_tab_handles", None)
            if tab_handles is not None:
                # Viser marks tab groups removed before removing their tabs.
                for tab in tuple(tab_handles):
                    tab.remove()
            handle.remove()

        while getattr(gui, "_modal_handle_from_uuid", {}):
            next(iter(gui._modal_handle_from_uuid.values())).close()
        while getattr(gui, "_command_handle_from_uuid", {}):
            next(iter(gui._command_handle_from_uuid.values())).remove()

    def initViewer(
        self,
        viewer=None,
        open=False,
        loadModel=False,
        host="localhost",
        port="8000",
        new_server=False,
    ):
        """
        Start a new Viser server and client.

        By default, all Viewer instances share the same ViserServer so the
        browser URL (port) stays the same.  Pass ``new_server=True`` to force
        a separate server on a new port (useful for side-by-side views).

        Args:
            viewer: An existing ViserServer instance, or None.
            open: Open a browser tab automatically.
            loadModel: Load the robot model immediately.
            host: Server hostname.
            port: Server port (only used for the first server).
            new_server: If True, always create a new ViserServer on a new port.
        """
        if (viewer is not None) and (not isinstance(viewer, viser.ViserServer)):
            raise RuntimeError(
                "'viewer' argument must be None or a valid ViserServer instance."
            )

        if viewer is not None:
            self.viewer = viewer
        elif new_server:
            self.viewer = viser.ViserServer(host=host, server_port=port)
        elif Viewer._shared_server is not None:
            self.viewer = Viewer._shared_server
            # Clear previous scene and GUI so the new model loads cleanly
            self.viewer.scene.reset()
            self._reset_gui(self.viewer.gui)
        else:
            self.viewer = viser.ViserServer(host=host, server_port=port)
            Viewer._shared_server = self.viewer

        if open:
            import webbrowser

            webbrowser.open(f"http://{self.viewer.get_host()}:{self.viewer.get_port()}")
            while len(self.viewer.get_clients()) == 0:
                time.sleep(0.1)

        if loadModel:
            self.loadViewerModel()

    def loadViewerModel(
        self,
        rootNodeName="pinocchio",
        collision_color=None,
        visual_color=None,
        frame_axis_length=0.1,
        frame_axis_radius=0.003,
    ):
        """Load the robot in a Viser viewer with Gepetto-GUI style hierarchy."""
        self.viewerRootNodeName = rootNodeName
        self._viewer_initialized = True
        self._geometry_frames = {}
        self._visual_geometry_frames = []
        self._visual_batched_geometry_frames = []
        self._visual_display_handles = []
        self._collision_geometry_frames = []
        self._contact_surface_frames = {}
        self._contact_surface_joints = {}
        self._contact_surfaces_root = None
        self._display.frames = False
        self._display.contact_surfaces = False
        self._frame_batches = {}
        self._frame_filter_pattern = ""

        # Create root frame
        if rootNodeName not in self.viser_frames:
            self.viser_frames[rootNodeName] = self.viewer.scene.add_frame(
                rootNodeName, show_axes=False
            )

        # Load visual model
        if (visual_color is not None) and (len(visual_color) != 4):
            raise RuntimeError("visual_color must have 4 elements for RGBA.")
        if self.visual_model is not None:
            self._load_visual_geometry_objects(visual_color)
        self.displayVisuals(True)

        # Load collision model
        if (collision_color is not None) and (len(collision_color) != 4):
            raise RuntimeError("collision_color must have 4 elements for RGBA.")
        if self.collision_model is not None:
            for collision in self.collision_model.geometryObjects:
                self.loadViewerGeometryObject(
                    collision, pin.GeometryType.COLLISION, collision_color
                )
        self.displayCollisions(
            self.collision_model is not None and self.visual_model is None
        )

        self.framesRootNodeName = rootNodeName + "/frames"
        self.framesRootFrame = self.viewer.scene.add_frame(
            self.framesRootNodeName, show_axes=False, visible=False
        )

        # Group frames by type for selective display in scene tree.
        self._frame_type_roots = {}
        frames_by_group = {}
        for frame_id, frame in enumerate(self.model.frames):
            group = _FRAME_TYPE_GROUPS.get(frame.type, "other")
            frames_by_group.setdefault(group, []).append((frame_id, frame.name))

        for group, frames in sorted(frames_by_group.items()):
            group_path = self.framesRootNodeName + "/" + group
            self._frame_type_roots[group] = self.viewer.scene.add_frame(
                group_path, show_axes=False
            )
            self.viser_frames[group_path] = self._frame_type_roots[group]
            self._add_frame_batch(
                group, group_path, frames, frame_axis_length, frame_axis_radius
            )

        # Auto-load contact surfaces if the robot supports them
        if hasattr(self._robot, "contactSurfaces"):
            self.loadContactSurfaces(self._robot)

        # Add display controls
        self._create_display_controls()

    def _add_frame_batch(
        self, group, group_path, frames, frame_axis_length, frame_axis_radius
    ):
        """Create one batched axes node for all kinematic frames of a type."""
        num_frames = len(frames)
        positions = np.zeros((num_frames, 3), dtype=np.float32)
        wxyzs = np.zeros((num_frames, 4), dtype=np.float32)
        wxyzs[:, 0] = 1.0
        scales = np.ones((num_frames,), dtype=np.float32)
        axes_path = group_path + "/axes"
        handle = self.viewer.scene.add_batched_axes(
            axes_path,
            batched_wxyzs=wxyzs,
            batched_positions=positions,
            batched_scales=scales,
            axes_length=frame_axis_length,
            axes_radius=frame_axis_radius,
            visible=False,
        )
        frame_ids = [frame_id for frame_id, _ in frames]
        frame_names = [name for _, name in frames]
        batch = _FrameBatchState(
            group=group,
            handle=handle,
            frame_ids=frame_ids,
            frame_names=frame_names,
            positions=positions,
            wxyzs=wxyzs,
            scales=scales,
        )
        self._frame_batches[group] = batch
        self.viser_frames[axes_path] = handle
        self._register_frame_batch_click_callback(batch)

    def _create_display_controls(self):
        """Create GUI controls for display options."""
        tab_group = self.viewer.gui.add_tab_group()

        with tab_group.add_tab("Controls"):
            self._create_selection_panel()
            self._create_path_player()
            self._create_graph_viewer_controls()

        with tab_group.add_tab("Display"):
            self._create_visibility_toggles()

    def _create_selection_panel(self):
        """Create GUI panel for displaying selected object info."""
        selection_folder = self.viewer.gui.add_folder("Selected Object")

        with selection_folder:
            self._selection_name_text = self.viewer.gui.add_markdown("*None*")
            self._selection_type_text = self.viewer.gui.add_markdown("")
            self._focus_button = self.viewer.gui.add_button("Focus Selected")

        @self._focus_button.on_click
        def _on_focus_click(_):
            self._focus_selected()

    def _create_path_player(self):
        """Create the path player GUI controls (always visible)."""
        path_folder = self.viewer.gui.add_folder("Path Player")

        with path_folder:
            self.path_dropdown = self.viewer.gui.add_dropdown(
                "Path", options=["None"], initial_value="None"
            )

            self.path_slider = self.viewer.gui.add_slider(
                "Position (s)",
                min=0.0,
                max=1.0,
                step=0.001,
                initial_value=0.0,
            )

            self.play_button = self.viewer.gui.add_button("Play")
            self.stop_button = self.viewer.gui.add_button("Stop")

            self.speed_slider = self.viewer.gui.add_slider(
                "Speed", min=0.1, max=10.0, step=0.1, initial_value=1.0
            )

            self.fps_slider = self.viewer.gui.add_slider(
                "Target FPS", min=10, max=120, step=5, initial_value=60
            )

        @self.path_dropdown.on_update
        def _on_path_select(_):
            self._path_player.playing = False
            name = self.path_dropdown.value
            if name == "None":
                self._path_player.current = None
                return
            self._path_player.current = self._path_player.paths[name]
            self._path_player.update_lock = True
            self.path_slider.max = float(self._path_player.current.length())
            self.path_slider.value = 0.0
            self._path_player.update_lock = False
            q, success = self._path_player.current.eval(0.0)
            if success:
                self.display(q)

        @self.path_slider.on_update
        def _on_slider_update(_):
            if (
                not self._path_player.update_lock
                and not self._path_player.playing
                and self._path_player.current is not None
            ):
                q, success = self._path_player.current.eval(self.path_slider.value)
                if success:
                    self.display(q)

        @self.play_button.on_click
        def _on_play_click(_):
            if self._path_player.current is not None and not self._path_player.playing:
                self._path_player.playing = True
                self._start_path_animation()

        @self.stop_button.on_click
        def _on_stop_click(_):
            self._path_player.playing = False

    def _register_click_callback(self, handle, node_name):
        """Register a click callback on a mesh handle for selection."""

        @handle.on_click
        def _on_mesh_click(_):
            self._select_node(node_name)

    def _register_frame_batch_click_callback(self, batch):
        """Register a click callback on a batched frame axes handle."""

        @batch.handle.on_click
        def _on_frame_click(event):
            frame_index = event.instance_index
            if frame_index is None or frame_index >= len(batch.frame_ids):
                return
            self._select_frame(batch, frame_index)

    def _register_batched_geometry_click_callback(self, batch):
        """Register a click callback on a batched geometry handle."""

        @batch.handle.on_click
        def _on_batched_geometry_click(event):
            instance_index = event.instance_index
            if instance_index is None or instance_index >= len(batch.entries):
                return
            self._select_node(batch.entries[instance_index].node_name)

    def _select_frame(self, batch, frame_index):
        frame_id = batch.frame_ids[frame_index]
        frame_name = batch.frame_names[frame_index]
        node_name = f"{self.framesRootNodeName}/{batch.group}/{frame_id}:{frame_name}"

        if self._selection.node_name == node_name:
            self._deselect()
            return

        self._deselect()
        self._selection.node_name = node_name
        self._selection.frames = [batch.handle]
        self._selection.geom_name = frame_name
        self._selection.geom_type = "frame"
        self._selection.frame_id = frame_id
        self._update_selection_panel()

    def _select_node(self, node_name):
        """Select or deselect a scene node."""
        if self._selection.node_name == node_name:
            self._deselect()
            return

        self._deselect()

        frames = self._get_geometry_frames(node_name)

        geom_info = self._node_to_geom_info.get(node_name, {})
        self._selection.node_name = node_name
        self._selection.frames = frames
        self._selection.geom_name = geom_info.get("name")
        self._selection.geom_type = geom_info.get("type")
        self._selection.frame_id = None

        self._update_selection_panel()

    def _deselect(self):
        """Clear the current selection."""
        self._selection.node_name = None
        self._selection.frames = []
        self._selection.geom_name = None
        self._selection.geom_type = None
        self._selection.frame_id = None
        self._update_selection_panel()

    def _update_selection_panel(self):
        """Update the selection info panel GUI."""
        if self._selection.geom_name is not None:
            self._selection_name_text.content = f"**{self._selection.geom_name}**"
            geom_type = self._selection.geom_type
            self._selection_type_text.content = f"Type: {geom_type}"
        else:
            self._selection_name_text.content = "*None*"
            self._selection_type_text.content = ""

    def _focus_selected(self):
        """Center the camera on the currently selected object."""
        if self._selection.node_name is None:
            return

        if self._selection.frame_id is not None:
            position = self.data.oMf[self._selection.frame_id].translation
            clients = self.viewer.get_clients()
            for client in clients.values():
                client.camera.look_at = position
            return

        geom_info = self._node_to_geom_info.get(self._selection.node_name, {})
        geometry_type = geom_info.get("geometry_type")
        geom_name = geom_info.get("name")
        if geometry_type is None or geom_name is None:
            return

        if geometry_type == pin.GeometryType.VISUAL and self.visual_model is not None:
            geom_id = self.visual_model.getGeometryId(geom_name)
            position = self.visual_data.oMg[geom_id].translation
        elif (
            geometry_type == pin.GeometryType.COLLISION
            and self.collision_model is not None
        ):
            geom_id = self.collision_model.getGeometryId(geom_name)
            position = self.collision_data.oMg[geom_id].translation
        else:
            return

        clients = self.viewer.get_clients()
        for client in clients.values():
            client.camera.look_at = position

    def _load_visual_geometry_objects(self, visual_color):
        """Load visual objects, batching repeated simple primitives when possible."""
        batched_groups = {}
        fallback_objects = []
        for visual in self.visual_model.geometryObjects:
            batch_spec = self._make_visual_batch_spec(visual, visual_color)
            if batch_spec is None:
                fallback_objects.append(visual)
                continue

            (
                key,
                mesh_source,
                primitive_color,
                flat_shading,
                batch_kind,
                node_name,
                batch_parent_path,
                batch_parent_names,
                is_static,
            ) = batch_spec
            group = batched_groups.setdefault(
                key,
                {
                    "kind": batch_kind,
                    "mesh_source": mesh_source,
                    "color": primitive_color,
                    "flat_shading": flat_shading,
                    "parent_path": batch_parent_path,
                    "parent_names": batch_parent_names,
                    "is_static": is_static,
                    "objects": [],
                    "node_names": [],
                },
            )
            group["objects"].append(visual)
            group["node_names"].append(node_name)

        for index, group in enumerate(batched_groups.values()):
            objects = group["objects"]
            if len(objects) < 2:
                fallback_objects.extend(objects)
                continue
            if not self._load_batched_visual_geometry(index, group):
                fallback_objects.extend(objects)

        for visual in fallback_objects:
            self.loadViewerGeometryObject(visual, pin.GeometryType.VISUAL, visual_color)

    def _make_visual_batch_spec(self, geometry_object, color):
        """Return a batching key and mesh for simple visual primitives."""
        primitive_color, color_override, use_embedded_colors = (
            self._geometry_color_options(geometry_object, color)
        )
        primitive_color = tuple(float(value) for value in primitive_color)
        geom = geometry_object.geometry
        mesh_path = getattr(geometry_object, "meshPath", "")
        node_name = self.getGeometryObjectNodeName(
            geometry_object, pin.GeometryType.VISUAL, create_groups=False
        )
        batch_parent_path, batch_parent_names = self._get_visual_batch_parent(
            geometry_object
        )
        is_static = self._is_geometry_static(geometry_object)

        if (
            color_override is None
            and use_embedded_colors
            and len(mesh_path) > 0
            and (isinstance(geom, MESH_TYPES) or isinstance(geom, hppfcl.Convex))
        ):
            real_mesh_path = os.path.realpath(mesh_path)
            key = (batch_parent_path, is_static, "mesh_path", real_mesh_path)
            return (
                key,
                real_mesh_path,
                None,
                None,
                "trimesh",
                node_name,
                batch_parent_path,
                batch_parent_names,
                is_static,
            )

        if (
            color_override is not None
            and len(mesh_path) > 0
            and (isinstance(geom, MESH_TYPES) or isinstance(geom, hppfcl.Convex))
        ):
            real_mesh_path = os.path.realpath(mesh_path)
            key = (
                batch_parent_path,
                is_static,
                "colored_mesh_path",
                real_mesh_path,
                primitive_color,
            )
            return (
                key,
                real_mesh_path,
                primitive_color,
                False,
                "mesh_simple",
                node_name,
                batch_parent_path,
                batch_parent_names,
                is_static,
            )

        if isinstance(geom, hppfcl.Box):
            extents = tuple(float(value) for value in geom.halfSide * 2.0)
            mesh = trimesh.creation.box(extents=extents)
            primitive_key = ("box", extents)
            flat_shading = True
        elif isinstance(geom, hppfcl.Sphere):
            radius = float(geom.radius)
            mesh = trimesh.creation.icosphere(radius=radius)
            primitive_key = ("sphere", radius)
            flat_shading = False
        elif isinstance(geom, hppfcl.Cylinder):
            radius = float(geom.radius)
            height = float(geom.halfLength * 2.0)
            mesh = trimesh.creation.cylinder(radius=radius, height=height)
            primitive_key = ("cylinder", radius, height)
            flat_shading = False
        elif isinstance(geom, hppfcl.Capsule):
            radius = float(geom.radius)
            height = float(geom.halfLength * 2.0)
            mesh = trimesh.creation.capsule(radius=radius, height=height)
            primitive_key = ("capsule", radius, height)
            flat_shading = False
        elif isinstance(geom, hppfcl.Cone):
            radius = float(geom.radius)
            height = float(geom.halfLength * 2.0)
            mesh = trimesh.creation.cone(radius=radius, height=height)
            primitive_key = ("cone", radius, height)
            flat_shading = False
        else:
            return None

        key = (
            batch_parent_path,
            is_static,
            primitive_key,
            primitive_color,
            flat_shading,
        )
        return (
            key,
            mesh,
            primitive_color,
            flat_shading,
            "simple",
            node_name,
            batch_parent_path,
            batch_parent_names,
            is_static,
        )

    def _load_batched_visual_geometry(self, batch_index, group):
        objects = group["objects"]
        num_objects = len(objects)
        positions = np.zeros((num_objects, 3), dtype=np.float32)
        wxyzs = np.zeros((num_objects, 4), dtype=np.float32)
        wxyzs[:, 0] = 1.0

        self._create_hierarchy_nodes([*group["parent_names"], "visual_batch"])

        batch_name = (
            f"{group['parent_path']}/visual_batch_{group['kind']}_{batch_index}"
        )
        color = group["color"]
        if group["kind"] in ("simple", "mesh_simple"):
            mesh = group["mesh_source"]
            if group["kind"] == "mesh_simple":
                try:
                    mesh = self._load_simple_batch_mesh(mesh)
                except Exception as exc:
                    warnings.warn(
                        f"Failed to batch visual mesh {mesh}: {exc}",
                        UserWarning,
                        stacklevel=2,
                    )
                    return False
            handle = self.viewer.scene.add_batched_meshes_simple(
                batch_name,
                vertices=np.asarray(mesh.vertices, dtype=np.float32),
                faces=np.asarray(mesh.faces, dtype=np.uint32),
                batched_wxyzs=wxyzs,
                batched_positions=positions,
                batched_colors=color[:3],
                opacity=color[3],
                flat_shading=group["flat_shading"],
                visible=self._display.visuals,
            )
        else:
            try:
                mesh = self._load_mesh(group["mesh_source"])
                handle = self.viewer.scene.add_batched_meshes_trimesh(
                    batch_name,
                    mesh=mesh,
                    batched_wxyzs=wxyzs,
                    batched_positions=positions,
                    visible=self._display.visuals,
                )
            except Exception as exc:
                warnings.warn(
                    f"Failed to batch visual mesh {group['mesh_source']}: {exc}",
                    UserWarning,
                    stacklevel=2,
                )
                return False
        self.viser_frames[batch_name] = handle
        self._visual_display_handles.append(handle)

        entries = []
        geom_ids = []
        mesh_scales = np.empty((num_objects, 3), dtype=np.float32)
        for index, (visual, node_name) in enumerate(zip(objects, group["node_names"])):
            geom_info = {
                "name": visual.name,
                "type": "visual",
                "geometry_type": pin.GeometryType.VISUAL,
            }
            geom_id = self.visual_model.getGeometryId(visual.name)
            self._node_to_geom_info[node_name] = geom_info
            self._geometry_frames[node_name] = [handle]
            entries.append(_BatchedGeometryEntry(node_name=node_name))
            geom_ids.append(geom_id)
            mesh_scales[index] = visual.meshScale

        batch = _BatchedGeometryState(
            name=batch_name,
            handle=handle,
            entries=entries,
            geom_ids=geom_ids,
            mesh_scales=mesh_scales,
            positions=positions,
            wxyzs=wxyzs,
            is_static=group["is_static"],
        )
        self._visual_batched_geometry_frames.append(batch)
        self._register_batched_geometry_click_callback(batch)
        return True

    def _load_simple_batch_mesh(self, mesh_path):
        mesh = self._load_mesh(mesh_path)
        if isinstance(mesh, trimesh.Scene):
            mesh = mesh.dump(concatenate=True)
        if not hasattr(mesh, "vertices") or not hasattr(mesh, "faces"):
            msg = f"mesh does not expose vertices/faces: {type(mesh)}"
            raise ValueError(msg)
        return mesh

    def _geometry_color_options(self, geometry_object, color):
        if color is not None:
            color_override = color
            use_embedded_colors = False
        elif geometry_object.overrideMaterial:
            color_override = geometry_object.meshColor
            use_embedded_colors = False
        else:
            color_override = None
            use_embedded_colors = True

        primitive_color = (
            (0.5, 0.5, 0.5, 1.0) if use_embedded_colors else color_override
        )
        return primitive_color, color_override, use_embedded_colors

    def loadViewerGeometryObject(self, geometry_object, geometry_type, color=None):
        """Load a single geometry object with hierarchical naming."""
        node_name = self.getGeometryObjectNodeName(
            geometry_object, geometry_type, create_groups=True
        )

        geom = geometry_object.geometry

        primitive_color, color_override, use_embedded_colors = (
            self._geometry_color_options(geometry_object, color)
        )

        type_str = (
            "collision" if geometry_type == pin.GeometryType.COLLISION else "visual"
        )

        try:
            if isinstance(geom, hppfcl.Box):
                frame = self.viewer.scene.add_box(
                    node_name,
                    dimensions=geom.halfSide * 2.0,
                    color=primitive_color[:3],
                    opacity=primitive_color[3],
                )
            elif isinstance(geom, hppfcl.Sphere):
                frame = self.viewer.scene.add_icosphere(
                    node_name,
                    radius=geom.radius,
                    color=primitive_color[:3],
                    opacity=primitive_color[3],
                )
            elif isinstance(geom, hppfcl.Cylinder):
                mesh = trimesh.creation.cylinder(
                    radius=geom.radius,
                    height=geom.halfLength * 2.0,
                )
                frame = self.viewer.scene.add_mesh_simple(
                    node_name,
                    mesh.vertices,
                    mesh.faces,
                    color=primitive_color[:3],
                    opacity=primitive_color[3],
                )
            elif isinstance(geom, hppfcl.Capsule):
                mesh = trimesh.creation.capsule(
                    radius=geom.radius,
                    height=geom.halfLength * 2.0,
                )
                frame = self.viewer.scene.add_mesh_simple(
                    node_name,
                    mesh.vertices,
                    mesh.faces,
                    color=primitive_color[:3],
                    opacity=primitive_color[3],
                )
            elif isinstance(geom, hppfcl.Cone):
                mesh = trimesh.creation.cone(
                    radius=geom.radius,
                    height=geom.halfLength * 2.0,
                )
                frame = self.viewer.scene.add_mesh_simple(
                    node_name,
                    mesh.vertices,
                    mesh.faces,
                    color=primitive_color[:3],
                    opacity=primitive_color[3],
                )
            elif isinstance(geom, MESH_TYPES):
                frame = self._add_mesh_from_path(
                    node_name,
                    geometry_object.meshPath,
                    color_override,
                    use_embedded_colors,
                    scale=geometry_object.meshScale,
                )
            elif isinstance(geom, hppfcl.Convex):
                if len(geometry_object.meshPath) > 0:
                    frame = self._add_mesh_from_path(
                        node_name,
                        geometry_object.meshPath,
                        color_override,
                        use_embedded_colors,
                        scale=geometry_object.meshScale,
                    )
                else:
                    frame = self._add_mesh_from_convex(
                        node_name, geom, color_override or (0.5, 0.5, 0.5, 1.0)
                    )
            else:
                msg = f"Unsupported geometry type for {geometry_object.name} ({type(geom)})"
                warnings.warn(msg, category=UserWarning, stacklevel=2)
                return

            # Store geometry info for selection lookups
            geom_info = {
                "name": geometry_object.name,
                "type": type_str,
                "geometry_type": geometry_type,
            }

            frames = [frame]
            self.viser_frames[node_name] = frame
            self._node_to_geom_info[node_name] = geom_info
            self._register_click_callback(frame, node_name)

            self._node_to_geom_info[node_name] = geom_info
            self._geometry_frames[node_name] = frames
            geom_model = (
                self.collision_model
                if geometry_type == pin.GeometryType.COLLISION
                else self.visual_model
            )
            if geom_model is not None:
                geom_id = geom_model.getGeometryId(geometry_object.name)
                cached_entry = _GeometryFrameState(
                    geom_id,
                    geometry_object,
                    tuple(frames),
                    is_static=self._is_geometry_static(geometry_object),
                )
                if geometry_type == pin.GeometryType.COLLISION:
                    self._collision_geometry_frames.append(cached_entry)
                else:
                    self._visual_geometry_frames.append(cached_entry)
                    self._visual_display_handles.extend(frames)

        except Exception as e:
            msg = (
                "Error while loading geometry object: "
                f"{geometry_object.name}\nError message:\n{e}"
            )
            warnings.warn(msg, category=UserWarning, stacklevel=2)

    def _add_mesh_from_path(
        self, name, mesh_path, color, use_embedded_colors, scale=None
    ):
        """Load a mesh from a file."""
        return self._load_standard_mesh(
            name, mesh_path, color, use_embedded_colors, scale=scale
        )

    def _load_mesh(self, mesh_path):
        try:
            return trimesh.load_mesh(mesh_path)
        except ImportError as exc:
            if os.path.splitext(mesh_path)[1].lower() == ".dae":
                msg = (
                    f"Failed to load COLLADA mesh {mesh_path}. "
                    "Install pycollada so trimesh can read .dae files."
                )
                raise ImportError(msg) from exc
            raise

    def _load_standard_mesh(
        self, name, mesh_path, color, use_embedded_colors, scale=None
    ):
        """Load a mesh using trimesh, preserving embedded colors when requested."""
        mesh = self._load_mesh(mesh_path)
        apply_scale = scale is not None and not np.allclose(scale, 1.0)

        if apply_scale:
            mesh.apply_scale(scale)

        # if we should use embedded colors and no explicit override, use trimesh mesh
        if use_embedded_colors and color is None:
            return self.viewer.scene.add_mesh_trimesh(name, mesh)

        # If explicit color provided use it as override
        if color is not None:
            return self.viewer.scene.add_mesh_simple(
                name, mesh.vertices, mesh.faces, color=color[:3], opacity=color[3]
            )

        return self.viewer.scene.add_mesh_trimesh(name, mesh)

    def _add_mesh_from_convex(self, name, geom, color):
        """Load a mesh from triangles stored inside a hppfcl.Convex."""
        num_tris = geom.num_polygons
        call_triangles = geom.polygons
        call_vertices = geom.points

        vertices = call_vertices()
        vertices = vertices.astype(np.float32)
        faces = np.empty((num_tris, 3), dtype=int)
        for k in range(num_tris):
            tri = call_triangles(k)
            faces[k] = [tri[i] for i in range(3)]

        return self.viewer.scene.add_mesh_simple(
            name,
            vertices,
            faces,
            color=color[:3],
            opacity=color[3],
        )

    def _get_geometry_frames(self, node_name):
        """Get all frames associated with a geometry object (handles indexed multi-geometry meshes)."""
        if node_name in self._geometry_frames:
            return self._geometry_frames[node_name]

        if node_name in self.viser_frames:
            return [self.viser_frames[node_name]]

        frames = []
        indexed_prefix = f"{node_name}_"
        for key in self.viser_frames:
            if key.startswith(indexed_prefix):
                suffix = key[len(indexed_prefix) :]
                if suffix.isdigit():
                    frames.append(self.viser_frames[key])

        return frames

    def enableProfiling(self, print_every=120, reset=True, reset_after_print=False):
        """Enable lightweight playback/display timing diagnostics.

        Args:
            print_every: Print stats every N animation frames. Use 0 to disable
                automatic printing and call printProfilingStats() manually.
            reset: Clear previous accumulated stats.
            reset_after_print: Clear stats after each automatic print.
        """
        if reset:
            self.resetProfilingStats()
        self._profiler.enabled = True
        self._profiler.print_every = int(print_every)
        self._profiler.reset_after_print = reset_after_print

    def disableProfiling(self):
        """Disable playback/display timing diagnostics."""
        self._profiler.enabled = False

    def resetProfilingStats(self):
        """Clear accumulated profiling stats."""
        self._profiler.entries.clear()
        self._profiler.animation_frames = 0

    def profilingStats(self):
        """Return profiling stats as a nested dictionary."""
        stats = {}
        for name, entry in self._profiler.entries.items():
            total = entry.total * 1000.0 if entry.unit == "ms" else entry.total
            max_value = entry.max * 1000.0 if entry.unit == "ms" else entry.max
            stats[name] = {
                "count": entry.count,
                "total": total,
                "average": total / entry.count if entry.count else 0.0,
                "max": max_value,
                "unit": entry.unit,
            }
        return stats

    def printProfilingStats(self, reset=False, limit=40):
        """Print accumulated profiling stats sorted by total cost."""
        print(self._formatProfilingStats(limit=limit))
        if reset:
            self.resetProfilingStats()

    def _formatProfilingStats(self, limit=40):
        entries = list(self._profiler.entries.items())
        if not entries:
            return "[pyhpp_viser profiler] no samples"

        def sort_key(item):
            name, entry = item
            return (entry.unit != "ms", -entry.total, name)

        lines = ["[pyhpp_viser profiler]"]
        lines.append(
            "name                                      count      total        avg        max"
        )
        lines.append("-" * 84)
        for name, entry in sorted(entries, key=sort_key)[:limit]:
            scale = 1000.0 if entry.unit == "ms" else 1.0
            total = entry.total * scale
            avg = total / entry.count if entry.count else 0.0
            max_value = entry.max * scale
            unit = "ms" if entry.unit == "ms" else entry.unit
            lines.append(
                f"{name:<40} {entry.count:>6} {total:>10.3f} "
                f"{avg:>10.3f} {max_value:>10.3f} {unit}"
            )
        return "\n".join(lines)

    def _profile_start(self):
        return time.perf_counter() if self._profiler.enabled else None

    def _profile_since(self, name, start):
        if start is not None:
            self._profile_value(name, time.perf_counter() - start, "ms")

    def _profile_value(self, name, value, unit="ms"):
        if not self._profiler.enabled:
            return
        entry = self._profiler.entries.get(name)
        if entry is None:
            entry = _ProfilerEntry(unit=unit)
            self._profiler.entries[name] = entry
        entry.total += value
        entry.count += 1
        entry.max = max(entry.max, value)

    def _profile_message_counter(self):
        if not self._profiler.enabled or not hasattr(self, "viewer"):
            return None
        try:
            return self.viewer._websock_server._broadcast_buffer.message_counter
        except AttributeError:
            return None

    def _profile_messages_since(self, name, start):
        if start is None:
            return
        end = self._profile_message_counter()
        if end is not None:
            self._profile_value(name, float(end - start), "messages")

    def _profile_animation_frame(self):
        if not self._profiler.enabled:
            return
        self._profiler.animation_frames += 1
        print_every = self._profiler.print_every
        if print_every > 0 and self._profiler.animation_frames % print_every == 0:
            self.printProfilingStats(reset=self._profiler.reset_after_print)

    def setPlaybackUpdateRates(
        self,
        visuals=None,
        collisions=None,
        frames=None,
        contact_surfaces=None,
    ):
        """Set display update caps used only during path playback.

        A value of 0 disables the cap for that display component.
        """
        updates = {
            "visuals": visuals,
            "collisions": collisions,
            "frames": frames,
            "contact_surfaces": contact_surfaces,
        }
        for component, value in updates.items():
            if value is None:
                continue
            setattr(self._playback_update_rates, f"{component}_fps", float(value))
            self._reset_playback_update_timer(component)

    def _reset_playback_update_timer(self, component=None):
        if component is None:
            self._playback_update_rates.last_updates.clear()
        else:
            self._playback_update_rates.last_updates.pop(component, None)

    def _should_update_display_component(self, component, now):
        if not self._path_player.playing:
            return True

        fps = getattr(self._playback_update_rates, f"{component}_fps")
        if fps <= 0:
            return True

        last_update = self._playback_update_rates.last_updates.get(component)
        if last_update is None or now - last_update >= 1.0 / fps:
            self._playback_update_rates.last_updates[component] = now
            return True

        self._profile_value(f"display.{component}_throttled", 1.0, "frames")
        return False

    def display(self, q=None):
        """Display the robot at configuration q."""
        display_start = self._profile_start()
        messages_start = self._profile_message_counter()

        if q is not None:
            fk_start = self._profile_start()
            pin.forwardKinematics(self.model, self.data, q)
            self._profile_since("display.forward_kinematics", fk_start)

        update_time = time.perf_counter()
        atomic_start = self._profile_start()
        with self.viewer.atomic():
            if (
                self._display.visuals
                and self.visual_model is not None
                and self._should_update_display_component("visuals", update_time)
            ):
                visuals_start = self._profile_start()
                self._update_geometry_frames(
                    self.visual_model,
                    self.visual_data,
                    self._visual_geometry_frames,
                    "visuals",
                    self._visual_batched_geometry_frames,
                )
                self._profile_since("display.visuals", visuals_start)

            if (
                self._display.collisions
                and self.collision_model is not None
                and self._should_update_display_component("collisions", update_time)
            ):
                collisions_start = self._profile_start()
                self._update_geometry_frames(
                    self.collision_model,
                    self.collision_data,
                    self._collision_geometry_frames,
                    "collisions",
                )
                self._profile_since("display.collisions", collisions_start)

            if self._display.frames and self._should_update_display_component(
                "frames", update_time
            ):
                frames_start = self._profile_start()
                self.updateFrames()
                self._profile_since("display.frames", frames_start)
            if self._display.contact_surfaces and self._should_update_display_component(
                "contact_surfaces", update_time
            ):
                contacts_start = self._profile_start()
                self.updateContactSurfaces()
                self._profile_since("display.contact_surfaces", contacts_start)
        self._profile_since("display.atomic_block", atomic_start)
        self._profile_messages_since("display.queued_messages", messages_start)
        self._profile_since("display.total", display_start)

    def _update_geometry_frames(
        self,
        geom_model,
        geom_data,
        geometry_frames,
        label,
        batched_geometry_frames=None,
    ):
        total_start = self._profile_start()
        pin_start = self._profile_start()
        pin.updateGeometryPlacements(self.model, self.data, geom_model, geom_data)
        self._profile_since(f"{label}.update_geometry_placements", pin_start)

        queue_start = self._profile_start()
        changed_entries = 0
        skipped_entries = 0
        for entry in geometry_frames:
            if entry.is_static and entry.initialized:
                skipped_entries += 1
                continue

            M = geom_data.oMg[entry.geom_id]
            position = M.translation * entry.geometry_object.meshScale
            rotation = M.rotation

            position_changed = entry.last_position is None or not np.array_equal(
                position, entry.last_position
            )
            rotation_changed = entry.last_rotation is None or not np.array_equal(
                rotation, entry.last_rotation
            )
            if not position_changed and not rotation_changed:
                skipped_entries += 1
                continue

            changed_entries += 1
            wxyz = None
            if rotation_changed:
                wxyz = pin.Quaternion(rotation).coeffs()[[3, 0, 1, 2]]

            for frame in entry.frames:
                if position_changed:
                    frame.position = position
                if rotation_changed:
                    frame.wxyz = wxyz

            if position_changed:
                entry.last_position = position.copy()
            if rotation_changed:
                entry.last_rotation = rotation.copy()
            entry.initialized = True
        self._profile_value(
            f"{label}.changed_geometry_entries", float(changed_entries), "entries"
        )
        self._profile_value(
            f"{label}.skipped_geometry_entries", float(skipped_entries), "entries"
        )
        self._profile_since(f"{label}.queue_transforms", queue_start)
        if batched_geometry_frames:
            self._update_batched_geometry_frames(
                geom_data, batched_geometry_frames, label
            )
        self._profile_since(f"{label}.total", total_start)

    def _update_batched_geometry_frames(
        self, geom_data, batched_geometry_frames, label
    ):
        total_entries = 0
        queued_batches = 0
        skipped_batches = 0
        fill_start = self._profile_start()
        for batch in batched_geometry_frames:
            total_entries += len(batch.entries)
            if batch.is_static and batch.initialized:
                skipped_batches += 1
                continue

            for index, geom_id in enumerate(batch.geom_ids):
                M = geom_data.oMg[geom_id]
                batch.positions[index] = M.translation * batch.mesh_scales[index]
                batch.wxyzs[index] = pin.Quaternion(M.rotation).coeffs()[[3, 0, 1, 2]]
        self._profile_since(f"{label}.batched_geometry.fill_arrays", fill_start)

        queue_start = self._profile_start()
        for batch in batched_geometry_frames:
            if batch.is_static and batch.initialized:
                continue

            queued_batches += 1
            batch.handle.batched_positions = batch.positions.copy()
            batch.handle.batched_wxyzs = batch.wxyzs.copy()
            batch.initialized = True

        self._profile_value(
            f"{label}.queued_batched_geometry", float(queued_batches), "batches"
        )
        self._profile_value(
            f"{label}.skipped_batched_geometry", float(skipped_batches), "batches"
        )
        self._profile_value(
            f"{label}.batched_geometry_entries", float(total_entries), "entries"
        )
        self._profile_since(f"{label}.queue_batched_transforms", queue_start)

    def updateFrames(self):
        """Update the position and orientation of all frames."""
        total_start = self._profile_start()
        pin_start = self._profile_start()
        pin.updateFramePlacements(self.model, self.data)
        self._profile_since("frames.update_frame_placements", pin_start)

        for batch in self._frame_batches.values():
            fill_start = self._profile_start()
            for index, frame_id in enumerate(batch.frame_ids):
                M = self.data.oMf[frame_id]
                batch.positions[index] = M.translation
                batch.wxyzs[index] = pin.Quaternion(M.rotation).coeffs()[[3, 0, 1, 2]]
            self._profile_since(f"frames.{batch.group}.fill_arrays", fill_start)

            positions_start = self._profile_start()
            batch.handle.batched_positions = batch.positions.copy()
            self._profile_since(
                f"frames.{batch.group}.queue_positions", positions_start
            )

            wxyzs_start = self._profile_start()
            batch.handle.batched_wxyzs = batch.wxyzs.copy()
            self._profile_since(f"frames.{batch.group}.queue_wxyzs", wxyzs_start)
        self._profile_since("frames.total", total_start)

    def displayCollisions(self, visibility):
        """Set whether to display collision objects or not."""
        self._display.collisions = visibility
        if visibility:
            self._reset_playback_update_timer("collisions")
        if self.collision_model is None:
            return

        for collision in self.collision_model.geometryObjects:
            node_name = self.getGeometryObjectNodeName(
                collision, pin.GeometryType.COLLISION
            )
            for frame in self._get_geometry_frames(node_name):
                frame.visible = visibility

    def displayVisuals(self, visibility):
        """Set whether to display visual objects or not."""
        self._display.visuals = visibility
        if visibility:
            self._reset_playback_update_timer("visuals")
        for frame in self._visual_display_handles:
            frame.visible = visibility

    def displayFrames(self, visibility):
        """Set whether to display frames or not.

        Explicitly sets visibility on all hierarchy levels (root, type groups,
        and batched axes so that prior scene tree interactions are overridden.
        """
        self._display.frames = visibility
        if visibility:
            self._reset_playback_update_timer("frames")
        if self.framesRootFrame is not None:
            self.framesRootFrame.visible = visibility
        for root in self._frame_type_roots.values():
            root.visible = visibility
        for batch in self._frame_batches.values():
            batch.handle.visible = visibility
        if visibility:
            self.updateFrames()

    def _apply_frame_filter(self, pattern):
        """Filter batched frame axes by scaling unmatched instances to zero."""
        self._frame_filter_pattern = pattern.lower()
        for root in self._frame_type_roots.values():
            root.visible = self._display.frames
        for batch in self._frame_batches.values():
            if self._frame_filter_pattern:
                batch.scales[:] = [
                    1.0 if self._frame_filter_pattern in name.lower() else 0.0
                    for name in batch.frame_names
                ]
            else:
                batch.scales[:] = 1.0
            batch.handle.batched_scales = batch.scales.copy()

    def loadContactSurfaces(self, robot, color=(0.2, 0.8, 0.2, 0.5)):
        """Load contact surfaces from a manipulation device.

        Args:
            robot: A pyhpp.manipulation.Device with contact surfaces
            color: RGBA tuple for surface color (default: semi-transparent green)
        """
        if not hasattr(robot, "contactSurfaces"):
            warnings.warn(
                "Robot does not have contactSurfaces method. ",
                UserWarning,
            )
            return

        surfaces = robot.contactSurfaces()
        if not surfaces:
            return

        contact_root = self.viewerRootNodeName + "/contact_surfaces"
        self._contact_surfaces_root = self.viewer.scene.add_frame(
            contact_root, show_axes=False, visible=False
        )

        for surface_name, surface_list in surfaces.items():
            for idx, surface_data in enumerate(surface_list):
                joint_name = surface_data["joint"]
                points = surface_data["points"]

                if len(points) < 3:
                    continue

                node_name = f"{contact_root}/{surface_name}_{idx}"
                vertices = np.array(points, dtype=np.float32)
                num_pts = len(vertices)

                if num_pts == 3:
                    faces = np.array([[0, 1, 2]], dtype=np.int32)
                else:
                    faces = self._triangulate_convex_polygon(num_pts)

                try:
                    mesh_handle = self.viewer.scene.add_mesh_simple(
                        node_name,
                        vertices,
                        faces,
                        color=color[:3],
                        opacity=color[3],
                        side="double",
                    )
                    self._contact_surface_frames[node_name] = mesh_handle
                    self._contact_surface_joints[node_name] = joint_name
                except Exception as e:
                    warnings.warn(
                        f"Failed to create contact surface {surface_name}: {e}",
                        UserWarning,
                    )

    def _triangulate_convex_polygon(self, num_vertices):
        """Triangulate a convex polygon using fan triangulation."""
        faces = []
        for i in range(1, num_vertices - 1):
            faces.append([0, i, i + 1])
        return np.array(faces, dtype=np.int32)

    def displayContactSurfaces(self, visibility):
        """Set whether to display contact surfaces or not."""
        self._display.contact_surfaces = visibility
        if visibility:
            self._reset_playback_update_timer("contact_surfaces")
        if self._contact_surfaces_root is not None:
            self._contact_surfaces_root.visible = visibility
        if visibility:
            self.updateContactSurfaces()

    def updateContactSurfaces(self):
        """Update contact surface positions based on current joint transforms."""
        total_start = self._profile_start()
        for node_name, mesh_handle in self._contact_surface_frames.items():
            joint_name = self._contact_surface_joints.get(node_name)
            if joint_name == "universe" or joint_name is None:
                continue

            try:
                frame = self.model.getFrameId(joint_name)
                M = self.data.oMf[frame]
                queue_start = self._profile_start()
                mesh_handle.position = M.translation
                mesh_handle.wxyz = pin.Quaternion(M.rotation).coeffs()[[3, 0, 1, 2]]
                self._profile_since("contacts.queue_transforms", queue_start)
            except (ValueError, KeyError):
                pass
        self._profile_since("contacts.total", total_start)

    def captureImage(self, w=None, h=None, client_id=None, transport_format="jpeg"):
        """Capture an image from the Viser viewer."""
        clients = self.viewer.get_clients()
        if len(clients) == 0:
            raise RuntimeError("Viser server has no attached clients!")

        if client_id is None:
            cli = next(iter(clients.values()))
        elif client_id not in clients:
            raise RuntimeError(
                f"Viser server does not have a client with ID '{client_id}'"
            )
        else:
            cli = clients[client_id]

        height = h or cli.camera.image_height
        width = w or cli.camera.image_width
        return cli.get_render(
            height=height, width=width, transport_format=transport_format
        )

    def loadPath(self, path, name=None):
        """Load a path into the path player dropdown."""
        if not self._viewer_initialized:
            if hasattr(self, "viewer") and self.viewer is not None:
                self.loadViewerModel()
            else:
                self.initViewer(loadModel=True)

        if name is None:
            name = f"Path {self._path_player.counter}"
        self._path_player.counter += 1

        self._path_player.playing = False
        self._path_player.paths[name] = path
        self._path_player.current = path

        self.path_dropdown.options = list(self._path_player.paths.keys())
        self.path_dropdown.value = name

        self._path_player.update_lock = True
        self.path_slider.max = float(path.length())
        self.path_slider.value = 0.0
        self._path_player.update_lock = False

        q, success = path.eval(0.0)
        if success:
            self.display(q)

    def _start_path_animation(self):
        """Start animating the path in a background thread."""
        if self._path_player.current is None:
            return
        if self._path_player.thread is not None and self._path_player.thread.is_alive():
            return

        def animate():
            self._reset_playback_update_timer()
            length_start = self._profile_start()
            path_length = self._path_player.current.length()
            self._profile_since("animation.path_length", length_start)
            path_time = self.path_slider.value
            last_wall_time = time.perf_counter()
            slider_update_counter = 0

            while self._path_player.playing and path_time < path_length:
                frame_start = time.perf_counter()
                target_frame_time = 1.0 / self.fps_slider.value

                wall_dt = frame_start - last_wall_time
                last_wall_time = frame_start

                path_time += wall_dt * self.speed_slider.value
                path_time = min(path_time, path_length)

                eval_start = self._profile_start()
                q, success = self._path_player.current.eval(path_time)
                self._profile_since("animation.path_eval", eval_start)

                if success:
                    display_start = self._profile_start()
                    self.display(q)
                    self._profile_since("animation.display", display_start)

                slider_update_counter += 1
                if slider_update_counter >= 10:
                    slider_start = self._profile_start()
                    self._path_player.update_lock = True
                    self.path_slider.value = path_time
                    self._path_player.update_lock = False
                    slider_update_counter = 0
                    self._profile_since("animation.slider_update", slider_start)

                # Adaptive sleep
                elapsed = time.perf_counter() - frame_start
                self._profile_value("animation.frame_work", elapsed, "ms")
                sleep_time = max(0, target_frame_time - elapsed)
                if sleep_time > 0:
                    sleep_start = self._profile_start()
                    time.sleep(sleep_time)
                    self._profile_since("animation.sleep", sleep_start)
                else:
                    self._profile_value("animation.over_budget_frames", 1.0, "frames")
                    time.sleep(0)
                self._profile_animation_frame()

            self._path_player.update_lock = True
            self.path_slider.value = 0.0 if path_time >= path_length else path_time
            self._path_player.update_lock = False

            self._path_player.playing = False

        self._path_player.thread = threading.Thread(target=animate, daemon=True)
        self._path_player.thread.start()

    def _create_graph_viewer_controls(self):
        """Create GUI controls for constraint graph viewer integration."""
        graph_folder = self.viewer.gui.add_folder("Constraint Graph")

        with graph_folder:
            self._graph_button = self.viewer.gui.add_button("Show Graph Viewer")

        @self._graph_button.on_click
        def _on_show_graph_click(_):
            self._launch_graph_viewer()
            if not self.start_qt_viewer:
                import webbrowser

                webbrowser.open(
                    f"http://{self._react_graph_viewer_host}:{self._react_graph_viewer_port}"
                )

    def _create_visibility_toggles(self):
        """Create checkboxes for toggling visibility of scene elements."""
        vis_checkbox = self.viewer.gui.add_checkbox(
            "Show Visuals", initial_value=self._display.visuals
        )
        col_checkbox = self.viewer.gui.add_checkbox(
            "Show Collisions", initial_value=self._display.collisions
        )
        frames_checkbox = self.viewer.gui.add_checkbox(
            "Show Frames", initial_value=False
        )
        frames_folder = self.viewer.gui.add_folder("Frame Options")
        with frames_folder:
            frame_filter = self.viewer.gui.add_text(
                "Filter", initial_value="", hint="Filter frames by name"
            )
            frame_length_slider = self.viewer.gui.add_slider(
                "Axes Length",
                min=0.01,
                max=1.0,
                step=0.01,
                initial_value=0.1,
            )
            frame_radius_slider = self.viewer.gui.add_slider(
                "Axes Radius",
                min=0.001,
                max=0.05,
                step=0.001,
                initial_value=0.003,
            )
        contacts_checkbox = self.viewer.gui.add_checkbox(
            "Show Contact Surfaces", initial_value=False
        )
        rates_folder = self.viewer.gui.add_folder("Playback Update Rates")
        with rates_folder:
            visual_rate_slider = self.viewer.gui.add_slider(
                "Visual FPS",
                min=0,
                max=120,
                step=5,
                initial_value=self._playback_update_rates.visuals_fps,
            )
            collision_rate_slider = self.viewer.gui.add_slider(
                "Collision FPS",
                min=0,
                max=120,
                step=5,
                initial_value=self._playback_update_rates.collisions_fps,
            )
            frame_rate_slider = self.viewer.gui.add_slider(
                "Frame FPS",
                min=0,
                max=120,
                step=5,
                initial_value=self._playback_update_rates.frames_fps,
            )
            contact_rate_slider = self.viewer.gui.add_slider(
                "Contact FPS",
                min=0,
                max=120,
                step=5,
                initial_value=self._playback_update_rates.contact_surfaces_fps,
            )

        @vis_checkbox.on_update
        def _(_):
            self.displayVisuals(vis_checkbox.value)

        @col_checkbox.on_update
        def _(_):
            self.displayCollisions(col_checkbox.value)

        @frames_checkbox.on_update
        def _(_):
            self.displayFrames(frames_checkbox.value)

        @frame_filter.on_update
        def _(_):
            self._apply_frame_filter(frame_filter.value)

        def _update_frame_axes(_):
            length = frame_length_slider.value
            radius = frame_radius_slider.value
            for batch in self._frame_batches.values():
                batch.handle.axes_length = length
                batch.handle.axes_radius = radius

        frame_length_slider.on_update(_update_frame_axes)
        frame_radius_slider.on_update(_update_frame_axes)

        @contacts_checkbox.on_update
        def _(_):
            self.displayContactSurfaces(contacts_checkbox.value)

        @visual_rate_slider.on_update
        def _(_):
            self.setPlaybackUpdateRates(visuals=visual_rate_slider.value)

        @collision_rate_slider.on_update
        def _(_):
            self.setPlaybackUpdateRates(collisions=collision_rate_slider.value)

        @frame_rate_slider.on_update
        def _(_):
            self.setPlaybackUpdateRates(frames=frame_rate_slider.value)

        @contact_rate_slider.on_update
        def _(_):
            self.setPlaybackUpdateRates(contact_surfaces=contact_rate_slider.value)

    def setProblem(self, problem):
        """Set Problem for graph viewer integration.

        Args:
            problem: PyWProblem from pyhpp.manipulation
        """
        self.problem = problem
        self._publish_viewer_snapshot(None, self.problem)

    def setGraph(self, graph):
        """Set constraint graph for graph viewer integration.

        Args:
            graph: PyWGraph from pyhpp.manipulation
        """
        self.graph = graph
        self._publish_viewer_snapshot(self.graph, None)

    def setupReactGraphViewer(self, port: int, host: str = "localhost"):
        """Set the port for the React-based graph viewer.

        Args:
            port: The port number to use for the React viewer.
            host: The host address for the React viewer.
        """
        self._react_graph_viewer_port = port
        self._react_graph_viewer_host = host

    def setupWebSocketBridge(self, port: int, host: str = "localhost"):
        """Set the port for the WebSocket-based graph viewer (Qt).

        Args:
            port: The port number to use for the WebSocket viewer.
            host: The host address for the WebSocket viewer.
        """
        self._web_socket_bridge_port = port
        self._web_socket_bridge_host = host

    def setQtGraphViewer(self, choice=True):
        """Set whether to start the Qt-based graph viewer instead of the React-based one.

        Args:
            choice: If True, start the Qt viewer. If False, start the React viewer. Default is True.
        """
        self.start_qt_viewer = choice

    def _publish_viewer_snapshot(self, graph=None, problem=None):
        """Send the current graph/problem snapshot to the React app if available."""
        if self._graph_thread is None or not self._graph_thread.is_alive():
            print("Graph viewer thread not running, cannot publish viewer snapshot.")
            return
        self._graph_thread.send_viewer_snapshot(graph, problem)

    def _launch_graph_viewer(self):
        """Launch hpp-plot graph viewer in separate thread."""
        if self.graph is None:
            print("No constraint graph set. Use viewer.setGraph(graph)")
            return
        if self.problem is None:
            print("No problem set. Use viewer.setProblem(problem)")
            return

        try:
            if self._graph_thread is not None and self._graph_thread.is_alive():
                print("Graph viewer thread is already running")
                return
            from pyhpp_plot import GraphViewerThread

            thread = GraphViewerThread(
                self.graph,
                self.problem,
                self._on_config_generated,
                react_port=self._react_graph_viewer_port,
                react_host=self._react_graph_viewer_host,
                ws_port=self._web_socket_bridge_port,
                ws_host=self._web_socket_bridge_host,
                start_qt_viewer=self.start_qt_viewer,
            )
            self._graph_thread = thread
            thread.start()
        except Exception as exc:
            print(f"Failed to launch graph viewer: {exc}")
            pass

    def _on_config_generated(self, config, label):
        """Called from graph viewer thread when config is generated."""
        self._last_config = config
        self.display(config)

    def playPath(self, path, speed=1):
        """
        Play a path animation (blocking).

        DEPRECATED: Use loadPath() then use GUI controls for playback.

        Args:
            path: The path to play
            speed: Playback speed multiplier (default 1)
        """
        warnings.warn(
            "'playPath' is deprecated since HPP 5.0. "
            "Use loadPath(path) and GUI controls instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        self.loadPath(path)
        self.speed_slider.value = speed
        self._path_player.playing = True
        self._start_path_animation()
        # Block until playback completes (for old API compatibility)
        while self._path_player.playing:
            time.sleep(0.1)

    def setBackgroundColor(self):
        raise NotImplementedError()

    def setCameraTarget(self, target):
        raise NotImplementedError()

    def setCameraPosition(self, position: np.ndarray):
        raise NotImplementedError()

    def setCameraZoom(self, zoom: float):
        raise NotImplementedError()

    def setCameraPose(self, pose: np.ndarray):
        raise NotImplementedError()

    def disableCameraControl(self):
        raise NotImplementedError()

    def enableCameraControl(self):
        raise NotImplementedError()

    def drawFrameVelocities(self, *args, **kwargs):
        raise NotImplementedError()


__all__ = ["Viewer"]
