import queue
import time
import warnings
from dataclasses import dataclass, field

try:
    import hppfcl
except ImportError:
    raise ImportError("hppfcl not found, but it is currently required by this viewer.")

import threading

import numpy as np
import pinocchio as pin
from pinocchio.visualize import BaseVisualizer

try:
    import collada
    import trimesh  # Required by viser
    import viser

except ImportError:
    import_viser_succeed = False
else:
    import_viser_succeed = True


MESH_TYPES = (hppfcl.BVHModelBase, hppfcl.HeightFieldOBBRSS, hppfcl.HeightFieldAABB)


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
    visuals: bool = True
    frames: bool = False


@dataclass
class _SelectionState:
    node_name: str | None = None
    frames: list = field(default_factory=list)
    geom_name: str | None = None
    geom_type: str | None = None


class Viewer(BaseVisualizer):
    """A Pinocchio visualizer using Viser with Gepetto-GUI style hierarchy."""

    def __init__(self, robot, problem=None):
        if not import_viser_succeed:
            msg = (
                "Error while importing the viewer client.\n"
                "Check whether viser and its dependencies are properly installed.\n"
                "Required packages: viser, trimesh, collada\n"
                "Install with: pip install --user viser trimesh pycollada"
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
        self._path_player = _PathPlayerState()
        self._selection = _SelectionState()
        self._node_to_geom_info = {}
        self.viewerRootNodeName = None
        self.framesRootNodeName = None
        self.framesRootFrame = None
        self._viewer_initialized = False

        self.problem = problem
        self.graph = None
        if problem is not None:
            try:
                self.graph = problem.constraintGraph()
            except AttributeError:
                pass
        self._config_queue = None

    def __call__(self, q):
        """Allow calling viewer as v(q) for compatibility with Gepetto-GUI."""
        if not self._viewer_initialized:
            self.initViewer(open=True, loadModel=True)
            self._viewer_initialized = True
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

    def initViewer(
        self,
        viewer=None,
        open=False,
        loadModel=False,
        host="localhost",
        port="8000",
    ):
        """
        Start a new Viser server and client.
        """
        if (viewer is not None) and (not isinstance(viewer, viser.ViserServer)):
            raise RuntimeError(
                "'viewer' argument must be None or a valid ViserServer instance."
            )

        self.viewer = viewer or viser.ViserServer(host=host, server_port=port)

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
        frame_axis_length=0.2,
        frame_axis_radius=0.01,
    ):
        """Load the robot in a Viser viewer with Gepetto-GUI style hierarchy."""
        self.viewerRootNodeName = rootNodeName
        self._viewer_initialized = True

        # Create root frame
        if rootNodeName not in self.viser_frames:
            self.viser_frames[rootNodeName] = self.viewer.scene.add_frame(
                rootNodeName, show_axes=False
            )

        # Load visual model
        if (visual_color is not None) and (len(visual_color) != 4):
            raise RuntimeError("visual_color must have 4 elements for RGBA.")
        if self.visual_model is not None:
            for visual in self.visual_model.geometryObjects:
                self.loadViewerGeometryObject(
                    visual, pin.GeometryType.VISUAL, visual_color
                )
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

        for frame in self.model.frames:
            frame_name = self.framesRootNodeName + "/" + frame.name
            self.viser_frames[frame_name] = self.viewer.scene.add_frame(
                frame_name,
                show_axes=True,
                axes_length=frame_axis_length,
                axes_radius=frame_axis_radius,
                visible=False,
            )
        self._display.frames = False

        # Add display controls
        self._create_display_controls()

    def _create_display_controls(self):
        """Create GUI controls for display options."""
        display_folder = self.viewer.gui.add_folder("Display Controls")

        with display_folder:
            self.frames_checkbox = self.viewer.gui.add_checkbox(
                "Show Frames", initial_value=False
            )

        @self.frames_checkbox.on_update
        def _on_frames_toggle(_):
            self.displayFrames(self.frames_checkbox.value)

        self._create_selection_panel()
        self._create_path_player()
        self._create_graph_viewer_controls()


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

        self._update_selection_panel()

    def _deselect(self):
        """Clear the current selection."""
        self._selection.node_name = None
        self._selection.frames = []
        self._selection.geom_name = None
        self._selection.geom_type = None
        self._update_selection_panel()

    def _update_selection_panel(self):
        """Update the selection info panel GUI."""
        if self._selection.geom_name is not None:
            self._selection_name_text.content = f"**{self._selection.geom_name}**"
            self._selection_type_text.content = f"Type: {self._selection.geom_type}"
        else:
            self._selection_name_text.content = "*None*"
            self._selection_type_text.content = ""

    def _focus_selected(self):
        """Center the camera on the currently selected object."""
        if self._selection.node_name is None:
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

    def loadViewerGeometryObject(self, geometry_object, geometry_type, color=None):
        """Load a single geometry object with hierarchical naming."""
        node_name = self.getGeometryObjectNodeName(
            geometry_object, geometry_type, create_groups=True
        )

        geom = geometry_object.geometry

        if color is not None:
            color_override = color
            use_embedded_colors = False
        elif geometry_object.overrideMaterial:
            color_override = geometry_object.meshColor
            use_embedded_colors = False
        else:
            color_override = None
            use_embedded_colors = True

        if use_embedded_colors:
            primitive_color = (0.5, 0.5, 0.5, 1.0)
        else:
            primitive_color = color_override

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
                )
            elif isinstance(geom, hppfcl.Convex):
                if len(geometry_object.meshPath) > 0:
                    frame = self._add_mesh_from_path(
                        node_name,
                        geometry_object.meshPath,
                        color_override,
                        use_embedded_colors,
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

            # Handle both single frame and list of frames (for multi-geometry COLLADA)
            if isinstance(frame, list):
                for i, f in enumerate(frame):
                    indexed_name = f"{node_name}_{i}"
                    self.viser_frames[indexed_name] = f
                    self._node_to_geom_info[indexed_name] = geom_info
                    self._register_click_callback(f, node_name)
            else:
                self.viser_frames[node_name] = frame
                self._node_to_geom_info[node_name] = geom_info
                self._register_click_callback(frame, node_name)

        except Exception as e:
            msg = (
                "Error while loading geometry object: "
                f"{geometry_object.name}\nError message:\n{e}"
            )
            warnings.warn(msg, category=UserWarning, stacklevel=2)

    def _add_mesh_from_path(self, name, mesh_path, color, use_embedded_colors):
        """Load a mesh from a file."""
        return self._load_standard_mesh(name, mesh_path, color, use_embedded_colors)

    def _load_collada_mesh(self, name, mesh_path, color):
        """Load a COLLADA mesh with color support."""
        try:
            mesh_collada = collada.Collada(mesh_path)
        except collada.DaeError:
            return self._load_standard_mesh(name, mesh_path, color)

        if len(mesh_collada.effects) < len(mesh_collada.geometries):
            return self._load_standard_mesh(name, mesh_path, color)

        frames = []
        for i, (geometry, effect) in enumerate(
            zip(mesh_collada.geometries, mesh_collada.effects)
        ):
            frame = self._process_collada_geometry(
                name, i, geometry, effect, color, mesh_path
            )
            if frame:
                frames.append(frame)

        # Return all frames as a list so they can all be tracked
        return frames if frames else None

    def _process_collada_geometry(
        self, name, index, geometry, effect, fallback_color, mesh_path
    ):
        """Process a single COLLADA geometry with its material."""
        indexed_name = f"{name}_{index}"

        try:
            vertices, faces = self._extract_geometry_data(geometry)
        except (AttributeError, IndexError, KeyError):
            # Fallback if geometry data extraction fails
            mesh = trimesh.load_mesh(mesh_path)
            return self.viewer.scene.add_mesh_trimesh(indexed_name, mesh)

        mesh_color = getattr(effect, "diffuse", None)

        if mesh_color is not None:
            return self.viewer.scene.add_mesh_simple(
                indexed_name,
                vertices,
                faces,
                color=mesh_color[:3],
                opacity=mesh_color[3],
            )
        elif fallback_color is not None:
            return self.viewer.scene.add_mesh_simple(
                indexed_name,
                vertices,
                faces,
                color=fallback_color[:3],
                opacity=fallback_color[3],
            )
        else:
            mesh = trimesh.load_mesh(mesh_path)
            return self.viewer.scene.add_mesh_trimesh(indexed_name, mesh)

    def _extract_geometry_data(self, geometry):
        """Extract vertices and faces from a COLLADA geometry."""
        vertices = geometry.primitives[0].sources["VERTEX"][0][4].data
        indices = geometry.primitives[0].indices

        if indices.ndim == 3:
            faces = indices[:, :, 0]
        else:
            faces = indices.reshape(-1, 3)

        return vertices, faces

    def _load_standard_mesh(self, name, mesh_path, color, use_embedded_colors):
        """Load a mesh using trimesh, preserving embedded colors when requested."""
        mesh = trimesh.load_mesh(mesh_path)

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

    def display(self, q=None):
        """Display the robot at configuration q."""
        if q is not None:
            pin.forwardKinematics(self.model, self.data, q)

        with self.viewer.atomic():
            if self._display.visuals and self.visual_model is not None:
                pin.updateGeometryPlacements(
                    self.model, self.data, self.visual_model, self.visual_data
                )
                for visual in self.visual_model.geometryObjects:
                    node_name = self.getGeometryObjectNodeName(
                        visual, pin.GeometryType.VISUAL
                    )

                    M = self.visual_data.oMg[
                        self.visual_model.getGeometryId(visual.name)
                    ]

                    for frame in self._get_geometry_frames(node_name):
                        frame.position = M.translation * visual.meshScale
                        frame.wxyz = pin.Quaternion(M.rotation).coeffs()[[3, 0, 1, 2]]

            if self._display.collisions and self.collision_model is not None:
                pin.updateGeometryPlacements(
                    self.model, self.data, self.collision_model, self.collision_data
                )
                for collision in self.collision_model.geometryObjects:
                    node_name = self.getGeometryObjectNodeName(
                        collision, pin.GeometryType.COLLISION
                    )

                    M = self.collision_data.oMg[
                        self.collision_model.getGeometryId(collision.name)
                    ]

                    for frame in self._get_geometry_frames(node_name):
                        frame.position = M.translation * collision.meshScale
                        frame.wxyz = pin.Quaternion(M.rotation).coeffs()[[3, 0, 1, 2]]

            if self._display.frames:
                self.updateFrames()

    def updateFrames(self):
        """Update the position and orientation of all frames."""
        pin.updateFramePlacements(self.model, self.data)
        for frame_id, frame in enumerate(self.model.frames):
            # Get frame pose
            M = self.data.oMf[frame_id]

            # Update viewer configuration
            viser_frame_name = self.framesRootNodeName + "/" + frame.name
            if viser_frame_name in self.viser_frames:
                viser_frame = self.viser_frames[viser_frame_name]
                viser_frame.position = M.translation
                viser_frame.wxyz = pin.Quaternion(M.rotation).coeffs()[[3, 0, 1, 2]]

    def displayCollisions(self, visibility):
        """Set whether to display collision objects or not."""
        self._display.collisions = visibility
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
        if self.visual_model is None:
            return

        for visual in self.visual_model.geometryObjects:
            node_name = self.getGeometryObjectNodeName(visual, pin.GeometryType.VISUAL)
            for frame in self._get_geometry_frames(node_name):
                frame.visible = visibility

    def displayFrames(self, visibility):
        """Set whether to display frames or not."""
        self._display.frames = visibility

        if self.framesRootFrame is not None:
            self.framesRootFrame.visible = visibility

        for frame in self.model.frames:
            frame_name = self.framesRootNodeName + "/" + frame.name
            if frame_name in self.viser_frames:
                self.viser_frames[frame_name].visible = visibility

        if visibility:
            self.updateFrames()

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
            path_length = self._path_player.current.length()
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

                q, success = self._path_player.current.eval(path_time)

                if success:
                    self.display(q)

                slider_update_counter += 1
                if slider_update_counter >= 10:
                    self._path_player.update_lock = True
                    self.path_slider.value = path_time
                    self._path_player.update_lock = False
                    slider_update_counter = 0

                # Adaptive sleep
                elapsed = time.perf_counter() - frame_start
                sleep_time = max(0, target_frame_time - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)

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

    def setProblem(self, problem):
        """Set Problem for graph viewer integration.

        Args:
            problem: PyWProblem from pyhpp.manipulation
        """
        self.problem = problem

    def setGraph(self, graph):
        """Set constraint graph for graph viewer integration.

        Args:
            graph: PyWGraph from pyhpp.manipulation
        """
        self.graph = graph

    def _launch_graph_viewer(self):
        """Launch hpp-plot graph viewer in separate thread."""
        if self.graph is None:
            print("No constraint graph set. Use viewer.setGraph(graph)")
            return
        if self.problem is None:
            print("No problem set. Use viewer.setProblem(problem)")
            return

        try:
            from pyhpp_plot import GraphViewerThread

            thread = GraphViewerThread(
                self.graph, self.problem, self._on_config_generated
            )
            thread.start()
            print("Graph viewer launched in background thread")
        except ImportError as e:
            print(f"Error: Could not import pyhpp_plot: {e}")
            print("Make sure hpp-plot is built and installed with Python bindings")
        except Exception as e:
            print(f"Error launching graph viewer: {e}")
            import traceback

            traceback.print_exc()

    def _on_config_generated(self, config, label):
        """Called from graph viewer thread when config is generated."""
        self.display(config)
        print(f"Displayed config: {label}")

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
