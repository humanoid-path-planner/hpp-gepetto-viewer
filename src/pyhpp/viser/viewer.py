import time
import warnings

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


class Viewer(BaseVisualizer):
    """A Pinocchio visualizer using Viser with Gepetto-GUI style hierarchy."""

    def __init__(
        self,
        robot
    ):
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
            copy_models = False,
            data = None,
            collision_data = None,
            visual_data = None,
        )
        self.viser_frames = {}
        self.display_collisions = False
        self.display_visuals = True
        self.display_frames_flag = False
        self.viewerRootNodeName = None
        self.framesRootNodeName = None
        self.framesRootFrame = None
        self._viewer_initialized = False
        self.path = None
        self.path_playing = False
        self.path_thread = None

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
        self.display_frames_flag = False

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

            # Handle both single frame and list of frames (for multi-geometry COLLADA)
            if isinstance(frame, list):
                for i, f in enumerate(frame):
                    indexed_name = f"{node_name}_{i}"
                    self.viser_frames[indexed_name] = f
            else:
                self.viser_frames[node_name] = frame

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
            if self.display_visuals and self.visual_model is not None:
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

            if self.display_collisions and self.collision_model is not None:
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

            if self.display_frames_flag:
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
        self.display_collisions = visibility
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
        self.display_visuals = visibility
        if self.visual_model is None:
            return

        for visual in self.visual_model.geometryObjects:
            node_name = self.getGeometryObjectNodeName(visual, pin.GeometryType.VISUAL)
            for frame in self._get_geometry_frames(node_name):
                frame.visible = visibility

    def displayFrames(self, visibility):
        """Set whether to display frames or not."""
        self.display_frames_flag = visibility

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

    def loadPath(self, path):
        """Load a path and create GUI controls for playback."""
        self.path = path
        self.path_playing = False
        self.path_update_lock = False

        path_folder = self.viewer.gui.add_folder("Path Player")

        with path_folder:
            self.path_slider = self.viewer.gui.add_slider(
                "Position (s)",
                min=0.0,
                max=float(path.length()),
                step=0.001,
                initial_value=0.0,
            )

            self.play_button = self.viewer.gui.add_button("▶ Play")
            self.stop_button = self.viewer.gui.add_button("⏸ Stop")

            self.speed_slider = self.viewer.gui.add_slider(
                "Speed", min=0.1, max=10.0, step=0.1, initial_value=1.0
            )

            self.fps_slider = self.viewer.gui.add_slider(
                "Target FPS", min=10, max=120, step=5, initial_value=60
            )

        @self.path_slider.on_update
        def _on_slider_update(_):
            if not self.path_update_lock:
                q, success = self.path.eval(self.path_slider.value)
                if success:
                    self.display(q)

        @self.play_button.on_click
        def _on_play_click(_):
            if not self.path_playing:
                self.path_playing = True
                self._start_path_animation()

        @self.stop_button.on_click
        def _on_stop_click(_):
            self.path_playing = False

        q, success = self.path.eval(0.0)
        if success:
            self.display(q)

    def _start_path_animation(self):
        """Start animating the path in a background thread."""
        if self.path_thread is not None and self.path_thread.is_alive():
            return

        def animate():
            path_length = self.path.length()
            path_time = self.path_slider.value
            last_wall_time = time.perf_counter()
            slider_update_counter = 0

            while self.path_playing and path_time < path_length:
                frame_start = time.perf_counter()
                target_frame_time = 1.0 / self.fps_slider.value

                wall_dt = frame_start - last_wall_time
                last_wall_time = frame_start

                path_time += wall_dt * self.speed_slider.value
                path_time = min(path_time, path_length)

                q, success = self.path.eval(path_time)

                if success:
                    self.display(q)

                slider_update_counter += 1
                if slider_update_counter >= 10:
                    self.path_update_lock = True
                    self.path_slider.value = path_time
                    self.path_update_lock = False
                    slider_update_counter = 0

                # Adaptive sleep
                elapsed = time.perf_counter() - frame_start
                sleep_time = max(0, target_frame_time - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)

            self.path_update_lock = True
            self.path_slider.value = 0.0 if path_time >= path_length else path_time
            self.path_update_lock = False

            self.path_playing = False

        self.path_thread = threading.Thread(target=animate, daemon=True)
        self.path_thread.start()

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
