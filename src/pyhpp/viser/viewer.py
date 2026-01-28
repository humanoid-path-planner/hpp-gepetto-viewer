import time
import os
import warnings

try:
    import hppfcl
except ImportError:
    raise ImportError("hppfcl not found, but it is currently required by this viewer.")

import numpy as np

import pinocchio as pin
from pinocchio.visualize import BaseVisualizer

try:
    import trimesh  # Required by viser
    import viser
    import collada

except ImportError:
    import_viser_succeed = False
else:
    import_viser_succeed = True


MESH_TYPES = (hppfcl.BVHModelBase, hppfcl.HeightFieldOBBRSS, hppfcl.HeightFieldAABB)


class Viewer(BaseVisualizer):
    """A Pinocchio visualizer using Viser with Gepetto-GUI style hierarchy."""

    def __init__(
        self,
        model=pin.Model(),
        collision_model=None,
        visual_model=None,
        copy_models=False,
        data=None,
        collision_data=None,
        visual_data=None,
    ):
        if not import_viser_succeed:
            msg = (
                "Error while importing the viewer client.\n"
                "Check whether viser and its dependencies are properly installed.\n"
                "Required packages: viser, trimesh, collada\n"
                "Install with: pip install --user viser trimesh pycollada"
            )
            raise ImportError(msg)

        if hasattr(model, 'model'):
            robot = model
            
            if hasattr(robot, 'asPinDevice'):
                robot = robot.asPinDevice()
            
            if callable(robot.model):
                model = robot.model()
            else:
                model = robot.model

            if collision_model is None and hasattr(robot, 'collision_model'):
                collision_model = robot.collision_model() if callable(robot.collision_model) else robot.collision_model
            if visual_model is None and hasattr(robot, 'visual_model'):
                visual_model = robot.visual_model() if callable(robot.visual_model) else robot.visual_model

            if collision_model is None and hasattr(robot, 'geomModel'):
                collision_model = robot.geomModel() if callable(robot.geomModel) else robot.geomModel()
            if visual_model is None and hasattr(robot, 'visualModel'):
                visual_model = robot.visualModel() if callable(robot.visualModel) else robot.visualModel()

        super().__init__(
            model,
            collision_model,
            visual_model,
            copy_models,
            data,
            collision_data,
            visual_data,
        )
        self.viser_frames = {}
        self.display_collisions = False
        self.display_visuals = True
        self.display_frames_flag = True
        self.viewerRootNodeName = None
        self.framesRootNodeName = None
        self.framesRootFrame = None
        self._viewer_initialized = False

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
            self._create_intermediate_frames(names)

        return res

    def _create_intermediate_frames(self, names):
        """
        Create intermediate Viser frames to build the hierarchy
        """
        frame_path = self.viewerRootNodeName

        if frame_path not in self.viser_frames:
            self.viser_frames[frame_path] = self.viewer.scene.add_frame(
                frame_path, show_axes=False
            )

        for n in names[:-1]:
            parent_path = frame_path
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

        # Load frames - créer un groupe "frames" dans l'arborescence
        self.framesRootNodeName = rootNodeName + "/frames"
        self.framesRootFrame = self.viewer.scene.add_frame(
            self.framesRootNodeName, show_axes=False
        )

        for frame in self.model.frames:
            frame_name = self.framesRootNodeName + "/" + frame.name
            self.viser_frames[frame_name] = self.viewer.scene.add_frame(
                frame_name,
                show_axes=True,
                axes_length=frame_axis_length,
                axes_radius=frame_axis_radius,
            )
        # Frames visibles par défaut
        self.display_frames_flag = True

    def loadViewerGeometryObject(self, geometry_object, geometry_type, color=None):
        """Load a single geometry object with hierarchical naming."""
        node_name = self.getGeometryObjectNodeName(
            geometry_object, geometry_type, create_groups=True
        )

        geom = geometry_object.geometry
        color_override = color or geometry_object.meshColor

        try:
            if isinstance(geom, hppfcl.Box):
                frame = self.viewer.scene.add_box(
                    node_name,
                    dimensions=geom.halfSide * 2.0,
                    color=color_override[:3],
                    opacity=color_override[3],
                )
            elif isinstance(geom, hppfcl.Sphere):
                frame = self.viewer.scene.add_icosphere(
                    node_name,
                    radius=geom.radius,
                    color=color_override[:3],
                    opacity=color_override[3],
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
                    color=color_override[:3],
                    opacity=color_override[3],
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
                    color=color_override[:3],
                    opacity=color_override[3],
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
                    color=color_override[:3],
                    opacity=color_override[3],
                )
            elif isinstance(geom, MESH_TYPES):
                frame = self._add_mesh_from_path(
                    node_name, geometry_object.meshPath, color_override
                )
            elif isinstance(geom, hppfcl.Convex):
                if len(geometry_object.meshPath) > 0:
                    frame = self._add_mesh_from_path(
                        node_name, geometry_object.meshPath, color_override
                    )
                else:
                    frame = self._add_mesh_from_convex(node_name, geom, color_override)
            else:
                msg = f"Unsupported geometry type for {geometry_object.name} ({type(geom)})"
                warnings.warn(msg, category=UserWarning, stacklevel=2)
                return

            self.viser_frames[node_name] = frame
            
        except Exception as e:
            msg = (
                "Error while loading geometry object: "
                f"{geometry_object.name}\nError message:\n{e}"
            )
            warnings.warn(msg, category=UserWarning, stacklevel=2)

    def _add_mesh_from_path(self, name, mesh_path, color=None):
        """Load a mesh from a file."""
        ext = os.path.splitext(mesh_path)[1].lower()
        
        if ext == ".dae":
            return self._load_collada_mesh(name, mesh_path, color)
        else:
            return self._load_standard_mesh(name, mesh_path, color)

    def _load_collada_mesh(self, name, mesh_path, color):
        """Load a COLLADA mesh with color support."""
        try:
            mesh_collada = collada.Collada(mesh_path)
        except collada.DaeError:
            return self._load_standard_mesh(name, mesh_path, color)
        
        if len(mesh_collada.effects) < len(mesh_collada.geometries):
            return self._load_standard_mesh(name, mesh_path, color)
        
        frames = []
        for i, (geometry, effect) in enumerate(zip(mesh_collada.geometries, mesh_collada.effects)):
            frame = self._process_collada_geometry(name, i, geometry, effect, color, mesh_path)
            if frame:
                frames.append(frame)
        
        return frames[0] if frames else None

    def _process_collada_geometry(self, name, index, geometry, effect, fallback_color, mesh_path):
        """Process a single COLLADA geometry with its material."""
        try:
            vertices, faces = self._extract_geometry_data(geometry)
        except (AttributeError, IndexError, KeyError):
            # Fallback if geometry data extraction fails
            mesh = trimesh.load_mesh(mesh_path)
            return self.viewer.scene.add_mesh_trimesh(f"{name}_{index}", mesh)
        
        mesh_color = getattr(effect, "diffuse", None)
        
        if mesh_color is not None:
            return self.viewer.scene.add_mesh_simple(
                f"{name}_{index}", vertices, faces,
                color=mesh_color[:3], opacity=mesh_color[3]
            )
        elif fallback_color is not None:
            return self.viewer.scene.add_mesh_simple(
                f"{name}_{index}", vertices, faces,
                color=fallback_color[:3], opacity=fallback_color[3]
            )
        else:
            # Let the exception propagate if mesh loading fails
            mesh = trimesh.load_mesh(mesh_path)
            return self.viewer.scene.add_mesh_trimesh(f"{name}_{index}", mesh)

    def _extract_geometry_data(self, geometry):
        """Extract vertices and faces from a COLLADA geometry."""
        vertices = geometry.primitives[0].sources["VERTEX"][0][4].data
        indices = geometry.primitives[0].indices

        if indices.ndim == 3:
            faces = indices[:, :, 0]
        else:
            faces = indices.reshape(-1, 3)

        return vertices, faces

    def _load_standard_mesh(self, name, mesh_path, color):
        """Load a mesh using trimesh."""
        mesh = trimesh.load_mesh(mesh_path)
        if color is None:
            return self.viewer.scene.add_mesh_trimesh(name, mesh)
        else:
            return self.viewer.scene.add_mesh_simple(
                name, mesh.vertices, mesh.faces,
                color=color[:3], opacity=color[3]
            )

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
            name, vertices, faces,
            color=color[:3], opacity=color[3],
        )

    def display(self, q=None):
        """Display the robot at configuration q."""
        if q is not None:
            pin.forwardKinematics(self.model, self.data, q)

        if self.display_visuals and self.visual_model is not None:
            pin.updateGeometryPlacements(
                self.model, self.data, self.visual_model, self.visual_data
            )
            for visual in self.visual_model.geometryObjects:
                node_name = self.getGeometryObjectNodeName(
                    visual, pin.GeometryType.VISUAL
                )
                if node_name in self.viser_frames:
                    M = self.visual_data.oMg[
                        self.visual_model.getGeometryId(visual.name)
                    ]
                    frame = self.viser_frames[node_name]
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
                if node_name in self.viser_frames:
                    M = self.collision_data.oMg[
                        self.collision_model.getGeometryId(collision.name)
                    ]
                    frame = self.viser_frames[node_name]
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
            if node_name in self.viser_frames:
                self.viser_frames[node_name].visible = visibility

    def displayVisuals(self, visibility):
        """Set whether to display visual objects or not."""
        self.display_visuals = visibility
        if self.visual_model is None:
            return

        for visual in self.visual_model.geometryObjects:
            node_name = self.getGeometryObjectNodeName(
                visual, pin.GeometryType.VISUAL
            )
            if node_name in self.viser_frames:
                self.viser_frames[node_name].visible = visibility

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
