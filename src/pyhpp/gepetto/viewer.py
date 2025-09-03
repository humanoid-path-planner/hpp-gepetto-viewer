# BSD 2-Clause License

# Copyright (c) 2014-2025, CNRS - INRIA

# Authors: Gabriele Buondono, Florent Lamiraux, Joris Vaillant

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import warnings

import numpy as np
from numpy.linalg import norm

import pinocchio as pin
from pinocchio.utils import npToTuple
from pinocchio.visualize import BaseVisualizer

try:
    import hppfcl

    WITH_HPP_FCL_BINDINGS = True
except:  # noqa: E722
    WITH_HPP_FCL_BINDINGS = False


class Viewer(BaseVisualizer):
    """An HPP display using Gepetto Viewer"""

    def __init__(self, robot):
        self.robot = robot
        self.viewerRootNodeName = self.robot.name()
        super().__init__(robot.model(), robot.geomModel(), robot.geomModel())
        self.initViewer()

    def getViewerNodeName(self, geometry_object, geometry_type, create_groups = False):
        """Return the name of the geometry object inside the viewer"""
        type_str = "collision" if geometry_type == pin.GeometryType.COLLISION \
            else "visual"
        names = geometry_object.name.split("/")
        assert(len(names) >= 2)
        names = names[:-1] + [type_str] + names[-1:]
        res = self.viewerRootNodeName
        for n in names:
            res += "/" + n
        if create_groups:
            group_name = self.viewerRootNodeName
            if not self.client.gui.nodeExists(group_name):
                self.client.gui.createGroup(group_name)
            for n in names[:-1]:
                node_name = group_name + "/" + n
                if not self.client.gui.nodeExists(node_name):
                    self.client.gui.createGroup(node_name)
                    self.client.gui.addToGroup(node_name, group_name)
                group_name = node_name
        return res

    def initViewer(
        self,
        viewer=None,
        loadModel=True,
    ):
        """Init GepettoViewer by loading the gui and creating a window."""
        windowName="hpp"
        sceneName=self.robot.name()

        try:
            import gepetto.corbaserver
        except ImportError:
            import warnings

            msg = (
                "Error while importing the viewer client.\n"
                "Check whether gepetto-gui is properly installed"
            )
            warnings.warn(msg, category=UserWarning, stacklevel=2)

        try:
            self.client = gepetto.corbaserver.Client() if viewer is None else viewer
            gui = self.client.gui

            # Create window
            window_l = gui.getWindowList()
            if windowName not in window_l:
                self.windowID = self.client.gui.createWindow(windowName)
            else:
                self.windowID = self.client.gui.getWindowID(windowName)

            # Create scene if needed
            scene_l = gui.getSceneList()
            if sceneName not in scene_l:
                gui.createScene(sceneName)
            self.sceneName = sceneName
            gui.addSceneToWindow(sceneName, self.windowID)

            if loadModel:
                self.loadViewerModel()
        except:  # noqa: E722
            import warnings

            msg = (
                "Error while starting the viewer client.\n"
                "Check whether gepetto-viewer is properly started"
            )
            warnings.warn(msg, category=UserWarning, stacklevel=2)

    def loadPrimitive(self, meshName, geometry_object):
        gui = self.client.gui

        meshColor = geometry_object.meshColor

        geom = geometry_object.geometry
        if isinstance(geom, hppfcl.Capsule):
            return gui.addCapsule(
                meshName, geom.radius, 2.0 * geom.halfLength, npToTuple(meshColor)
            )
        elif isinstance(geom, hppfcl.Cylinder):
            return gui.addCylinder(
                meshName, geom.radius, 2.0 * geom.halfLength, npToTuple(meshColor)
            )
        elif isinstance(geom, hppfcl.Box):
            w, h, d = npToTuple(2.0 * geom.halfSide)
            return gui.addBox(meshName, w, h, d, npToTuple(meshColor))
        elif isinstance(geom, hppfcl.Sphere):
            return gui.addSphere(meshName, geom.radius, npToTuple(meshColor))
        elif isinstance(geom, hppfcl.Cone):
            return gui.addCone(
                meshName, geom.radius, 2.0 * geom.halfLength, npToTuple(meshColor)
            )
        elif isinstance(geom, hppfcl.Plane) or isinstance(geom, hppfcl.Halfspace):
            res = gui.createGroup(meshName)
            if not res:
                return False
            planeName = meshName + "/plane"
            res = gui.addFloor(planeName)
            if not res:
                return False
            normal = geom.n
            rot = pin.Quaternion.FromTwoVectors(normal, pin.ZAxis)
            alpha = geom.d / norm(normal, 2) ** 2
            trans = alpha * normal
            plane_offset = pin.SE3(rot, trans)
            gui.applyConfiguration(planeName, pin.SE3ToXYZQUATtuple(plane_offset))
        elif isinstance(geom, hppfcl.Convex):
            pts = [
                npToTuple(geom.points(geom.polygons(f)[i]))
                for f in range(geom.num_polygons)
                for i in range(3)
            ]
            gui.addCurve(meshName, pts, npToTuple(meshColor))
            gui.setCurveMode(meshName, "TRIANGLES")
            gui.setLightingMode(meshName, "ON")
            gui.setBoolProperty(meshName, "BackfaceDrawing", True)
            return True
        elif isinstance(geom, hppfcl.ConvexBase):
            pts = [npToTuple(geom.points(i)) for i in range(geom.num_points)]
            gui.addCurve(meshName, pts, npToTuple(meshColor))
            gui.setCurveMode(meshName, "POINTS")
            gui.setLightingMode(meshName, "OFF")
            return True
        else:
            msg = f"Unsupported geometry type for {geometry_object.name} ({type(geom)})"
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return False

    def loadViewerGeometryObject(self, geometry_object, geometry_type):
        """Load a single geometry object"""

        gui = self.client.gui

        meshName = self.getViewerNodeName(geometry_object, geometry_type, True)
        meshPath = geometry_object.meshPath
        meshTexturePath = geometry_object.meshTexturePath
        meshScale = geometry_object.meshScale
        meshColor = geometry_object.meshColor

        try:
            if WITH_HPP_FCL_BINDINGS and isinstance(
                geometry_object.geometry, hppfcl.ShapeBase
            ):
                success = self.loadPrimitive(meshName, geometry_object)
            else:
                if meshName == "":
                    msg = (
                        "Display of geometric primitives is supported only if "
                        "pinocchio is build with HPP-FCL bindings."
                    )
                    warnings.warn(msg, category=UserWarning, stacklevel=2)
                    return
                success = gui.addMesh(meshName, meshPath)
            if not success:
                return
        except Exception as e:
            msg = (
                "Error while loading geometry object: "
                f"{geometry_object.name}\nError message:\n{e}"
            )
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        gui.setScale(meshName, npToTuple(meshScale))
        if geometry_object.overrideMaterial:
            gui.setColor(meshName, npToTuple(meshColor))
            if meshTexturePath != "":
                gui.setTexture(meshName, meshTexturePath)

    def loadViewerModel(self):
        """Create the scene displaying the robot meshes in gepetto-viewer"""

        # Start a new "scene" in this window, named "world", with just a floor.
        gui = self.client.gui

        if not gui.nodeExists(self.viewerRootNodeName):
            gui.createGroup(self.viewerRootNodeName)

        # iterate over visuals and create the meshes in the viewer
        if self.collision_model is not None:
            for collision in self.collision_model.geometryObjects:
                self.loadViewerGeometryObject(collision, pin.GeometryType.COLLISION)
        # Display collision if we have them and there is no visual
        self.displayCollisions(
            self.collision_model is not None and self.visual_model is None
        )

        if self.visual_model is not None:
            for visual in self.visual_model.geometryObjects:
                self.loadViewerGeometryObject(visual, pin.GeometryType.VISUAL)
        self.displayVisuals(self.visual_model is not None)

        # Finally, refresh the layout to obtain your first rendering.
        gui.refresh()

    def display(self, q=None):
        """
        Display the robot at configuration q in the viewer by placing all the bodies.
        """
        gui = self.client.gui
        # Update the robot kinematics and geometry.
        if q is not None:
            pin.forwardKinematics(self.model, self.data, q)

        if self.display_collisions:
            pin.updateGeometryPlacements(
                self.model, self.data, self.collision_model, self.collision_data
            )
            gui.applyConfigurations(
                [
                    self.getViewerNodeName(collision, pin.GeometryType.COLLISION)
                    for collision in self.collision_model.geometryObjects
                ],
                [
                    pin.SE3ToXYZQUATtuple(
                        self.collision_data.oMg[
                            self.collision_model.getGeometryId(collision.name)
                        ]
                    )
                    for collision in self.collision_model.geometryObjects
                ],
            )

        if self.display_visuals:
            pin.updateGeometryPlacements(
                self.model, self.data, self.visual_model, self.visual_data
            )
            gui.applyConfigurations(
                [
                    self.getViewerNodeName(visual, pin.GeometryType.VISUAL)
                    for visual in self.visual_model.geometryObjects
                ],
                [
                    pin.SE3ToXYZQUATtuple(
                        self.visual_data.oMg[
                            self.visual_model.getGeometryId(visual.name)
                        ]
                    )
                    for visual in self.visual_model.geometryObjects
                ],
            )

        gui.refresh()

    def displayCollisions(self, visibility):
        """Set whether to display collision objects or not"""
        gui = self.client.gui
        self.display_collisions = visibility
        if self.collision_model is None:
            return

        if visibility:
            visibility_mode = "ON"
        else:
            visibility_mode = "OFF"

        for collision in self.collision_model.geometryObjects:
            nodeName = self.getViewerNodeName(collision, pin.GeometryType.COLLISION)
            gui.setVisibility(nodeName, visibility_mode)

    def displayVisuals(self, visibility):
        """Set whether to display visual objects or not"""
        gui = self.client.gui
        self.display_visuals = visibility
        if self.visual_model is None:
            return

        if visibility:
            visibility_mode = "ON"
        else:
            visibility_mode = "OFF"

        for visual in self.visual_model.geometryObjects:
            nodeName = self.getViewerNodeName(visual, pin.GeometryType.VISUAL)
            gui.setVisibility(nodeName, visibility_mode)

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

    def captureImage(self, w=None, h=None):
        raise NotImplementedError()

    def disableCameraControl(self):
        raise NotImplementedError()

    def enableCameraControl(self):
        raise NotImplementedError()

    def drawFrameVelocities(self, *args, **kwargs):
        raise NotImplementedError()


__all__ = ["Viewer"]
