#!/usr/bin/env python

# Copyright (c) 2014 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-gepetto-viewer.
# hpp-gepetto-viewer is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-gepetto-viewer is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-gepetto-viewer.  If not, see
# <http://www.gnu.org/licenses/>.

import math

import omniORB.any

from gepetto.color import Color
from gepetto.corbaserver.client import _GhostGraphicalInterface
from hpp.quaternion import Quaternion


class _GhostViewerClient:
    def __init__(self):
        self.gui = _GhostGraphicalInterface()


def _urdfPath(Type):
    return (
        "package://"
        + Type.packageName
        + "/urdf/"
        + Type.urdfName
        + Type.urdfSuffix
        + ".urdf"
    )


def _srdfPath(Type):
    return (
        "package://"
        + Type.packageName
        + "/srdf/"
        + Type.urdfName
        + Type.srdfSuffix
        + ".srdf"
    )


def _urdfSrdfFilenames(Type):
    """Return urdf and srdf filenames."""
    # if packageName, urdfName, urdfSuffix, srdfSuffix are members of the
    # class, build urdf and srdf filenames
    if _urdfSrdfString(Type):
        return Type.urdfString, Type.srdfString
    if (
        hasattr(Type, "packageName")
        and hasattr(Type, "urdfName")
        and hasattr(Type, "urdfSuffix")
        and hasattr(Type, "srdfSuffix")
    ):
        urdfFilename = _urdfPath(Type)
        srdfFilename = _srdfPath(Type)
    elif hasattr(Type, "urdfFilename") and hasattr(Type, "srdfFilename"):
        urdfFilename = Type.urdfFilename
        srdfFilename = Type.srdfFilename
    else:
        raise RuntimeError(
            """instance should have one of the following sets of members
         - (packageName, urdfName, urdfSuffix, srdfSuffix),
         - (urdfFilename, srdfFilename)"""
        )
    return urdfFilename, srdfFilename


def _urdfSrdfString(Type):
    return hasattr(Type, "urdfString") and hasattr(Type, "srdfString")


def hppToViewerTransform(input):
    return input


class Viewer:
    """
    Simultaneous control of hppcorbaserver and gepetto-viewer-server

    This class implements clients to both
    \\li hppcorbaserver through hpp.corbaserver.problem_solver.ProblemSolver
        python class,
    \\li gepetto-viewer-server through gepetto.corbaserver.Client python class.

    Operation that need to be synchronized between hppcorbaserver internal
    model and graphical user interface should be implemented by this class.
    """

    # Default scene name
    #  Useful when debugging for creating an instance with a client to an
    #  existing server.
    sceneName = "0_scene_hpp_"

    # Whether light sources in collada files should be removed at loading.
    removeLightSources = True

    def __init__(
        self,
        problemSolver,
        viewerClient=None,
        ghost=False,
        collisionURDF=False,
        displayName=None,
        displayArrows=False,
        displayCoM=False,
    ):
        """
        Constructor
        \\param problemSolver object of type ProblemSolver
        \\param viewerClient if not provided, a new client to
               gepetto-viewer-server is created.
        \\param displayArrow if True, the publish method will display 2 arrows
                             representing the velocity (green) and acceleration
                             (red) of the root.
                   This parameter can only be used if the robot have at least 6
                   extraDOF storing the velocity and acceleration of the root.
        \\param displayCoM if True, the publish method will also display a small red
                           sphere representing the position of the CoM for the
                           published configuration.

        The robot loaded in hppcorbaserver is loaded into gepetto-viewer-server.
        """

        from gepetto.corbaserver import Client as GuiClient

        self.problemSolver = problemSolver
        self.robot = problemSolver.robot
        self.collisionURDF = collisionURDF
        self.color = Color()
        if not viewerClient:
            try:
                viewerClient = GuiClient()
            except Exception as e:
                if ghost:
                    print("Failed to connect to the viewer.")
                    print("Check whether gepetto-gui is properly started.")
                    viewerClient = _GhostViewerClient()
                else:
                    raise e
        self.createWindowAndScene(viewerClient, "hpp_")
        self.client = viewerClient
        self.callbacks = []
        if displayName is not None:
            self.displayName = displayName
        else:
            self.displayName = self.robot.displayName
        # Load robot in viewer
        self.buildRobotBodies()
        self._initDisplay()
        # create velocity and acceleration arrows :
        self.displayArrows = displayArrows
        if displayArrows:
            if self.robot.client.robot.getDimensionExtraConfigSpace() < 6:
                raise RuntimeError(
                    "displayArrows can only be True if the robot have at least 6 "
                    "extraDof storing velocity and acceleration of the root."
                )
            self.colorVelocity = [0.2, 1, 0, 0.6]
            self.colorAcceleration = [1, 0, 0, 0.6]
            self.arrowRadius = 0.01
            self.arrowMinSize = 0.05
            self.arrowMaxSize = 1.0 - self.arrowMinSize
            if (
                self.client.gui.getNodeList() is not None
                and "Vec_Velocity" not in self.client.gui.getNodeList()
            ):
                self.client.gui.addArrow(
                    "Vec_Velocity",
                    self.arrowRadius,
                    self.arrowMinSize,
                    self.colorVelocity,
                )
                self.client.gui.addToGroup("Vec_Velocity", self.sceneName)
                self.client.gui.setVisibility("Vec_Velocity", "OFF")
                self.client.gui.addArrow(
                    "Vec_Acceleration",
                    self.arrowRadius,
                    self.arrowMinSize,
                    self.colorAcceleration,
                )
                self.client.gui.addToGroup("Vec_Acceleration", self.sceneName)
                self.client.gui.setVisibility("Vec_Acceleration", "OFF")
            self.amax = omniORB.any.from_any(
                self.problemSolver.hppcorba.problem.getParameter(
                    "Kinodynamic/accelerationBound"
                )
            )
            self.vmax = omniORB.any.from_any(
                self.problemSolver.hppcorba.problem.getParameter(
                    "Kinodynamic/velocityBound"
                )
            )
        self.displayCoM = displayCoM

    # Set lighting mode OFF for a group of nodes
    #
    # Some robot models have light produced by their links. This produces a
    # undesired effect in gepetto-gui.
    def _removeLightSources(self, nodes):
        if not self.removeLightSources or nodes is None:
            return
        for n in nodes:
            properties = self.client.gui.getPropertyNames(n)
            if "RemoveLightSources" in properties:
                self.client.gui.removeLightSources(n)
            # Recursively explore child nodes
            children = self.client.gui.getGroupNodeList(n)
            self._removeLightSources(children)

    def _initDisplay(self):
        urdfFilename, srdfFilename = self.robot.urdfSrdfFilenames()
        self.client.gui.addURDF(self.displayName, urdfFilename)
        # Remove lighting from meshes
        self._removeLightSources(self.client.gui.getGroupNodeList(self.displayName))
        if self.collisionURDF:
            self.toggleVisual(False)
        self.client.gui.addToGroup(self.displayName, self.sceneName)

    def createWindowAndScene(self, viewerClient, name):
        from gepetto import Error as GepettoError

        self.windowName = "scene_" + name
        try:
            self.windowId = viewerClient.gui.getWindowID(self.windowName)
        except GepettoError:
            self.windowId = viewerClient.gui.createWindow(self.windowName)
        self.sceneName = self.windowName

    def addCallback(self, cb):
        """
        \\param cb Callable object, whose arguments are this object
        and a robot configuration.
        """
        self.callbacks.append(cb)

    def buildRobotBodies(self):
        self.robotBodies = list()
        # build list of pairs (robotName, objectName)
        for j in self.robot.getAllJointNames():
            self.robotBodies.extend(
                [(j, self.displayName + "/", n) for n in self.robot.getLinkNames(j)]
            )
        self.guiObjectNames = [prefix + o for j, prefix, o in self.robotBodies]
        self.hppObjectNames = [o for j, prefix, o in self.robotBodies]

    def addLandmark(self, linkname, size):
        """
         Add a landmark
        \\sa gepetto::corbaserver::GraphicalInterface::addLandmark
        """
        return self.client.gui.addLandmark(linkname, size)

    def displayPathMap(
        self,
        nameRoadmap,
        pathID,
        radiusSphere=0.03,
        sizeAxis=0.09,
        colorNode=[1, 0.0, 0.0, 1.0],
        colorEdge=[1, 0.0, 0.0, 0.5],
        joint=None,
    ):
        """
        Display the part of the roadmap used by the solution path
        \\param nameRoadmap : the name of the osgNode added to the scene
        \\param pathID : the id of the path we want to display
        \\param radiusSphere : the radius of the node
        \\param sizeAxis : size of axes (proportionnaly to the radius of the sphere)
                           0 = only sphere
        \\param colorNode : the color of the sphere for the nodes (default value : red)
        \\param colorEdge : the color of the edges (default value : light red)
        \\param joint : the link we want to display the configuration
                        (by defaut, root link of the robot)
        BE CAREFULL : in the .py file wich init the robot, you must define a valid
                        tf_root (this is the displayed joint by default)
        notes : the edges are always straight lines and doesn't represent the real
                path beetwen the configurations of the nodes
        """
        ps = self.problemSolver
        gui = self.client.gui
        robot = self.robot
        rootJoint = robot.getJointNames()[0]
        lastPos = 0
        currentPos = 0
        # find the link :
        if joint is None:
            joint = rootJoint
        if ps.numberNodes() == 0:
            return False
        if not gui.createRoadmap(
            nameRoadmap, colorNode, radiusSphere, sizeAxis, colorEdge
        ):
            return False
        waypoints = ps.getWaypoints(pathID)[0]
        for i in range(0, len(waypoints)):
            robot.setCurrentConfig(waypoints[i])
            currentPos = robot.getJointPosition(joint)
            gui.addNodeToRoadmap(nameRoadmap, currentPos)
            if i > 0:
                gui.addEdgeToRoadmap(nameRoadmap, lastPos[0:3], currentPos[0:3])
            lastPos = currentPos

        gui.addToGroup(nameRoadmap, self.sceneName)
        gui.setVisibility(nameRoadmap, "ALWAYS_ON_TOP")
        gui.refresh()
        return True

    def displayRoadmap(
        self,
        nameRoadmap,
        radiusSphere=0.01,
        sizeAxis=0.03,
        colorNode=[1.0, 1.0, 1.0, 1.0],
        colorEdge=[0.85, 0.75, 0.15, 0.7],
        joint=None,
    ):
        """
        Display the roadmap created by problem.solve()
        \\param radiusSphere : the radius of the node
        \\param sizeAxis : size of axes (proportionnaly to the radius of the sphere)
                          0 = only sphere
        \\param colorNode : the color of the sphere for the nodes
                            (default value : white)
        \\param colorEdge : the color of the edges (default value : yellow)
        \\param joint : the link we want to display the configuration
                       (by defaut, root link of the robot)
        BE CAREFULL : in the .py file wich init the robot, you must define a valid
                      tf_root (this is the displayed joint by default)
        notes : the edges are always straight lines and doesn't represent the real
                path beetwen the configurations of the nodes
        """

        ps = self.problemSolver
        gui = self.client.gui
        robot = self.robot
        rootJoint = robot.getJointNames()[0]
        # find the link :
        if joint is None:
            joint = rootJoint
        if ps.numberNodes() == 0:
            return False
        if not gui.createRoadmap(
            nameRoadmap, colorNode, radiusSphere, sizeAxis, colorEdge
        ):
            return False
        # set the start in green and the goal in red :
        gui.addSphere(nameRoadmap + "/start", radiusSphere * 1.5, self.color.green)
        robot.setCurrentConfig(ps.node(0))
        gui.applyConfiguration(nameRoadmap + "start", robot.getJointPosition(joint))
        gui.addToGroup(nameRoadmap + "/start", self.sceneName)
        if ps.numberNodes() == 1:
            return True
        gui.addSphere(nameRoadmap + "/goal", radiusSphere * 1.5, self.color.red)
        robot.setCurrentConfig(ps.node(1))
        gui.applyConfiguration(nameRoadmap + "/goal", robot.getJointPosition(joint))
        gui.addToGroup(nameRoadmap + "/goal", self.sceneName)
        # add all the nodes :
        for i in range(0, ps.numberNodes()):
            robot.setCurrentConfig(ps.node(i))
            gui.addNodeToRoadmap(nameRoadmap, robot.getJointPosition(joint))
        for i in range(0, ps.numberEdges()):
            robot.setCurrentConfig(ps.edge(i)[0])
            e0 = robot.getJointPosition(joint)[0:3]
            robot.setCurrentConfig(ps.edge(i)[1])
            e1 = robot.getJointPosition(joint)[0:3]
            gui.addEdgeToRoadmap(nameRoadmap, e0, e1)
        gui.addToGroup(nameRoadmap, self.sceneName)
        gui.refresh()
        return True

    def solveAndDisplay(
        self,
        nameRoadmap,
        numberIt,
        radiusSphere=0.01,
        sizeAxis=0.03,
        colorNode=[1.0, 1.0, 1.0, 1.0],
        colorEdge=[0.85, 0.75, 0.15, 0.7],
        joint=None,
    ):
        """
        build the roadmap and diplay it during construction
        (delete existing roadmap if problem already solved )
        \\param nameRoadmap : name of the new roadmap
        \\param numberIt : number of iteration beetwen to refresh of the roadmap
        (be careful, if numberIt is too low it can crash gepetto-viewer-server)
        \\param radiusSphere : the radius of the node
        \\param sizeAxis : size of axes (proportionnaly to the radius of the sphere)
                            0 = only sphere
        \\param colorNode : the color of the sphere for the nodes
                            (default value : white)
        \\param colorEdge : the color of the edges (default value : yellow)
        \\param joint : the link we want to display the configuration
                        (by defaut, root link of the robot)
        """

        import time

        ps = self.problemSolver
        problem = self.problemSolver.client.problem
        gui = self.client.gui
        robot = self.robot
        rootJoint = robot.getJointNames()[0]
        # find the link :
        if joint is None:
            joint = rootJoint
        if ps.numberNodes() > 0:
            ps.clearRoadmap()
        tStart = time.time()
        if problem.prepareSolveStepByStep():
            problem.finishSolveStepByStep()
            self.displayRoadmap(
                nameRoadmap, radiusSphere, sizeAxis, colorNode, colorEdge, joint
            )
            tStop = time.time()
            return tStop - tStart
        beginEdge = ps.numberEdges()
        beginNode = ps.numberNodes()
        it = 1
        self.displayRoadmap(
            nameRoadmap, radiusSphere, sizeAxis, colorNode, colorEdge, joint
        )
        while not problem.executeOneStep():
            if it == numberIt:
                for i in range(beginNode, ps.numberNodes()):
                    if joint == 0:
                        gui.addNodeToRoadmap(nameRoadmap, ps.node(i)[0:7])
                    else:
                        robot.setCurrentConfig(ps.node(i))
                        gui.addNodeToRoadmap(nameRoadmap, robot.getJointPosition(joint))
                for i in range(beginEdge, ps.numberEdges()):
                    if joint == 0:
                        gui.addEdgeToRoadmap(
                            nameRoadmap, ps.edge(i)[0][0:3], ps.edge(i)[1][0:3]
                        )
                    else:
                        robot.setCurrentConfig(ps.edge(i)[0])
                        e0 = robot.getJointPosition(joint)[0:3]
                        robot.setCurrentConfig(ps.edge(i)[1])
                        e1 = robot.getJointPosition(joint)[0:3]
                        gui.addEdgeToRoadmap(nameRoadmap, e0, e1)
                beginNode = ps.numberNodes()
                beginEdge = ps.numberEdges()
                it = 1
            else:
                it = it + 1
        problem.finishSolveStepByStep()
        # display new edge (node ?) added by finish()
        for i in range(beginNode, ps.numberNodes()):
            if joint == 0:
                gui.addNodeToRoadmap(nameRoadmap, ps.node(i)[0:7])
            else:
                robot.setCurrentConfig(ps.node(i))
                gui.addNodeToRoadmap(nameRoadmap, robot.getJointPosition(joint))
        for i in range(beginEdge, ps.numberEdges()):
            if joint == 0:
                gui.addEdgeToRoadmap(
                    nameRoadmap, ps.edge(i)[0][0:3], ps.edge(i)[1][0:3]
                )
            else:
                robot.setCurrentConfig(ps.edge(i)[0])
                e0 = robot.getJointPosition(joint)[0:3]
                robot.setCurrentConfig(ps.edge(i)[1])
                e1 = robot.getJointPosition(joint)[0:3]
                gui.addEdgeToRoadmap(nameRoadmap, e0, e1)
        problem.optimizePath(problem.numberPaths() - 1)
        tStop = time.time()
        return tStop - tStart

    def loadObstacleModel(self, filename, prefix, guiOnly=False):
        """
        Load obstacles from a urdf file

        \\param filename name of the urdf file, may contain "package://"
        \\param prefix prefix added to object names in case the same file
               is loaded several times,
        \\param guiOnly whether to control only gepetto-viewer-server
        """
        if not guiOnly:
            self.problemSolver.loadObstacleFromUrdf(filename, prefix + "/")
        self.client.gui.addUrdfObjects(prefix, filename, not self.collisionURDF)
        # Remove lighting from meshes
        self._removeLightSources(self.client.gui.getGroupNodeList(prefix))
        self.client.gui.addToGroup(prefix, self.sceneName)
        self.computeObjectPosition()

    def loadPolyhedronObstacleModel(self, name, filename, guiOnly=False):
        """
        Load polyhedron from a 3D mesh file

        \\param filename name of the 3D mesh file, may contain "package://"
        \\param name name of the object,
        \\param guiOnly whether to control only gepetto-viewer-server
        """
        if not guiOnly:
            self.problemSolver.hppcorba.obstacle.loadPolyhedron(name, filename)
        self.client.gui.addMesh(name, filename)
        # Remove lighting from meshes
        self._removeLightSources(
            [
                name,
            ]
        )
        self.client.gui.addToGroup(name, self.sceneName)
        self.computeObjectPosition()

    def moveObstacle(self, name, position, guiOnly=False):
        """
        Move Obstacle

        \\param name Name of the object
        \\param position Position of the object as a 7-d vector
               (translation-quaternion)
        \\param guiOnly whether to control only gepetto-viewer-server
        """
        if not guiOnly:
            self.problemSolver.moveObstacle(name, position)
        self.computeObjectPosition()

    def computeObjectPosition(self):
        """
        Synchronize object positions in gepetto-viewer-server

        Get position of objects from hppcorbaserver and forward to
        gepetto-viewer-server.
        """
        # compute object positions
        objects = self.problemSolver.getObstacleNames(True, False)
        for o in objects:
            pos = self.problemSolver.getObstaclePosition(o)
            self.client.gui.applyConfiguration(o, pos)
        self.client.gui.refresh()

    def publishRobots(self):
        self.robot.setCurrentConfig(self.robotConfig)
        self.client.gui.applyConfigurations(
            self.guiObjectNames,
            [self.robot.getLinkPosition(o) for o in self.hppObjectNames],
        )
        # display velocity and acceleration arrows :
        if self.displayArrows:
            if self.robot.client.robot.getDimensionExtraConfigSpace() >= 6:
                configSize = (
                    self.robot.getConfigSize()
                    - self.robot.client.robot.getDimensionExtraConfigSpace()
                )
                q = self.robotConfig[::]
                qV = (
                    q[0:3]
                    + Quaternion()
                    .fromTwoVector([1, 0, 0], q[configSize : configSize + 3])
                    .array.tolist()
                )
                qA = (
                    q[0:3]
                    + Quaternion()
                    .fromTwoVector([1, 0, 0], q[configSize + 3 : configSize + 6])
                    .array.tolist()
                )
                v = (
                    math.sqrt(
                        q[configSize] * q[configSize]
                        + q[configSize + 1] * q[configSize + 1]
                        + q[configSize + 2] * q[configSize + 2]
                    )
                ) / self.vmax
                a = (
                    math.sqrt(
                        q[configSize + 3] * q[configSize + 3]
                        + q[configSize + 1 + 3] * q[configSize + 1 + 3]
                        + q[configSize + 2 + 3] * q[configSize + 2 + 3]
                    )
                ) / self.amax
                if v > 0:
                    self.client.gui.resizeArrow(
                        "Vec_Velocity",
                        self.arrowRadius,
                        self.arrowMinSize + v * self.arrowMaxSize,
                    )
                    self.client.gui.applyConfiguration("Vec_Velocity", qV[0:7])
                    self.client.gui.setVisibility("Vec_Velocity", "ALWAYS_ON_TOP")
                else:
                    self.client.gui.setVisibility("Vec_Velocity", "OFF")
                    self.client.gui.resizeArrow("Vec_Velocity", self.arrowRadius, 0)
                    self.client.gui.applyConfiguration("Vec_Velocity", qV[0:7])
                if a > 0:
                    self.client.gui.resizeArrow(
                        "Vec_Acceleration",
                        self.arrowRadius,
                        self.arrowMinSize + a * self.arrowMaxSize,
                    )
                    self.client.gui.applyConfiguration("Vec_Acceleration", qA[0:7])
                    self.client.gui.setVisibility("Vec_Acceleration", "ALWAYS_ON_TOP")
                else:
                    self.client.gui.setVisibility("Vec_Acceleration", "OFF")
                    self.client.gui.resizeArrow("Vec_Acceleration", self.arrowRadius, 0)
                    self.client.gui.applyConfiguration("Vec_Acceleration", qA[0:7])
        if self.displayCoM:
            name = "sphere_CoM"
            if (
                self.client.gui.getNodeList() is not None
                and name not in self.client.gui.getNodeList()
            ):
                self.client.gui.addSphere(name, 0.01, self.color.red)
                self.client.gui.setVisibility(name, "ALWAYS_ON_TOP")
                self.client.gui.addToGroup(name, self.sceneName)
            self.client.gui.applyConfiguration(
                name, [*self.robot.getCenterOfMass(), 0, 0, 0, 1]
            )

    def __call__(self, args):
        self.robotConfig = args
        self.publishRobots()
        for cb in self.callbacks:
            cb(self, args)
        self.client.gui.refresh()

    def startCapture(self, filename, extension):
        """
        Start a screen capture
        \\sa gepetto::corbaserver::GraphicalInterface::startCapture
        """
        return self.client.gui.startCapture(self.windowId, filename, extension)

    def stopCapture(self):
        """
        Stop a screen capture
        \\sa gepetto::corbaserver::GraphicalInterface::stopCapture
        """
        return self.client.gui.stopCapture(self.windowId)

    def captureFrame(self, filename):
        """
        Save current scene to image.
        \\sa gepetto::corbaserver::GraphicalInterface::captureFrame
        """
        return self.client.gui.captureFrame(self.windowId, filename)

    def toggleVisual(self, visual):
        for n in self.client.gui.getGroupNodeList(self.displayName):
            self.client.gui.setBoolProperty(n, "ShowVisual", visual)

    def drawRobotAABB(self):
        aabb = self.robot.getRobotAABB()
        n = self.sceneName + "/robotAABB"
        if self.client.gui.nodeExists(n):
            self.client.gui.deleteNode(n, True)
        self.client.gui.addBox(
            n,
            (aabb[3] - aabb[0]) / 2,
            (aabb[4] - aabb[1]) / 2,
            (aabb[5] - aabb[2]) / 2,
            self.color.black,
        )
        self.client.gui.applyConfiguration(
            n,
            [
                (aabb[0] + aabb[3]) / 2,
                (aabb[1] + aabb[4]) / 2,
                (aabb[2] + aabb[5]) / 2,
                0,
                0,
                0,
                1,
            ],
        )
        self.client.gui.setWireFrameMode(n, "WIREFRAME")
        self.client.gui.refresh()
