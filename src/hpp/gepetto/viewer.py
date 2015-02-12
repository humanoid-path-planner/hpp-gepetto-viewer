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

import os.path
import rospkg
from gepetto.corbaserver import Client as GuiClient

rospack = rospkg.RosPack()

## Simultaneous control of hppcorbaserver and gepetto-viewer-server
#
#  This class implements clients to both
#    \li hppcorbaserver through hpp.corbaserver.problem_solver.ProblemSolver
#        python class,
#    \li gepetto-viewer-server through gepetto.corbaserver.Client python class.
#
#  Operation that need to be synchronized between hppcorbaserver internal
#  model and graphical user interface should be implemented by this class.
class Viewer (object):
    withFloor = False
    @staticmethod
    def createWindowAndScene (viewerClient, name):
        Viewer.windowName = "window_" + name
        Viewer.sceneName = "scene_" + name
        if not viewerClient.gui.createWindow (Viewer.windowName):
            raise RuntimeError ('Failed to create window "%s"'%
                                Viewer.windowName)
        if Viewer.withFloor:
            viewerClient.gui.createSceneWithFloor ("scene_" + name)
        else:
            viewerClient.gui.createScene ("scene_" + name)
        if not viewerClient.gui.addSceneToWindow (Viewer.sceneName,
                                                  Viewer.windowName):
            raise RuntimeError ('Failed to add scene "%s" to window "%s"'%
                                (Viewer.sceneName, Viewer.windowName))

    ## Constructor
    #  \param problemSolver object of type ProblemSolver
    #  \param viewerClient if not provided, a new client to
    #         gepetto-viewer-server is created.
    #
    #  The robot loaded in hppcorbaserver is loaded into gepetto-viewer-server.
    def __init__ (self, problemSolver, viewerClient = None):
        self.problemSolver = problemSolver
        self.robot = problemSolver.robot
        if not viewerClient:
            viewerClient = GuiClient ()
            self.createWindowAndScene (viewerClient, "hpp_")
        self.client = viewerClient
        self.displayName = self.robot.displayName
        if hasattr (self.robot, 'meshPackageName'):
            meshPackageName = self.robot.meshPackageName
        else:
            meshPackageName = self.robot.packageName
        # Load robot in viewer
        self.buildRobotBodies ()
        rospack = rospkg.RosPack()
        packagePath = rospack.get_path (self.robot.packageName)
        meshPackagePath = rospack.get_path (meshPackageName)
        dataRootDir = os.path.dirname (meshPackagePath) + "/"
        packagePath += '/urdf/' + self.robot.urdfName + self.robot.urdfSuffix +\
            '.urdf'
        self.client.gui.addURDF (self.displayName, packagePath, dataRootDir)
        self.client.gui.addToGroup (self.displayName, self.sceneName)

    def buildRobotBodies (self):
        self.robotBodies = list ()
        # build list of pairs (robotName, objectName)
        for j in self.robot.getAllJointNames ():
            self.robotBodies.extend (map (lambda n:
                                              (j, self.displayName + "/", n),
                                          [self.robot.getLinkName (j),]))

    ## Load obstacles from a urdf file
    #
    #  \param package ros package containing the urdf file,
    #  \param filename name of the urdf file without extension,
    #  \param prefix prefix added to object names in case the same file
    #         is loaded several times,
    #  \param meshPackageName ros package containing the geometry files
    #         (collada, stl,...) if different from package
    #  \param guiOnly whether to control only gepetto-viewer-server
    def loadObstacleModel (self, package, filename, prefix,
                           meshPackageName = None, guiOnly = False):
        if not meshPackageName:
            meshPackageName = package
        if not guiOnly:
            self.problemSolver.loadObstacleFromUrdf (package, filename, prefix+'/')
        rospack = rospkg.RosPack()
        packagePath = rospack.get_path (package)
        meshPackagePath = rospack.get_path (meshPackageName)
        dataRootDir = os.path.dirname (meshPackagePath) + "/"
        packagePath += '/urdf/' + filename + '.urdf'
        self.client.gui.addUrdfObjects (prefix, packagePath, dataRootDir, True)
        self.client.gui.addToGroup (prefix, self.sceneName)
        self.computeObjectPosition ()

    ## Synchronize object positions in gepetto-viewer-server
    #
    #  Get position of objects from hppcorbaserver and forward to
    #  gepetto-viewer-server.
    def computeObjectPosition (self):
        # compute object positions
        objects = self.problemSolver.getObstacleNames (True, False)
        for o in objects:
            pos = self.problemSolver.getObstaclePosition (o)
            self.client.gui.applyConfiguration (o, pos)
        self.client.gui.refresh ()

    def publishRobots (self):
        self.robot.setCurrentConfig (self.robotConfig)
        for j, prefix, o in self.robotBodies:
            pos = self.robot.getLinkPosition (j)
            objectName = prefix + o
            self.client.gui.applyConfiguration (objectName, pos)
        self.client.gui.refresh ()

    def __call__ (self, args):
        self.robotConfig = args
        self.publishRobots ()
