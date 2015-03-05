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

import os
import rospkg
from hpp.gepetto import Viewer as Parent

## Simultaneous control to hpp-manipulation-server and gepetto-viewer-server.
#
class Viewer (Parent):
    def __init__ (self, problemSolver, viewerClient = None) :
        Parent.__init__ (self, problemSolver, viewerClient)

    def buildRobotBodies (self):
        self.robotBodies = list ()
        # build list of pairs (robotName, objectName)
        for j in self.robot.getAllJointNames ():
            # Guess robot name from joint name
            prefix = j.split ('/') [0]
            self.robotBodies.append ((j, '', self.robot.getLinkName (j)))

    def loadRobotModel (self, RobotType, robotName, guiOnly = False):
        self.robot.insertRobotModel (robotName, RobotType.rootJointType,
                                   RobotType.packageName,
                                   RobotType.modelName, RobotType.urdfSuffix,
                                   RobotType.srdfSuffix)
        self.buildRobotBodies ()
        self.loadUrdfInGUI (RobotType, robotName)

    def loadHumanoidModel (self, RobotType, robotName, guiOnly = False):
        self.robot.loadHumanoidModel (robotName, RobotType.rootJointType,
                                      RobotType.packageName,
                                      RobotType.modelName, RobotType.urdfSuffix,
                                      RobotType.srdfSuffix)
        self.buildRobotBodies ()
        self.loadUrdfInGUI (RobotType, robotName)

    def loadEnvironmentModel (self, EnvType, envName, guiOnly = False):
        if not guiOnly:
            self.robot.loadEnvironmentModel (EnvType.packageName, EnvType.urdfName,
                EnvType.urdfSuffix, EnvType.srdfSuffix, envName + "/")
        self.loadUrdfObjectsInGUI (EnvType, envName)
        self.computeObjectPosition ()

    def loadObjectModel (self, RobotType, robotName, guiOnly = False):
        if not guiOnly:
            self.robot.insertObjectModel (robotName, RobotType.rootJointType,
                                    RobotType.packageName, RobotType.urdfName,
                                    RobotType.urdfSuffix, RobotType.srdfSuffix)
        self.buildRobotBodies ()
        self.loadUrdfInGUI (RobotType, robotName)
        self.computeObjectPosition ()

    def buildCompositeRobot (self, robotNames):
        self.robot.buildCompositeRobot (robotNames)
        self.buildRobotBodies ()

    def loadUrdfInGUI (self, RobotType, robotName):
        if hasattr (RobotType, 'meshPackageName'):
            meshPackageName = RobotType.meshPackageName
        else:
            meshPackageName = RobotType.packageName
        # Load robot in viewer
        rospack = rospkg.RosPack()
        packagePath = rospack.get_path (RobotType.packageName)
        meshPackagePath = rospack.get_path (meshPackageName)
        dataRootDir = os.path.dirname (meshPackagePath) + "/"
        packagePath += '/urdf/' + RobotType.urdfName + RobotType.urdfSuffix + \
            '.urdf'
        self.client.gui.addURDF (robotName, packagePath, dataRootDir)
        self.client.gui.addToGroup (robotName, self.sceneName)

    def loadUrdfObjectsInGUI (self, RobotType, robotName):
        if hasattr (RobotType, 'meshPackageName'):
            meshPackageName = RobotType.meshPackageName
        else:
            meshPackageName = RobotType.packageName
        rospack = rospkg.RosPack()
        packagePath = rospack.get_path (RobotType.packageName)
        meshPackagePath = rospack.get_path (meshPackageName)
        dataRootDir = os.path.dirname (meshPackagePath) + "/"
        packagePath += '/urdf/' + RobotType.urdfName + RobotType.urdfSuffix + \
            '.urdf'
        self.client.gui.addUrdfObjects (robotName, packagePath, meshPackagePath,
                                        True)
        self.client.gui.addToGroup (robotName, self.sceneName)
