#!/usr/bin/env python

# BSD 2-Clause License

# Copyright (c) 2014-2025, CNRS - INRIA

# Author: Florent Lamiraux

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

from hpp.gepetto import Viewer as Parent
from hpp.gepetto.viewer import _urdfSrdfFilenames


class Viewer(Parent):
    def __init__(
        self,
        problemSolver,
        viewerClient=None,
        ghost=False,
        collisionURDF=False,
        *args,
        **kwargs,
    ):
        """
        Simultaneous control to hpp-manipulation-server and gepetto-viewer-server.
        """
        self.compositeRobotName = problemSolver.robot.client.basic.robot.getRobotName()
        Parent.__init__(
            self, problemSolver, viewerClient, ghost, collisionURDF, *args, **kwargs
        )

    def _initDisplay(self):
        if not self.client.gui.nodeExists(self.compositeRobotName):
            self.client.gui.createGroup(self.compositeRobotName)
            self.client.gui.addToGroup(self.compositeRobotName, self.sceneName)
        urdfFilename, srdfFilename = self.robot.urdfSrdfFilenames()
        name = self.compositeRobotName + "/" + self.robot.robotNames[0]
        self.client.gui.addURDF(name, urdfFilename)
        # Remove lighting from meshes
        self._removeLightSources(self.client.gui.getGroupNodeList(name))
        if self.collisionURDF:
            self.toggleVisual(False)
        # self.client.gui.addToGroup (name, self.compositeRobotName)

    def loadRobotModel(
        self, RobotType, robotName, guiOnly=False, collisionURDF=False, frame=None
    ):
        if not guiOnly:
            urdfFilename, srdfFilename = _urdfSrdfFilenames(RobotType)
            if frame is None:
                self.robot.insertRobotModel(
                    robotName, RobotType.rootJointType, urdfFilename, srdfFilename
                )
            else:
                self.robot.insertRobotModelOnFrame(
                    robotName,
                    frame,
                    RobotType.rootJointType,
                    urdfFilename,
                    srdfFilename,
                )
        self.buildRobotBodies()
        self.loadUrdfInGUI(RobotType, robotName)

    def loadRobotModelFromString(
        self,
        robotName,
        rootJointType,
        urdfString,
        srdfString="<robot/>",
        guiOnly=False,
        frame="universe",
    ):
        if not guiOnly:
            self.robot.insertRobotModelFromString(
                robotName, rootJointType, urdfString, srdfString, frame=frame
            )
        nodeName = self.compositeRobotName + "/" + robotName
        if self.collisionURDF:
            self.client.gui.addUrdfCollision(nodeName, urdfString)
        else:
            self.client.gui.addURDF(nodeName, urdfString)
        # Remove lighting from meshes
        self._removeLightSources(self.client.gui.getGroupNodeList(nodeName))

    def loadHumanoidModel(self, RobotType, robotName, guiOnly=False):
        if not guiOnly:
            urdfFilename, srdfFilename = _urdfSrdfFilenames(RobotType)
            self.robot.loadHumanoidModel(
                robotName, RobotType.rootJointType, urdfFilename, srdfFilename
            )
        self.buildRobotBodies()
        self.loadUrdfInGUI(RobotType, robotName)

    def loadEnvironmentModel(self, EnvType, envName, guiOnly=False):
        if not guiOnly:
            urdfFilename, srdfFilename = _urdfSrdfFilenames(EnvType)
            self.robot.loadEnvironmentModel(urdfFilename, srdfFilename, envName + "/")
        self.loadUrdfObjectsInGUI(EnvType, envName)
        self.computeObjectPosition()

    def loadEnvironmentModelFromString(self, EnvType, envName, guiOnly=False):
        if not guiOnly:
            urdfFilename, srdfFilename = _urdfSrdfFilenames(EnvType)
            self.robot.loadEnvironmentModelFromString(
                urdfFilename, srdfFilename, envName + "/"
            )
        self.loadUrdfObjectsInGUI(EnvType, envName)
        self.computeObjectPosition()

    def loadObjectModel(self, RobotType, robotName, guiOnly=False):
        if not guiOnly:
            urdfFilename, srdfFilename = _urdfSrdfFilenames(RobotType)
            self.robot.insertRobotModel(
                robotName, RobotType.rootJointType, urdfFilename, srdfFilename
            )
        self.buildRobotBodies()
        base = "collision_" if self.collisionURDF else ""
        self.loadUrdfInGUI(RobotType, base + robotName)
        self.computeObjectPosition()

    def buildCompositeRobot(self, robotNames):
        self.robot.buildCompositeRobot(robotNames)
        self.buildRobotBodies()

    def loadUrdfInGUI(self, RobotType, robotName):
        # Load robot in viewer
        urdfFilename, srdfFilename = _urdfSrdfFilenames(RobotType)
        nodeName = self.compositeRobotName + "/" + robotName
        if self.collisionURDF:
            self.client.gui.addUrdfCollision(nodeName, urdfFilename)
        else:
            self.client.gui.addURDF(nodeName, urdfFilename)
        # Remove lighting from meshes
        self._removeLightSources(self.client.gui.getGroupNodeList(nodeName))

    def loadUrdfObjectsInGUI(self, RobotType, robotName):
        urdfFilename, srdfFilename = _urdfSrdfFilenames(RobotType)
        self.client.gui.addUrdfObjects(robotName, urdfFilename, not self.collisionURDF)
        self._removeLightSources(self.client.gui.getGroupNodeList(robotName))
        self.client.gui.addToGroup(robotName, self.sceneName)
