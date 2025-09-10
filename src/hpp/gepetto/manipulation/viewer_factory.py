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

from hpp.gepetto import ViewerFactory as Parent
from hpp.gepetto.manipulation import Viewer
from hpp.gepetto.viewer import _urdfSrdfFilenames


class ViewerFactory(Parent):
    """
    Viewer factory for manipulation.Viewer

    Store commands to be sent to \\c gepetto-viewer-server, create
    clients on demand and send stored commands.
    """

    def __init__(self, problemSolver):
        """
        Constructor
        \\param problemSolver instance of class
           manipulation.problem_solver.ProblemSolver
        """
        Parent.__init__(self, problemSolver)

    def buildRobotBodies(self):
        loc = locals()
        self.guiRequest.append((Viewer.buildRobotBodies, loc))

    def loadRobotModel(self, RobotType, robotName, guiOnly=False, frame=None):
        loc = locals().copy()
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
        loc["guiOnly"] = True
        self.guiRequest.append((Viewer.loadRobotModel, loc))

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
        loc = locals().copy()
        loc["guiOnly"] = True
        self.guiRequest.append((Viewer.loadRobotModelFromString, loc))

    def loadHumanoidModel(self, RobotType, robotName, guiOnly=False):
        loc = locals().copy()
        if not guiOnly:
            urdfFilename, srdfFilename = _urdfSrdfFilenames(RobotType)
            self.robot.loadHumanoidModel(
                robotName, RobotType.rootJointType, urdfFilename, srdfFilename
            )
        loc["guiOnly"] = True
        self.guiRequest.append((Viewer.loadHumanoidModel, loc))

    def loadEnvironmentModel(self, EnvType, envName, guiOnly=False):
        loc = locals().copy()
        if not guiOnly:
            urdfFilename, srdfFilename = _urdfSrdfFilenames(EnvType)
            self.robot.loadEnvironmentModel(urdfFilename, srdfFilename, envName + "/")
        loc["guiOnly"] = True
        self.guiRequest.append((Viewer.loadEnvironmentModel, loc))

    def loadEnvironmentModelFromString(self, EnvType, envName, guiOnly=False):
        loc = locals().copy()
        if not guiOnly:
            urdfFilename, srdfFilename = _urdfSrdfFilenames(EnvType)
            self.robot.loadEnvironmentModelFromString(
                urdfFilename, srdfFilename, envName + "/"
            )
        loc["guiOnly"] = True
        self.guiRequest.append((Viewer.loadEnvironmentModel, loc))

    def loadObjectModel(self, RobotType, robotName, guiOnly=False):
        loc = locals().copy()
        if not guiOnly:
            urdfFilename, srdfFilename = _urdfSrdfFilenames(RobotType)
            self.robot.insertRobotModel(
                robotName, RobotType.rootJointType, urdfFilename, srdfFilename
            )
        loc["guiOnly"] = True
        self.guiRequest.append((Viewer.loadObjectModel, loc))

    def buildCompositeRobot(self, robotNames):
        self.robot.buildCompositeRobot(robotNames)
        self.buildRobotBodies()

    def loadUrdfInGUI(self, RobotType, robotName):
        self.guiRequest.append((Viewer.loadUrdfInGUI, locals()))

    def loadUrdfObjectsInGUI(self, RobotType, robotName):
        self.guiRequest.append((Viewer.loadUrdfObjectsInGUI, locals()))

    def createViewer(self, ViewerClass=Viewer, *args, **kwargs):
        """
        Create a client to \\c gepetto-viewer-server and send stored commands
        """
        return Parent.createViewer(self, ViewerClass, *args, **kwargs)
