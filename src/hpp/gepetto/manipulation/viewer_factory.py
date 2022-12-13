#!/usr/bin/env python

# Copyright (c) 2015 CNRS
# Author: Florent Lamiraux, Joseph Mirabel
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
        self, robotName, rootJointType, urdfString, srdfString="<robot/>", guiOnly=False, frame="universe"
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
