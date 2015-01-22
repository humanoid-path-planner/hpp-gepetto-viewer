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
from hpp.gepetto import Viewer

rospack = rospkg.RosPack()

class ViewerFactory (object):
    def __init__ (self, problemSolver, viewerClient = None):
        self.guiRequest = list ()
        self.problemSolver = problemSolver
        self.robot = problemSolver.robot
        self.buildRobotBodies ()

    def buildRobotBodies (self):
        self.robotBodies = list ()
        # build list of pairs (robotName, objectName)
        for j in self.robot.getAllJointNames ():
            self.robotBodies.extend (map (lambda n:
                                              (j, self.displayName + "/", n),
                                          self.robot.getJointInnerObjects (j)))

    def loadObstacleModel (self, package, filename, prefix,
                           meshPackageName = None, guiOnly = False):
        if not guiOnly:
            self.problemSolver.loadObstacleFromUrdf (package, filename, prefix+'/')
        l = locals ();
        l ['guiOnly'] = True
        self.guiRequest.append ((Viewer.loadObstacleModel, l));

    def computeObjectPosition (self):
        pass

    def publishRobots (self):
        pass

    def __call__ (self, args):
        self.robotConfig = args
        self.publishRobots ()

    def createRealClient (self, ViewerClass = Viewer, viewerClient = None):
        v = ViewerClass (self.problemSolver, viewerClient)
        for call in self.guiRequest:
            kwargs = call[1].copy ();
            s = kwargs.pop ('self')
            f = call[0];
            f (v, **kwargs)
        v.computeObjectPosition ()
        return v
