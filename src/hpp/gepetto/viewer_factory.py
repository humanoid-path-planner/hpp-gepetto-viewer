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

import os.path
import rospkg
from hpp.gepetto import Viewer

rospack = rospkg.RosPack()

## Viewer factory
#
#  Store commands to be sent to \c gepetto-viewer-server, create
#  clients on demand and send stored commands.
class ViewerFactory (object):
    ## Constructor
    #  \param problemSolver instance of
    #         hpp.corbaserver.problem_solver.ProblemSolver,
    def __init__ (self, problemSolver):
        self.guiRequest = list ()
        self.problemSolver = problemSolver
        self.robot = problemSolver.robot

    def buildRobotBodies (self):
        l = locals ();
        self.guiRequest.append ((Viewer.buildRobotBodies, l));

    def addLandmark (self, linkname, size):
        l = locals ();
        self.guiRequest.append ((Viewer.addLandmark, l));

    def loadObstacleModel (self, package, filename, prefix,
                           meshPackageName = None, guiOnly = False):
        if not guiOnly:
            self.problemSolver.loadObstacleFromUrdf (package, filename, prefix+'/')
        l = locals ();
        l ['guiOnly'] = True
        self.guiRequest.append ((Viewer.loadObstacleModel, l));

    def computeObjectPosition (self):
        l = locals ();
        self.guiRequest.append ((Viewer.computeObjectPosition, l));

    def publishRobots (self):
        pass

    def __call__ (self, args):
        l = locals ();
        self.guiRequest.append ((Viewer.__call__, l));

    ## Create a client to \c gepetto-viewer-server and send stored commands
    #
    def createRealClient (self, ViewerClass = Viewer, viewerClient = None):
        v = ViewerClass (self.problemSolver, viewerClient)
        for call in self.guiRequest:
            kwargs = call[1].copy ();
            s = kwargs.pop ('self')
            f = call[0];
            f (v, **kwargs)
        return v
