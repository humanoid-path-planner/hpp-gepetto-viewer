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
import warnings
from hpp.gepetto import Viewer
from hpp.gepetto.viewer import _urdfPath, _srdfPath, _urdfSrdfFilenames, _GhostViewerClient


## Viewer factory
#
#  Store commands to be sent to \c gepetto-viewer-server, create
#  clients on demand and send stored commands.
class ViewerFactory (object):
    ## Whether light sources in collada files should be removed at loading.
    removeLightSources = True

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

    ## Load obstacles from a urdf file
    #
    #  \param filename name of the urdf file, may contain "package://"
    #  \param prefix prefix added to object names in case the same file
    #         is loaded several times,
    #  \param guiOnly whether to control only gepetto-viewer-server
    def loadObstacleModel (self, filename, prefix, guiOnly = False):
        l = locals ().copy ()
        if not guiOnly:
            self.problemSolver.loadObstacleFromUrdf (filename, prefix+'/')
        l ['guiOnly'] = True
        self.guiRequest.append ((Viewer.loadObstacleModel, l));

    ## Load polyhedron from a 3D mesh file
    #
    #  \param filename name of the 3D mesh file, may contain "package://"
    #  \param name name of the object,
    #  \param guiOnly whether to control only gepetto-viewer-server
    def loadPolyhedronObstacleModel (self, name, filename, guiOnly = False):
        l = locals ().copy ()
        if not guiOnly:
            self.problemSolver.hppcorba.obstacle.loadPolyhedron(name, filename)
        l ['guiOnly'] = True
        self.guiRequest.append ((Viewer.loadPolyhedronObstacleModel, l));

    ## Move Obstacle
    #
    #  \param name Name of the object
    #  \param position Position of the object as a 7-d vector
    #         (translation-quaternion)
    #  \param guiOnly whether to control only gepetto-viewer-server
    def moveObstacle (self, name, position, guiOnly = False):
        if not guiOnly:
            self.problemSolver.moveObstacle (name, position)
        self.computeObjectPosition ()

    def computeObjectPosition (self):
        l = locals ();
        self.guiRequest.append ((Viewer.computeObjectPosition, l));

    def publishRobots (self):
        pass

    def displayRoadmap (self, nameRoadmap, radiusSphere=0.01, sizeAxis=0.03,
                        colorNode=[1.0,1.0,1.0,1.0],
                        colorEdge=[0.85,0.75,0.15,0.7], joint=None):
        l = locals ()
        self.guiRequest.append ((Viewer.displayRoadmap, l))

    def __call__ (self, args):
        l = locals ();
        self.guiRequest.append ((Viewer.__call__, l));



    ## Create a client to \c gepetto-viewer-server and send stored commands
    #
    # The arguments of Viewer.__init__ can be passed through kwargs
    def createViewer (self, ViewerClass = Viewer, viewerClient = None, ghost = False, host = None, *args, **kwargs):
        if host is not None and viewerClient is None:
            from gepetto.corbaserver import Client as GuiClient
            try:
                viewerClient = GuiClient (host = host)
            except Exception as e:
               if ghost:
                   print("Failed to connect to the viewer.")
                   print("Check whether gepetto-gui is properly started.")
                   viewerClient = _GhostViewerClient()
               else:
                   raise e
        v = ViewerClass (self.problemSolver, viewerClient, ghost = ghost, *args, **kwargs)
        v.removeLightSources = self.removeLightSources
        for call in self.guiRequest:
            kwargs = call[1].copy ();
            s = kwargs.pop ('self')
            f = call[0];
            f (v, **kwargs)
        return v
