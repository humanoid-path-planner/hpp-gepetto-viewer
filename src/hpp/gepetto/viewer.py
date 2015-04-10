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

    def createWindowAndScene (self, viewerClient, name):
        self.windowName = "window_" + name
        self.windowId = viewerClient.gui.createWindow (self.windowName)
        self.sceneName = "%i_scene_%s" % (self.windowId, name)
        viewerClient.gui.createScene (self.sceneName)
        if not viewerClient.gui.addSceneToWindow (self.sceneName,
                                                  self.windowId):
            raise RuntimeError ('Failed to add scene "%s" to window %i ("%s")'%
                                (self.sceneName, self.windowId, self.windowName))


    def buildRobotBodies (self):
        self.robotBodies = list ()
        # build list of pairs (robotName, objectName)
        for j in self.robot.getAllJointNames ():
            self.robotBodies.extend (map (lambda n:
                                              (j, self.displayName + "/", n),
                                          [self.robot.getLinkName (j),]))

    ## Add a landmark
    # \sa gepetto::corbaserver::GraphicalInterface::addLandmark
    def addLandmark (self, linkname, size):
        return self.client.gui.addLandmark (linkname, size)

		##Display the roadmap created by problem.solve()
		# \param colorNode : the color of the sphere for the nodes
		# \param radiusSphere : the radius of the node
		# \param sizeAxis : size of axes (proportionnaly to the radius of the sphere) 0 = only sphere
		# \param colorEdge : the color of the edges
    def displayRoadmap (self,colorNode,radiusSphere,sizeAxis,colorEdge):
      ps = self.problemSolver
      problem = self.problemSolver.client.problem
      gui = self.client.gui
      for i in range(0,ps.numberNodes()) :	
        name = "R_node%d" % i
        if not gui.addXYZaxis(name,colorNode,radiusSphere,sizeAxis):
          return False
        gui.addToGroup(name,self.sceneName)
        gui.applyConfiguration(name,ps.node(i)[0:7])
      for i in range(0,ps.numberEdges()) : 
        if i%2 == 0 :
          name = "R_edge%d" % i
          gui.addLine(name,ps.edge(i)[0][0:3],ps.edge(i)[1][0:3],colorEdge)
          gui.addToGroup(name,self.sceneName)
      gui.refresh()
      return True

	

		##build the roadmap and diplay it during construction
		# (delete existing roadmap if problem already solved )
		# \param colorNode : the color of the sphere for the nodes
		# \param radiusSphere : the radius of the node
		# \param sizeAxis : size of axes (proportionnaly to the radius of the sphere) 0 = only sphere
		# \param colorEdge : the color of the edges
    def solveAndDisplay (self,colorNode,radiusSphere,sizeAxis,colorEdge):
      import time
      ps = self.problemSolver
      problem = self.problemSolver.client.problem
      gui = self.client.gui
      if ps.numberNodes() > 0 : 
        ps.clearRoadmap()
      tStart = time.time()
      problem.prepareSolveStepByStep()
      beginEdge = 0
      beginNode = 0
      while not problem.executeOneStep():
        for i in range(beginNode,ps.numberNodes()) :	
          name = "R_node%d" % i
          gui.addXYZaxis(name,colorNode,radiusSphere,sizeAxis)
          gui.addToGroup(name,self.sceneName)
          gui.applyConfiguration(name,ps.node(i)[0:7])
        for i in range(beginEdge,ps.numberEdges()) : 
          if i%2 == 0:
            name = "R_edge%d" % i
            gui.addLine(name,ps.edge(i)[0][0:3],ps.edge(i)[1][0:3],colorEdge)
            gui.addToGroup(name,self.sceneName)
        if beginNode < (ps.numberNodes()):
          gui.refresh()
        beginNode = ps.numberNodes() 
        beginEdge = ps.numberEdges()
      problem.finishSolveStepByStep()
			#display new edge (node ?) added by finish()
      for i in range(beginNode,ps.numberNodes()) :	
        name = "R_node%d" % i
        gui.addXYZaxis(name,colorNode,radiusSphere,sizeAxis)
        gui.addToGroup(name,self.sceneName)
        gui.applyConfiguration(name,ps.node(i)[0:7])
      for i in range(beginEdge,ps.numberEdges()) : 
        name = "R_edge%d" % i
        gui.addLine(name,ps.edge(i)[0][0:3],ps.edge(i)[1][0:3],colorEdge)
        gui.addToGroup(name,self.sceneName)
      if beginNode < (ps.numberNodes()):
        gui.refresh()
      tStop = time.time()
      return tStop-tStart



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
        self.client.gui.addUrdfObjects (prefix, packagePath, meshPackagePath,
                                        True)
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

    ## Start a screen capture
    # \sa gepetto::corbaserver::GraphicalInterface::startCapture
    def startCapture (self, filename, extension):
        return self.client.gui.startCapture (self.windowId, filename, extension)

    ## Stop a screen capture
    # \sa gepetto::corbaserver::GraphicalInterface::stopCapture
    def stopCapture (self):
        return self.client.gui.stopCapture (self.windowId)
