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

rospack = rospkg.RosPack()

class Viewer (object):
    @staticmethod
    def createWindowAndScene (viewerClient, name):
        Viewer.windowName = "window_" + name
        Viewer.sceneName = "scene_" + name
        if not viewerClient.gui.createWindow (Viewer.windowName):
            raise RuntimeError ('Failed to create window "%s"'%
                                Viewer.windowName)
        viewerClient.gui.createSceneWithFloor ("scene_" + name)
        if not viewerClient.gui.addSceneToWindow (Viewer.sceneName,
                                                  Viewer.windowName):
            raise RuntimeError ('Failed to add scene "%s" to window "%s"'%
                                (Viewer.sceneName, Viewer.windowName))

    def __init__ (self, robot, viewerClient):
        self.robotName = robot.name
        if hasattr (robot, 'meshPackageName'):
            meshPackageName = robot.meshPackageName
        else:
            meshPackageName = robot.packageName
        self.client = viewerClient
        self.robot = robot
        self.objects = list ()
        for j in robot.getAllJointNames ():
            self.objects.extend (robot.getJointInnerObjects (j))
        # Load robot in viewer
        rospack = rospkg.RosPack()
        packagePath = rospack.get_path (robot.packageName)
        meshPackagePath = rospack.get_path (meshPackageName)
        dataRootDir = os.path.dirname (meshPackagePath) + "/"
        packagePath += '/urdf/' + robot.urdfName + robot.urdfSuffix + '.urdf'
        self.client.gui.addURDF (self.robotName, packagePath, dataRootDir)
        self.client.gui.addToGroup (self.robotName, self.sceneName)

    def publishRobots (self):
        self.robot.setCurrentConfig (self.robotConfig)
        for o in self.objects:
            pos = self.robot.getObjectPosition (o)
            transform = tuple (pos.translation) + pos.quaternion.toTuple ()
            objectName = self.robotName + "/" + o
            self.client.gui.applyConfiguration (objectName, transform)
        self.client.gui.refresh ()

    def __call__ (self, args):
        self.robotConfig = args
        self.publishRobots ()
