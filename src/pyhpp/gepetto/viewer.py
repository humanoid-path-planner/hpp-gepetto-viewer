import time

import numpy as np
from eigenpy import Quaternion
from gepetto import Error as GepettoError
from gepetto.corbaserver import Client
from pinocchio import SE3, forwardKinematics, updateFramePlacements

from pyhpp.pinocchio import urdf
import pinocchio.pinocchio_pywrap_default
BODY = pinocchio.pinocchio_pywrap_default.FrameType(8)

class Viewer:
    def displayPath(self, path, speed=0.01, nbPoints=300):
        for i in range(path.numberPaths()):
            for t in np.linspace(0, path.pathAtRank(i).length(), nbPoints):
                q, success = path.pathAtRank(i).eval(t)
                if success:
                    self.applyConfiguration(q)
                    time.sleep(speed)

    def createWindowAndScene(self, name):
        self.windowName = "scene_hpp_" + name
        try:
            self.windowId = self.client.gui.getWindowID(self.windowName)
        except GepettoError:
            self.windowId = self.client.gui.createWindow(self.windowName)
        self.sceneName = self.windowName

    def addURDFObstacleToScene(self, filename, prefix):
        urdf.loadModel(self.robot, 0, prefix, "anchor", filename, "", SE3.Identity())
        self.client.gui.addUrdfObjects(self.sceneName + "/" + prefix, filename, True)
        self._recomputeObjects()
        #self.client.gui.addToGroup(prefix, self.sceneName)

    def addURDFToScene(
        self, frameIndex, prefix, rootType, urdfFilename, srdfFilename, pose
    ):
        urdf.loadModel(
            self.robot, frameIndex, prefix, rootType, urdfFilename, srdfFilename, pose
        )
        self.client.gui.addURDF(self.sceneName + "/" + prefix, urdfFilename)
        self._recomputeObjects()
        #self.client.gui.addToGroup(prefix, self.sceneName)

    def __init__(self, name, robot):
        self.robot = robot
        self.client = Client()
        self.createWindowAndScene(name)
        self.hppObjectNames = list()
        self.guiObjectNames = list()
        self.objectIndices = list()

    def applyConfiguration(self, q):
        model = self.robot.model()
        data = model.createData()
        forwardKinematics(model, data, q)
        updateFramePlacements(model, data)
        for i, hppObj, guiObj in zip(self.objectIndices, self.hppObjectNames, self.guiObjectNames):
            pose = data.oMf[i]
            pose1 = list(pose.translation) + list(Quaternion(pose.rotation).coeffs())
            self.client.gui.applyConfiguration(guiObj, pose1)
        self.client.gui.refresh()

    def _recomputeObjects(self):
        self.hppObjectNames = list()
        self.guiObjectNames = list()
        model = self.robot.model()
        for f in model.frames:
            if model.existFrame(f.name, BODY):
                self.hppObjectNames.append(f.name)
                self.guiObjectNames.append(self.sceneName + "/" + f.name)
                self.objectIndices.append(model.getFrameId(f.name))
