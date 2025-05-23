import numpy
from math import pi
from gepetto.corbaserver import Client
from gepetto import Error as GepettoError
from pinocchio import forwardKinematics, updateGeometryPlacements
from eigenpy import Quaternion
from pyhpp.manipulation import urdf

class Viewer:


    def createWindowAndScene(self, name):
        self.windowName = "scene_hpp_" + name
        try:
            self.windowId = self.client.gui.getWindowID(self.windowName)
        except GepettoError:
            self.windowId = self.client.gui.createWindow(self.windowName)
        self.sceneName = self.windowName

    def addURDFToScene(self, frameIndex, prefix, rootType, urdfFilename, srdfFilename, pose):
        urdf.loadModel(self.robot, frameIndex, prefix, rootType, urdfFilename, srdfFilename, pose)
        self.client.gui.addURDF(prefix, urdfFilename)
        self.client.gui.addToGroup(prefix, self.sceneName)

    def __init__(self, name, robot):
        self.robot = robot
        self.client = Client()
        self.createWindowAndScene(name)

    def applyConfiguration(self, q):
        model = self.robot.model()
        gmodel = self.robot.geomModel()
        data = model.createData()
        gData = gmodel.createData()
        objects = [gmodel.geometryObjects[i].name for i in range(gmodel.ngeoms)]
        forwardKinematics(model, data, q)
        updateGeometryPlacements(model, data, gmodel, gData)
        for i, o in enumerate(objects):
            pose = gData.oMg[i]
            pose1 = list(pose.translation) + list(Quaternion(pose.rotation).coeffs())
            self.client.gui.applyConfiguration(o, pose1)
        self.client.gui.refresh()

