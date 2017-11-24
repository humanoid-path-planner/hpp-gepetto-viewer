from gepetto.gui.matplotlibwidget import MatplotlibWidget
from PythonQt import QtGui, QtCore, Qt

from hpp import Quaternion
from hpp.corbaserver import Client
from gepetto.corbaserver import Client as GuiClient
from gepetto.color import Color

import numpy as np

class VelGetter:
    def __init__(self, plugin, name):
        self.plugin = plugin
        self.name = str(name)
    def __call__(self):
        return self.plugin.client.robot.getJointVelocityInLocalFrame (self.name)
class ComGetter:
    def __init__(self, plugin, com):
        self.plugin = plugin
        self.com = str(com)

    def __call__(self):
        return self.plugin.client.robot.getVelocityPartialCom (self.com)

class Velocities:
    def __init__(self, plugin):
        self.plugin = plugin
        self.radius = 0.005
        self.suffix = "vel"
        self.colorLin = Color.lightRed
        self.suffixLin = "linear"
        self.colorAng = Color.lightBlue
        self.suffixAng = "angular"

        self.jointgroupcreator = self.plugin.main.getFromSlot("requestCreateJointGroup")
        self.comgroupcreator = self.plugin.main.getFromSlot("requestCreateComGroup")

        # For each arrow to plot, there are
        # - a node
        # - a getter
        self.getters = dict()

    def add (self, node, getter, color):
        self.getters[node] = getter
        if not isinstance(node, tuple):
            node = (node,)
            color = (color,)
        for n, c in zip(node, color):
            self.plugin.gui.gui.addArrow (n, self.radius, 1, c)
        # node will be automaticcally added to the good group
        # because of its name
        # self.plugin.gui.gui.addToGroup (node, group)

    def remove (self, node):
        self.getters.pop(node)
        if not isinstance(node, tuple): node = (node,)
        for n in node:
            self.plugin.gui.gui.deleteNode (n, True)

    def toggleJoint (self, joint):
        group = str(self.jointgroupcreator.requestCreateJointGroup(joint))
        nodes = (group + '/' + self.suffixLin, group + '/' + self.suffixAng)
        if self.getters.has_key(nodes): self.remove (nodes)
        else:
            g = VelGetter (self.plugin, joint)
            self.add (nodes, g, (self.colorLin, self.colorAng))
            self.apply (nodes, g)

    def toggleCom (self, com):
        group = str(self.comgroupcreator.requestCreateComGroup(com))
        node = group + '/' + self.suffixLin
        if self.getters.has_key(node): self.remove (node)
        else:
            g = ComGetter (self.plugin, com)
            self.add (node, g, self.colorLin)
            self.apply (node, g)

    def applyAll (self):
        for n, getV in self.getters.items():
            self.apply (n, getV)

    def apply (self, n, getV):
        from math import pi
        v = getV()
        if len(v) == 6: # linear and angular velocities
            assert isinstance(n, (tuple, list)) and len(n) == 2
            self.setVelocity (n[0], v[:3])
            self.setVelocity (n[1], np.array(v[3:]) / pi)
        elif len(v) == 3: # linear and angular velocities
            self.setVelocity (n, v)

    def setVelocity (self, n, v):
        from math import sqrt
        norm_v = np.linalg.norm(v)
        if norm_v == 0:
            self.plugin.gui.gui.resizeArrow (n, self.radius, norm_v)
            return
        # x = (1, 0, 0)
        u = v / norm_v
        if u[0] < -1 + 1e-6: # Nearly opposite vectors
            m = np.Array ( ( (1, 0, 0), u ) )
            U, S, V = np.linalg.svd (m)
            c = max (u[0], -1)
            w2 = (1 + c) / 2
            s = sqrt(1 - w2)
            t = (0,0,0, V[0,2]/s, V[1,2]/s, V[2,2]/s, w2)
        else:
            s = sqrt(2 * (1 + u[0]))
            # axis = (0, -u[2]/s, u[1]/s) # x ^ u
            t = (0,0,0, 0, -u[2]/s, u[1]/s, s/2)

        self.plugin.gui.gui.applyConfiguration (n, t)
        self.plugin.gui.gui.resizeArrow (n, self.radius, norm_v)

class JointAction(Qt.QAction):
    def __init__ (self, action, joint, velocities, parent):
        super(JointAction, self).__init__ (action, parent)
        self.joint = joint
        self.velocities = velocities
        self.connect (QtCore.SIGNAL('triggered(bool)'), self.trigger)

    def trigger (self):
        self.velocities.toggleJoint (self.joint)

class Plugin (QtGui.QDockWidget):
    def __init__ (self, mainWindow, flags = None):
        if flags is None:
            super(Plugin, self).__init__ ("Path graph plugin", mainWindow)
        else:
            super(Plugin, self).__init__ ("Path graph plugin", mainWindow, flags)
        self.setObjectName("Path graph plugin")

        self.main = mainWindow
        self.hppPlugin = self.main.getFromSlot("getHppIIOPurl")
        self.velocities = Velocities(self)
        self.jointActions = dict()
        self.resetConnection()

        self.main.connect(QtCore.SIGNAL('applyCurrentConfiguration()'),
                self.applyCurrentConfig)

        obj = self.main.getFromSlot("setRobotVelocity")
        obj.setRobotVelocity(True)

        # This avoids having a widget bigger than what it needs. It avoids having
        # a big dock widget and a small osg widget when creating the main osg widget.
        p = Qt.QSizePolicy.Ignored
        self.topWidget = QtGui.QSplitter(Qt.Qt.Horizontal, self)
        self.topWidget.setSizePolicy(Qt.QSizePolicy(p,p))
        self.setWidget (self.topWidget)

        self.leftPane = QtGui.QWidget (self)
        l = QtGui.QVBoxLayout ()
        self.makeLeftPane (l)
        self.leftPane.setLayout (l)

        self.leftPaneSA = QtGui.QScrollArea (self)
        self.leftPaneSA.setWidget (self.leftPane)
        # self.leftPane.setVerticalScrollBarPolicy(Qt.Qt.ScrollBarAsNeeded)
        self.topWidget.addWidget (self.leftPaneSA)

        self.mplWidget = MatplotlibWidget(self)
        self.topWidget.addWidget (self.mplWidget)

    def refreshPlot (self):
        # Plot something
        print "refreshPlot"
        x = np.linspace (0, 10, num=100)
        y = np.sin(x)
        self.mplWidget.figure.gca().plot (x, y)

    def makeLeftPane (self, layout):
        layout.addWidget (QtGui.QLabel ("Select Y data"))
        refresh = QtGui.QPushButton ("Refresh", self)
        refresh.connect (QtCore.SIGNAL("clicked()"), self.refreshPlot)
        layout.addWidget (refresh)

        self.yselectcb = list ()
        rank = 0
        for n in self.client.robot.getJointNames ():
            size = self.client.robot.getJointConfigSize (n)
            if size == 1:
                cb = QtGui.QCheckBox (n)
                self.yselectcb.append ((cb, rank))
                layout.addWidget (cb)
            else:
                for i in xrange (size):
                    cb = QtGui.QCheckBox ("%s (%i)" % (n, i))
                    self.yselectcb.append ((cb, rank + i))
                    layout.addWidget (cb)
            rank = rank + size

    ### If present, this function is called when a new OSG Widget is created.
    def osgWidget(self, osgWindow):
        pass

    def resetConnection(self):
        self.client = Client(url= str(self.hppPlugin.getHppIIOPurl()))
        self.gui = GuiClient()

    def applyCurrentConfig(self):
        self.velocities.applyAll()

    def getJointActions(self, name):
        print "getJointActions", name
        if not self.jointActions.has_key(name):
            self.jointActions[name] = (JointAction ("Show/Hide joint &velocity", name, self.velocities, self),)
        return self.jointActions[name]
