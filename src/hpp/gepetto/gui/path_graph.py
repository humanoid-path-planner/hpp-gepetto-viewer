from PythonQt import QtGui, QtCore, Qt

from hpp import Quaternion
from hpp.corbaserver import Client
from gepetto.corbaserver import Client as GuiClient
from gepetto.corbaserver.tools import Linear, Vector6
from gepetto.color import Color

import numpy as np
from math import pi, sqrt, cos, sin

pens = (
        # Qt.QPen (Qt.Qt.white),
        Qt.QPen (Qt.Qt.black),
        Qt.QPen (Qt.Qt.red),
        Qt.QPen (Qt.Qt.green),
        Qt.QPen (Qt.Qt.blue),
        Qt.QPen (Qt.Qt.cyan),
        Qt.QPen (Qt.Qt.magenta),
        Qt.QPen (Qt.Qt.yellow),
        Qt.QPen (Qt.Qt.gray),
        Qt.QPen (Qt.Qt.darkRed),
        Qt.QPen (Qt.Qt.darkGreen),
        Qt.QPen (Qt.Qt.darkBlue),
        Qt.QPen (Qt.Qt.darkCyan),
        Qt.QPen (Qt.Qt.darkMagenta),
        Qt.QPen (Qt.Qt.darkYellow),
        Qt.QPen (Qt.Qt.darkGray),
        Qt.QPen (Qt.Qt.lightGray),
        )

class VelGetter:
    def __init__(self, plugin, name):
        self.plugin = plugin
        self.name = str(name)
        self.vector6 = Vector6 (self.name)

    def getV(self):
        return self.plugin.client.robot.getJointVelocityInLocalFrame (self.name)

    def createNodes(self, color):
        self.group = str(self.plugin.jointgroupcreator.requestCreateJointGroup(self.name))
        self.vector6.color = color
        self.vector6.create(self.plugin.gui.gui)

    def removeNodes(self):
        self.vector6.remove(self.plugin.gui.gui)

    def apply(self):
        v = self.getV()
        self.vector6.set (self.plugin.gui.gui, v)

class ComGetter:
    def __init__(self, plugin, com):
        self.plugin = plugin
        self.name = str(com)
        self.linear = Linear (self.name)

    def getV(self):
        return self.plugin.client.robot.getVelocityPartialCom (self.name)

    def createNodes(self, color):
        self.group = str(self.plugin.comgroupcreator.requestCreateComGroup(self.name))
        self.linear.color = color
        self.linear.create (self.plugin.gui.gui)

    def removeNodes(self):
        self.linear.remove (self.plugin.gui.gui)

    def apply(self):
        v = self.getV()
        self.linear.set(self.plugin.gui.gui, v)

class Velocities:
    color = Color.red
    def __init__(self, plugin):
        self.plugin = plugin
        self.connected = False

        # For each arrow to plot, there are
        # - a node
        # - a getter
        self.getters = dict()

    def connect(self):
        if not self.connected:
            self.plugin.pathPlayer.setRobotVelocity(True)
            self.plugin.main.connect(QtCore.SIGNAL('applyCurrentConfiguration()'),
                    self.applyAll)
            self.connected = True

    def disconnect(self):
        if self.connected:
            self.plugin.main.disconnect(QtCore.SIGNAL('applyCurrentConfiguration()'),
                    self.applyAll)
            self.connected = False

    def add (self, getter):
        self.getters[getter.name] = getter
        getter.apply ()
        self.connect()

    def remove (self, node):
        getter = self.getters.pop(node)
        getter.removeNodes()
        if len(self.getters) == 0:
            self.disconnect()

    def toggleJoint (self, joint):
        if self.getters.has_key(joint): self.remove(joint)
        else:
            g = VelGetter(self.plugin, joint)
            g.createNodes (Velocities.color)
            self.add (g)

    def toggleCom (self, com):
        if self.getters.has_key(com): self.remove (com)
        else:
            g = ComGetter (self.plugin, com)
            g.createNodes (Velocities.color)
            self.add (g)

    def applyAll (self):
        for n, getV in self.getters.items():
            getV.apply()

class JointAction(Qt.QAction):
    def __init__ (self, action, joint, velocities, parent):
        super(JointAction, self).__init__ (action, parent)
        self.joint = joint
        self.velocities = velocities
        self.connect (QtCore.SIGNAL('triggered(bool)'), self.trigger)

    def trigger (self):
        self.velocities.toggleJoint (self.joint)

class DataQCP:
    def __init__ (self, plugin):
        self.plugin = plugin
        self.qcp = self.plugin.qcpWidget
        self.pb = plugin.progressBar
        self.timer = Qt.QTimer()
        self.timer.setSingleShot(True)

        self.pathLength = -1
        self.dl = -1
        self.pathId = -1

    def selectData (self, pid, dl, x, ys):
        if dl < 1e-8:
            return
        self.x = x
        self.ys = ys
        pl = self.plugin.client.problem.pathLength(pid)
        if   not self.pathLength == pl \
            or not self.pathId     == pid \
            or not self.dl         == dl:
            self.dataAreOld = True
            self.pathId = pid
            self.pathLength = pl
            self.dl = dl
            self.l = 0
            self.datas = list ()
        else:
            self.dataAreOld = False

    def acquireData (self):
        if self.dataAreOld:
            self.pb.reset()
            self._setNextCall (self._getNextData)
        else:
            self.pb.setValue(99)
            self._setNextCall (self._genPlot)
        self.timer.start(0)

    def _setNextCall (self, f):
        self.timer.disconnect(Qt.SIGNAL("timeout()"))
        self.timer.connect(Qt.SIGNAL("timeout()"), f)

    def _getNextData (self):
        d = [ self.l, ]
        try:
            q = self.plugin.client.problem.configAtParam (self.pathId, self.l)
        except:
            q = [np.nan] * self.plugin.client.robot.getConfigSize()
        d.extend (q)
        self.datas.append (d)
        self.l += self.dl
        self.pb.setValue (int(100 * self.l / self.pathLength))
        if self.l < self.pathLength:
            self._setNextCall (self._getNextData)
        else:
            self._setNextCall (self._genPlot)
        self.timer.start(0)

    def _genPlot (self):
        self.npdatas = np.matrix (self.datas)

        self.qcp.clearGraphs()
        for i, elt in enumerate(self.ys):
            graph = self.qcp.addGraph ()
            graph.setData (self.npdatas [:,self.x[1]], self.npdatas [:,elt[1]])
            graph.setName (elt[0])
            graph.setPen(pens[i])
        self.qcp.xAxis().setLabel(self.x[0])
        self.qcp.rescaleAxes()
        self.qcp.replot()
        # TODO Add a vertical bar at current param along the path.

        self.pb.setValue(100)
        return False

class Plugin (QtGui.QDockWidget):
    def __init__ (self, mainWindow, flags = None):
        if flags is None:
            super(Plugin, self).__init__ ("Path graph plugin", mainWindow)
        else:
            super(Plugin, self).__init__ ("Path graph plugin", mainWindow, flags)
        self.setObjectName("Path graph plugin")

        self.main = mainWindow
        self.hppPlugin = self.main.getFromSlot("getHppIIOPurl")
        self.pathPlayer = self.main.getFromSlot("getCurrentPath")
        self.jointgroupcreator = self.main.getFromSlot("requestCreateJointGroup")
        self.comgroupcreator = self.main.getFromSlot("requestCreateComGroup")
        self.velocities = Velocities(self)
        self.jointActions = dict()
        self.resetConnection()

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

        self.topWidget.addWidget (self.leftPane)

        self.rightPane = QtGui.QWidget (self)
        l = QtGui.QVBoxLayout ()
        self.makeRightPane (l)
        self.rightPane.setLayout (l)
        self.topWidget.addWidget (self.rightPane)

        self.data = DataQCP(self)

    def refreshPlot (self):
        pid = self.pathPlayer.getCurrentPath()
        if pid < 0: return
        dl = self.pathPlayer.lengthBetweenRefresh()
        idxX = self.xselect.currentIndex
        x = (self.xselect.itemText(idxX),
             int(self.xselect.itemData(idxX)) + 1)

        ys = list ()
        for elt in self.yselectcb:
            cb = elt[0]
            if cb.checked:
                ys.append ((str(cb.text), elt[1]+1))

        self.data.selectData(pid, dl, x, ys)
        self.data.acquireData()

    def refreshJointList (self):
        jointNames = self.client.robot.getJointNames ()
        # Left pane
        saLayout = QtGui.QVBoxLayout ()

        self.yselectcb = list ()
        rank = 0
        for n in jointNames:
            size = self.client.robot.getJointConfigSize (n)
            if size == 1:
                cb = QtGui.QCheckBox (n)
                self.yselectcb.append ((cb, rank))
                saLayout.addWidget (cb)
            else:
                for i in xrange (size):
                    cb = QtGui.QCheckBox ("%s (%i)" % (n, i))
                    self.yselectcb.append ((cb, rank + i))
                    saLayout.addWidget (cb)
            rank = rank + size

        saContent = QtGui.QWidget (self)
        saContent.setLayout (saLayout)
        self.scrollArea.setWidget (saContent)

        # Right pane
        self.xselect.clear()
        # time index is 0 and is value is -1
        self.xselect.addItem ("time", -1)
        rank = 0
        for n in self.client.robot.getJointNames ():
            size = self.client.robot.getJointConfigSize (n)
            if size == 1:
                self.xselect.addItem (n, rank)
            else:
                for i in xrange (size):
                    self.xselect.addItem ("%s (%i)" % (n, i), rank + i)
            rank = rank + size

    def refreshInterface (self):
        self.refreshJointList()

    def makeLeftPane (self, layout):
        layout.addWidget (QtGui.QLabel ("Select Y data"))
        refresh = QtGui.QPushButton ("Refresh", self)
        refresh.connect (QtCore.SIGNAL("clicked()"), self.refreshPlot)
        layout.addWidget (refresh)

        self.scrollArea = QtGui.QScrollArea (self)
        layout.addWidget (self.scrollArea)

    def makeRightPane (self, layout):
        from PythonQt.QCustomPlot import QCustomPlot
        self.qcpWidget = QCustomPlot()
        self.qcpWidget.setAutoAddPlottableToLegend (True)
        self.qcpWidget.setInteraction (1, True) # iRangeDrap
        self.qcpWidget.setInteraction (2, True) # iRangeZoom
        self.qcpWidget.legend().setVisible(True)
        layout.addWidget (self.qcpWidget)
        self.qcpWidget.connect (Qt.SIGNAL("mouseDoubleClick(QMouseEvent*)"), self._mouseDoubleClick)

        self.xselect = QtGui.QComboBox(self)
        layout.addWidget (self.xselect)

        self.progressBar = QtGui.QProgressBar(self)
        self.progressBar.setRange(0,100)
        layout.addWidget (self.progressBar)

        # time index is 0 and is value is -1
        self.xselect.addItem ("time", -1)

    def _mouseDoubleClick (self, event):
        try:
            if self.data.x[1] > 0: return
        except:
            return
        try: # Qt 5
            x = event.localPos().x()
        except: # Qt 4
            x = event.posF().x()
        t = self.qcpWidget.xAxis().pixelToCoord (x)
        self.pathPlayer.setCurrentTime(t)

    def resetConnection(self):
        self.client = Client(url= str(self.hppPlugin.getHppIIOPurl()))
        self.gui = GuiClient()

    def getJointActions(self, name):
        if not self.jointActions.has_key(name):
            self.jointActions[name] = (JointAction ("Show/Hide joint &velocity", name, self.velocities, self),)
        return self.jointActions[name]
