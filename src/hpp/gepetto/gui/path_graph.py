from gepetto.gui.matplotlibwidget import MatplotlibWidget
from PythonQt import QtGui, QtCore, Qt

from hpp import Quaternion
from hpp.corbaserver import Client
from gepetto.corbaserver import Client as GuiClient
from gepetto.color import Color

import numpy as np
from math import pi, sqrt, cos, sin

def _pointsTorus(R, r, nR, nr):
    pts = []
    twopi = 2 * pi
    for j in range(nR):
        for i in range(nr + 1):
            s = i % nr
            phi = s* twopi /nr
            for k in (1, 0):
                t = (j + k) % nR

                theta = t* twopi /nR
                y =  (R+r*cos(phi))*cos(theta);
                z = -(R+r*cos(phi))*sin(theta);
                x = r * sin(phi);
                pts.append ((x, y, z))
    return pts

def _pointsCone(r, L, nr, R):
    pts = []
    twopi = 2 * pi
    for j in range(nr+1):
        t = j % nr
        theta = t* twopi /nr
        pts.append ((0., R, L))
        pts.append ((r*cos(theta), R + r*sin(theta), 0.))
    return pts

def _pointsCircularArrow (R, r, nR, nr):
    ptsCone =  _pointsCone (1.5*r, 3*r, nr, R)
    ptsTorus = _pointsTorus (R, r, nR, nr)
    return ptsCone + ptsTorus, len(ptsCone)

# Compute a transformation which rotates (1, 0, 0) to v
# Return (norm_v, T)
# When norm_v == 0, T is None
def _tranformFromXvector (v):
    norm_v = np.linalg.norm(v)
    if norm_v == 0:
        # self.plugin.gui.gui.resizeArrow (n, self.radius, norm_v)
        return 0, None
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
    return norm_v, t

class VelGetter:
    suffixLin = "linear"
    suffixAng = "angular"
    radius = 0.005
    torusR = 0.1

    def __init__(self, plugin, name):
        self.plugin = plugin
        self.name = str(name)

    def getV(self):
        return self.plugin.client.robot.getJointVelocityInLocalFrame (self.name)

    def createNodes(self, color):
        self.group = str(self.plugin.jointgroupcreator.requestCreateJointGroup(self.name))
        self.lin = self.group + '/' + VelGetter.suffixLin
        self.ang = self.group + '/' + VelGetter.suffixAng
        self.plugin.gui.gui.addArrow (self.lin, VelGetter.radius, 1, color)
        pts, self.minC = _pointsCircularArrow (VelGetter.torusR, VelGetter.radius, 100, 20)
        self.plugin.gui.gui.addCurve (self.ang, pts, color)
        self.plugin.gui.gui.setCurveMode (self.ang, "quad_strip")
        self.maxC = len(pts)

    def removeNodes(self):
        self.plugin.gui.gui.deleteNode (self.lin, False)
        self.plugin.gui.gui.deleteNode (self.ang, False)

    def apply(self):
        gui = self.plugin.gui.gui
        v = self.getV()
        norm_vl, tl = _tranformFromXvector (v[:3])
        norm_va, ta = _tranformFromXvector (v[3:])

        count = min (self.minC + int (norm_va * (self.maxC-self.minC) / (pi)), self.maxC)

        gui.resizeArrow (self.lin, VelGetter.radius, norm_vl)
        gui.setCurvePointsSubset (self.ang, 0, count)

        if tl is not None: gui.applyConfiguration (self.lin, tl)
        if ta is not None: gui.applyConfiguration (self.ang, ta)

class ComGetter:
    suffixLin = "linear"
    radius = 0.005
    def __init__(self, plugin, com):
        self.plugin = plugin
        self.name = str(com)

    def getV(self):
        return self.plugin.client.robot.getVelocityPartialCom (self.name)

    def createNodes(self, color):
        self.group = str(self.plugin.comgroupcreator.requestCreateComGroup(self.name))
        self.lin = self.group + '/' + ComGetter.suffixLin
        self.plugin.gui.gui.addArrow (self.lin, ComGetter.radius, 1, color)

    def removeNodes(self):
        self.plugin.gui.gui.deleteNode (self.lin, False)

    def apply(self):
        gui = self.plugin.gui.gui
        v = self.getV()
        norm_v, t = _tranformFromXvector (v)

        gui.resizeArrow (self.lin, ComGetter.radius, norm_v)
        if t is not None: gui.applyConfiguration (self.lin, t)

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

class Data:
    def __init__ (self, plugin):
        self.plugin = plugin
        self.mpl = self.plugin.mplWidget
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
        self.timer.disconnect(QtCore.SIGNAL("timeout()"))
        self.timer.connect(QtCore.SIGNAL("timeout()"), f)

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

        fig = self.mpl.figure
        fig.clf()
        gca = fig.gca ()
        for elt in self.ys:
            gca.plot (self.npdatas [:,self.x[1]], self.npdatas [:,elt[1]], label=elt[0])
        gca.set_xlabel (self.x[0])
        self.plugin.rightPane.repaint()
        # pylab.legend (loc='best')
        self.mpl.canvas.draw ()
        # self.background = self.canvas.copy_from_bbox (gca.bbox)
        # self.cursor = gca.axvline(x=0, color='r', linestyle='--', animated=True)
        # self.canvas.blit (gca.bbox)

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

        self.data = Data(self)

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
        self.mplWidget = MatplotlibWidget(self, True)
        layout.addWidget (self.mplWidget)
        self.mplWidget.canvas.mpl_connect ("button_release_event", self._canvasReleased)

        self.xselect = QtGui.QComboBox(self)
        layout.addWidget (self.xselect)

        self.progressBar = QtGui.QProgressBar(self)
        self.progressBar.setRange(0,100)
        layout.addWidget (self.progressBar)

        # time index is 0 and is value is -1
        self.xselect.addItem ("time", -1)

    def _canvasReleased (self, event):
        if self.mplWidget.toolbar._active is not None:
            return
        self.last_event = event
        if not event.button == 1:
          return False
        l = event.xdata
        self.pathPlayer.setCurrentTime(l)
        return True

    def resetConnection(self):
        self.client = Client(url= str(self.hppPlugin.getHppIIOPurl()))
        self.gui = GuiClient()

    def getJointActions(self, name):
        if not self.jointActions.has_key(name):
            self.jointActions[name] = (JointAction ("Show/Hide joint &velocity", name, self.velocities, self),)
        return self.jointActions[name]
