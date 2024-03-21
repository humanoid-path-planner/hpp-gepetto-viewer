import hpp_idl
import numpy as np
from hpp.corbaserver import Client
from PythonQt import Qt, QtCore, QtGui

from gepetto.color import Color
from gepetto.corbaserver import Client as GuiClient
from gepetto.corbaserver.tools import Linear, Vector6

colors = (
    # Qt.Qt.white,
    Qt.Qt.black,
    Qt.Qt.red,
    Qt.Qt.green,
    Qt.Qt.blue,
    Qt.Qt.cyan,
    Qt.Qt.magenta,
    Qt.Qt.yellow,
    Qt.Qt.gray,
    Qt.Qt.darkRed,
    Qt.Qt.darkGreen,
    Qt.Qt.darkBlue,
    Qt.Qt.darkCyan,
    Qt.Qt.darkMagenta,
    Qt.Qt.darkYellow,
    Qt.Qt.darkGray,
    Qt.Qt.lightGray,
)
lineStyles = (
    Qt.Qt.SolidLine,
    Qt.Qt.DashLine,
    Qt.Qt.DotLine,
    Qt.Qt.DashDotLine,
    Qt.Qt.DashDotDotLine,
)

pens = []
for ls in lineStyles:
    for c in colors:
        qpen = Qt.QPen(c)
        qpen.setStyle(ls)
        pens.append(qpen)

pens = tuple(pens)


class VelGetter:
    def __init__(self, plugin, name):
        self.plugin = plugin
        self.name = str(name)
        self.vector6 = Vector6(self.name)

    def getV(self):
        return self.plugin.client.robot.getJointVelocityInLocalFrame(self.name)

    def createNodes(self, color):
        self.group = str(
            self.plugin.jointgroupcreator.requestCreateJointGroup(self.name)
        )
        self.vector6.color = color
        self.vector6.create(self.plugin.gui.gui)

    def removeNodes(self):
        self.vector6.remove(self.plugin.gui.gui)

    def apply(self):
        v = self.getV()
        self.vector6.set(self.plugin.gui.gui, v)


class ComGetter:
    def __init__(self, plugin, com):
        self.plugin = plugin
        self.name = str(com)
        self.linear = Linear(self.name)

    def getV(self):
        return self.plugin.client.robot.getVelocityPartialCom(self.name)

    def createNodes(self, color):
        self.group = str(self.plugin.comgroupcreator.requestCreateComGroup(self.name))
        self.linear.color = color
        self.linear.create(self.plugin.gui.gui)

    def removeNodes(self):
        self.linear.remove(self.plugin.gui.gui)

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
            self.plugin.main.connect(
                QtCore.SIGNAL("applyCurrentConfiguration()"), self.applyAll
            )
            self.connected = True

    def disconnect(self):
        if self.connected:
            self.plugin.main.disconnect(
                QtCore.SIGNAL("applyCurrentConfiguration()"), self.applyAll
            )
            self.connected = False

    def add(self, getter):
        self.getters[getter.name] = getter
        getter.apply()
        self.connect()

    def remove(self, node):
        getter = self.getters.pop(node)
        getter.removeNodes()
        if len(self.getters) == 0:
            self.disconnect()

    def toggleJoint(self, joint):
        if joint in self.getters:
            self.remove(joint)
        else:
            g = VelGetter(self.plugin, joint)
            g.createNodes(Velocities.color)
            self.add(g)

    def toggleCom(self, com):
        if com in self.getters:
            self.remove(com)
        else:
            g = ComGetter(self.plugin, com)
            g.createNodes(Velocities.color)
            self.add(g)

    def applyAll(self):
        for n, getV in list(self.getters.items()):
            getV.apply()


class JointAction(Qt.QAction):
    def __init__(self, action, joint, velocities, parent):
        super().__init__(action, parent)
        self.joint = joint
        self.velocities = velocities
        self.connect(QtCore.SIGNAL("triggered(bool)"), self.trigger)

    def trigger(self):
        self.velocities.toggleJoint(self.joint)


class DataQCP:
    def __init__(self, plugin, qcpWidget):
        self.plugin = plugin
        self.qcp = qcpWidget
        self.timer = Qt.QTimer()
        self.timer.setSingleShot(True)

        self.pathLength = -1
        self.dl = -1
        self.pathId = -1

    def selectData(self, pid, dl, x, ys):
        if dl < 1e-8:
            return
        self.x = x
        self.ys = ys
        pl = self.plugin.client.problem.pathLength(pid)
        if not self.pathLength == pl or not self.pathId == pid or not self.dl == dl:
            self.dataAreOld = True
            self.pathId = pid
            self.pathLength = pl
            self.dl = dl
            self.le = 0
            self.datas = list()
        else:
            self.dataAreOld = False
        self.qcp.clearGraphs()
        for i, elt in enumerate(self.ys):
            graph = self.qcp.addGraph()
            graph.name = elt[0]
            graph.pen = pens[i]
        self.qcp.xAxis().label = self.x[0]

    def acquireData(self):
        if self.dataAreOld:
            self._setNextCall(self._getNextData)
        else:
            self._setNextCall(self._genPlot)
        self.timer.start(0)

    def _setNextCall(self, f):
        self.timer.disconnect(Qt.SIGNAL("timeout()"))
        self.timer.connect(Qt.SIGNAL("timeout()"), f)

    def _getDerivative(self, order):
        try:
            if order == 0:
                return self.plugin.client.problem.configAtParam(self.pathId, self.le)
            else:
                return self.plugin.client.problem.derivativeAtParam(
                    self.pathId, order, self.le
                )
        except hpp_idl.hpp.Error as e:
            if order == 0:
                return [np.nan] * self.plugin.client.robot.getConfigSize()
            else:
                return [np.nan] * self.plugin.client.robot.getNumberDof()
            print(str(e))

    def _getNextData(self):
        d = [
            self.le,
        ]
        d.extend(self._getDerivative(0))
        d.extend(self._getDerivative(1))
        d.extend(self._getDerivative(2))
        self.datas.append(d)

        for i, elt in enumerate(self.ys):
            graph = self.qcp.graph(i)
            graph.addData(self.le, d[elt[1]])
        self.qcp.replot()

        self.le += self.dl
        if self.le < self.pathLength:
            self._setNextCall(self._getNextData)
        else:
            self._setNextCall(self._finishDataAcquisition)
        self.timer.start(0)

    def _finishDataAcquisition(self):
        self.npdatas = np.matrix(self.datas)
        self.qcp.rescaleAxes()
        self.qcp.replot()

    def _genPlot(self):
        self.qcp.clearGraphs()
        for i, elt in enumerate(self.ys):
            graph = self.qcp.addGraph()
            graph.setData(self.npdatas[:, self.x[1]], self.npdatas[:, elt[1]])
            graph.name = elt[0]
            graph.pen = pens[i % len(pens)]
        self.qcp.xAxis().label = self.x[0]
        self.qcp.rescaleAxes()
        self.qcp.replot()
        # TODO Add a vertical bar at current param along the path.

        return False


formats = ("%s (%s)", "%s (%s, %i)")


def iconFromTheme(name):
    if QtGui.QIcon.hasThemeIcon(name):
        return QtGui.QIcon.fromTheme(name)
    else:
        return QtGui.QIcon()


class QCPWidget(QtGui.QWidget):
    """A plot widget with a combo box to select the X axis."""

    def __init__(self, plugin):
        super().__init__(plugin)
        self.plugin = plugin

        layout = QtGui.QVBoxLayout(self)

        from PythonQt.QCustomPlot import QCP, QCustomPlot

        self.qcpWidget = QCustomPlot()
        self.qcpWidget.autoAddPlottableToLegend = True
        self.qcpWidget.setInteraction(QCP.iRangeDrag, True)  # iRangeDrap
        self.qcpWidget.setInteraction(QCP.iRangeZoom, True)  # iRangeZoom
        self.qcpWidget.setInteraction(QCP.iSelectAxes, True)  # iSelectAxes
        self.qcpWidget.legend().visible = True
        layout.addWidget(self.qcpWidget, 1)
        self.qcpWidget.connect(
            Qt.SIGNAL("mouseDoubleClick(QMouseEvent*)"), self._mouseDoubleClick
        )
        self.qcpWidget.xAxis().connect(
            Qt.SIGNAL("selectionChanged(QCPAxis::SelectableParts)"),
            self._axesSelectionChanged,
        )
        self.qcpWidget.yAxis().connect(
            Qt.SIGNAL("selectionChanged(QCPAxis::SelectableParts)"),
            self._axesSelectionChanged,
        )

        buttonbar = QtGui.QWidget()
        bbLayout = QtGui.QHBoxLayout(buttonbar)
        button = QtGui.QPushButton("Replot", buttonbar)
        button.connect(QtCore.SIGNAL("clicked()"), self.refreshPlot)
        bbLayout.addWidget(button)
        button = QtGui.QPushButton("Remove", buttonbar)
        button.connect(QtCore.SIGNAL("clicked()"), self.removePlot)
        bbLayout.addWidget(button)
        button = QtGui.QPushButton("Legend", buttonbar)
        button.checkable = True
        button.checked = True
        button.connect(QtCore.SIGNAL("toggled(bool)"), self.toggleLegend)
        bbLayout.addWidget(button)
        button = QtGui.QPushButton(
            QtGui.QIcon.fromTheme("zoom-fit-best"), "", buttonbar
        )
        button.setToolTip("Zoom fit best")
        button.connect(QtCore.SIGNAL("clicked()"), self.qcpWidget.rescaleAxes)
        bbLayout.addWidget(button)

        self.xselect = QtGui.QComboBox(self)
        bbLayout.addWidget(self.xselect, 1)
        layout.addWidget(buttonbar)

        self.data = DataQCP(plugin, self.qcpWidget)

    def toggleLegend(self, enable):
        self.qcpWidget.legend().visible = enable
        self.qcpWidget.replot()

    def refreshPlot(self):
        pid = self.plugin.pathPlayer.getCurrentPath()
        if pid < 0:
            return
        dl = self.plugin.pathPlayer.lengthBetweenRefresh()
        idxX = self.xselect.currentIndex
        x = (self.xselect.itemText(idxX), int(self.xselect.itemData(idxX)) + 1)

        ys = list()
        for elt in self.plugin.yselectcb:
            cb = elt[0]
            if cb.checked:
                ys.append((str(cb.text), elt[1] + 1))

        self.data.selectData(pid, dl, x, ys)
        self.data.acquireData()

    def removePlot(self):
        self.hide()
        try:
            self.plugin.qcpWidgets.remove(self)
        except ValueError:
            print("Did not find QCPWidget in plugin")
        self.plugin.rightPaneLayout.removeWidget(self)

    def refreshJointList(self, jointNames, jointCfgSize, jointNbDof):
        # Right pane
        self.xselect.clear()
        # time index is 0 and is value is -1
        self.xselect.addItem("time", -1)
        rank = 0
        for n, size in zip(jointNames, jointCfgSize):
            if size == 1:
                self.xselect.addItem(formats[0] % (n, "q"), rank)
            else:
                for i in range(size):
                    self.xselect.addItem(formats[1] % (n, "q", i), rank + i)
            rank = rank + size
        for type in ("v", "a"):
            for n, size in zip(jointNames, jointNbDof):
                if size == 1:
                    self.xselect.addItem(formats[0] % (n, type), rank)
                else:
                    for i in range(size):
                        self.xselect.addItem(formats[1] % (n, type, i), rank + i)
                rank = rank + size

    def _mouseDoubleClick(self, event):
        try:
            if self.data.x[1] > 0:
                return
        except Exception:
            return
        try:  # Qt 5
            x = event.localPos().x()
        except Exception:  # Qt 4
            x = event.posF().x()
        t = self.qcpWidget.xAxis().pixelToCoord(x)
        self.plugin.pathPlayer.setCurrentTime(t)

    def _axesSelectionChanged(self, unused_parts):
        xAxis = self.qcpWidget.xAxis()
        yAxis = self.qcpWidget.yAxis()
        x = xAxis.selectedParts != 0
        y = yAxis.selectedParts != 0
        if not x and not y:
            self.qcpWidget.axisRect().setRangeZoomAxes(xAxis, yAxis)
        elif x:
            self.qcpWidget.axisRect().setRangeZoomAxes(xAxis, None)
        elif y:
            self.qcpWidget.axisRect().setRangeZoomAxes(None, yAxis)


class Plugin(QtGui.QDockWidget):
    def __init__(self, mainWindow, flags=None):
        if flags is None:
            super().__init__("Path graph plugin", mainWindow)
        else:
            super().__init__("Path graph plugin", mainWindow, flags)
        self.setObjectName("Path graph plugin")

        self.main = mainWindow
        self.hppPlugin = self.main.getFromSlot("getHppIIOPurl")
        self.pathPlayer = self.main.getFromSlot("getCurrentPath")
        self.jointgroupcreator = self.main.getFromSlot("requestCreateJointGroup")
        self.comgroupcreator = self.main.getFromSlot("requestCreateComGroup")
        self.velocities = Velocities(self)
        self.jointActions = dict()
        self.jointNames = None

        self.qcpWidgets = list()

        # This avoids having a widget bigger than what it needs. It avoids having
        # a big dock widget and a small osg widget when creating the main osg widget.
        p = Qt.QSizePolicy.Ignored
        self.topWidget = QtGui.QSplitter(Qt.Qt.Horizontal, self)
        self.topWidget.setSizePolicy(Qt.QSizePolicy(p, p))
        self.setWidget(self.topWidget)

        self.leftPane = QtGui.QWidget(self)
        layout = QtGui.QVBoxLayout()
        self.makeLeftPane(layout)
        self.leftPane.setLayout(layout)

        self.topWidget.addWidget(self.leftPane)

        self.rightPane = QtGui.QWidget(self)
        self.rightPaneLayout = QtGui.QVBoxLayout()
        self.rightPane.setLayout(self.rightPaneLayout)
        self.addPlotBelow()
        self.topWidget.addWidget(self.rightPane)
        self.topWidget.setStretchFactor(1, 1)

    def addPlotBelow(self):
        widget = QCPWidget(self)
        self.qcpWidgets.append(widget)
        self.rightPaneLayout.addWidget(widget)

        if self.jointNames is not None:
            jointCfgSize = [
                self.client.robot.getJointConfigSize(n) for n in self.jointNames
            ]
            jointNbDof = [
                self.client.robot.getJointNumberDof(n) for n in self.jointNames
            ]
            widget.refreshJointList(self.jointNames, jointCfgSize, jointNbDof)

    def refreshJointList(self):
        self.jointNames = self.client.robot.getJointNames()
        # Left pane
        saLayout = QtGui.QVBoxLayout()

        self.yselectcb = list()
        rank = 0
        for n in self.jointNames:
            size = self.client.robot.getJointConfigSize(n)
            if size == 1:
                cb = QtGui.QCheckBox(formats[0] % (n, "q"))
                self.yselectcb.append((cb, rank))
                saLayout.addWidget(cb)
            else:
                for i in range(size):
                    cb = QtGui.QCheckBox(formats[1] % (n, "q", i))
                    self.yselectcb.append((cb, rank + i))
                    saLayout.addWidget(cb)
            rank = rank + size
        for type in ("v", "a"):
            saLayout.addSpacing(5)
            for n in self.jointNames:
                size = self.client.robot.getJointNumberDof(n)
                if size == 1:
                    cb = QtGui.QCheckBox(formats[0] % (n, type))
                    self.yselectcb.append((cb, rank))
                    saLayout.addWidget(cb)
                else:
                    for i in range(size):
                        cb = QtGui.QCheckBox(formats[1] % (n, type, i))
                        self.yselectcb.append((cb, rank + i))
                        saLayout.addWidget(cb)
                rank = rank + size

        saContent = QtGui.QWidget(self)
        saContent.setLayout(saLayout)
        self.scrollArea.setWidget(saContent)

        # Right pane
        jointCfgSize = [
            self.client.robot.getJointConfigSize(n) for n in self.jointNames
        ]
        jointNbDof = [self.client.robot.getJointNumberDof(n) for n in self.jointNames]
        for w in self.qcpWidgets:
            w.refreshJointList(self.jointNames, jointCfgSize, jointNbDof)

    def refreshInterface(self):
        self.refreshJointList()

    def makeLeftPane(self, layout):
        # button = QtGui.QPushButton ("Refresh", self)
        # button.connect (QtCore.SIGNAL("clicked()"), self.refreshPlot)
        # layout.addWidget (button)

        button = QtGui.QPushButton("Add plot below", self)
        button.connect(QtCore.SIGNAL("clicked()"), self.addPlotBelow)
        layout.addWidget(button)

        layout.addWidget(QtGui.QLabel("Select Y data"))
        self.scrollArea = QtGui.QScrollArea(self)
        layout.addWidget(self.scrollArea)

    def resetConnection(self):
        self.client = Client(url=str(self.hppPlugin.getHppIIOPurl()))
        self.gui = GuiClient()

    def getJointActions(self, name):
        if name not in self.jointActions:
            self.jointActions[name] = (
                JointAction("Show/Hide joint &velocity", name, self.velocities, self),
            )
        return self.jointActions[name]
