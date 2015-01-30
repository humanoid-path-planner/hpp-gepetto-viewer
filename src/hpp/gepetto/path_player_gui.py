# Copyright (c) 2015 CNRS
# Author: Joseph Mirabel
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

import pygtk
pygtk.require("2.0")
import os, sys, time, gtk, glib

import numpy as np
import pylab
# uncomment to select /GTK/GTKAgg/GTKCairo
#from matplotlib.backends.backend_gtk      import FigureCanvasGTK as FigureCanvas
#from matplotlib.backends.backend_gtk      import NavigationToolbar2GTK as NavigationToolbar
from matplotlib.backends.backend_gtkagg   import FigureCanvasGTKAgg as FigureCanvas
from matplotlib.backends.backend_gtkagg   import NavigationToolbar2GTKAgg as NavigationToolbar
#from matplotlib.backends.backend_gtkcairo import FigureCanvasGTKCairo as FigureCanvas
#from matplotlib.backends.backend_gtkcairo import NavigationToolbar2GTKCairo as NavigationToolbar

class PathPlayerGui (object):

  def __init__(self, client, publisher):
    self.gladefile = "@PATH_PLAYER_GLADE_FILENAME@"
    self.glade = gtk.Builder ()
    self.glade.add_from_file (self.gladefile)
    self ["MainWindow"].connect("destroy", self.quit)
    self.publisher = publisher
    self.client = client
    self.l = 0.
    self.dt = 1. / 25.
    self.total_time = 1
    self.isPlaying  = False
    self.pauseRequest = False
    self.createPlotCheckButton (self ["VBoxYSelect"])
    self.fillComboBoxXSelect (self ["XSelectList"])
    self.plotRefresher = _Matplotlib (self, self ["ProgressBarPlot"])
    handlers = {
      "on_Time_value_changed": self.on_time_changed,
      "on_PathIndex_value_changed": self.on_pathindex_changed,
      "on_PlayButton_clicked": self.on_play_clicked,
      "on_PauseButton_clicked": self.on_pause_clicked,
      "on_StopButton_clicked": self.on_stop_clicked,
      "on_PathScale_value_changed": self.on_pathscale_changed,
      "on_ButtonPlotRefresh_clicked": self.refreshPlot,
      "on_PathScale_value_changed": self.on_pathscale_changed
      }
    self.glade.connect_signals (handlers)
    self.refresh ()
    self.adjust_increments ()

  def refresh (self):
    nbPaths = self.client.problem.numberPaths ()
    if nbPaths > 0:
      self ["PathIndex"].set_range (0, nbPaths - 1)
      self.pathId = self ["PathIndex"].get_value_as_int ()
      self.pathLength = self.client.problem.pathLength (self.pathId)
      self ["PathAdjustment"].set_lower (0)
      self ["PathAdjustment"].set_upper (self.pathLength)
    else:
      self ["PathIndex"].set_range (0, 0)
      self.pathLength = 0
    self ["Time"].set_value (self.total_time)

  def on_time_changed (self, w):
    self.total_time = w.get_value ()
    self.adjust_increments ()

  def adjust_increments (self):
    self.dl = self.pathLength * self.dt / self.total_time
    self ["PathAdjustment"].set_step_increment (self.dl)
    self ["PathAdjustment"].set_page_increment (10 * self.dl)

  def on_pathindex_changed (self, w):
    self.pathId = w.get_value_as_int ()
    self.pathLength = self.client.problem.pathLength (self.pathId)
    self ["PathAdjustement"].set_upper (self.pathLength)
    self.adjust_increments ()

  def on_play_clicked (self, w):
    if not self.isPlaying:
      self.isPlaying = True
      glib.timeout_add (int (1000*self.dt), self.path_pulse)

  def on_pause_clicked (self, w):
    if self.isPlaying:
      self.pauseRequest = True

  def on_stop_clicked (self, w):
    if self.isPlaying:
      self.pauseRequest = True
    self["PathScale"].set_value (0)

  def on_pathscale_changed (self, w):
    self.l = w.get_value ()
    self.publisher.robotConfig = self.client.problem.configAtParam (self.pathId, self.l)
    self.publisher.publishRobots ()

  def path_pulse (self):
    if self.pauseRequest or self.l > self.pathLength:
      if self.l > self.pathLength:
        self.l = self.pathLength
      self.pauseRequest = False
      self.isPlaying = False
      return False
    self.l += self.dl
    self["PathScale"].set_value (self.l)
    return True

  def createPlotCheckButton (self, w):
    self.yselectcb = list ()
    rank = 0
    for n in self.client.robot.getJointNames ():
      size = self.client.robot.getJointConfigSize (n)
      if size == 1:
        cb = gtk.CheckButton (label = n)
        self.yselectcb.append ((cb, rank))
        w.pack_end (cb)
      else:
        for i in xrange (size):
          cb = gtk.CheckButton (label = "%s (%i)" % (n, i))
          self.yselectcb.append ((cb, rank + i))
          w.pack_end (cb)
      rank = rank + size

  def fillComboBoxXSelect (self, w):
    rank = 0
    for n in self.client.robot.getJointNames ():
      size = self.client.robot.getJointConfigSize (n)
      if size == 1:
        w.append ([n,rank])
      else:
        for i in xrange (size):
          w.append (["%s (%i)" % (n, i),rank+i])
      rank = rank + size

  def refreshPlot (self, w):
    pb = self ["ProgressBarPlot"]
    xselect = self ["ComboBoxXSelect"]
    xiter = xselect.get_active_iter ()
    if iter is None:
      pb.set_text ("Wrong X data")
      return
    # time index is 0 and is value is -1
    x = (xselect.get_model ().get_value (xiter, 0),\
         xselect.get_model ().get_value (xiter, 1) + 1)
    ys = list ()
    for elt in self.yselectcb:
      cb = elt[0]
      if cb.get_active ():
        ys.append ((cb.get_label (), elt[1]+1))
    if len (ys) is 0:
      pb.set_text ("Wrong Y data")
      return
    self.plotRefresher.selectData (x, ys)
    glib.idle_add (self.plotRefresher.init_pulse)

  def show (self):
    self ["MainWindow"].show_all ()
    gtk.main ()

  def __getitem__ (self, key):
    return self.glade.get_object (key)

  def quit (self, window):
    self ["PauseButton"].clicked ()
    gtk.main_quit ()

class _Matplotlib:
  def __init__ (self, pp, progressbar):
    self.pp = pp
    self.pb = progressbar
    self.figure = pylab.figure ()
    self.canvas = FigureCanvas (self.figure)
    self.toolbar = NavigationToolbar (self.canvas, pp ["MainWindow"])
    self.pp ["BoxPlotArea"].pack_start (self.toolbar, expand = False, fill = False) 
    self.pp ["BoxPlotArea"].pack_start (self.canvas , expand = True, fill = True) 
    self.canvas.connect ("button_press_event", self.on_button_press_event)

    self.l = 0
    self.dataAreOld = True
    self.pathLength = None
    self.pathId = None
    self.dl = None

  def get_x_cursor_position(self,event):
    gca = self.figure.gca ()
    ap = gca.get_position ()
    xmin, xmax = gca.get_xbound ()
    x,y = self.canvas.get_width_height()
    posx = ((event.x/x)-ap.x0)*(1/(ap.x1-ap.x0))
    sx = (posx*(xmax - xmin)) + xmin
    return sx

  def on_button_press_event (self, w, event):
    if   not event.type == gtk.gdk._2BUTTON_PRESS \
      or not event.button == 1:
      return False
    l = self.get_x_cursor_position(event)
    self.pp ["PathScale"].set_value (l)
    return True

  def selectData (self, x, ys):
    self.x = x
    self.ys = ys
    if   not self.pathLength == self.pp.pathLength \
      or not self.pathId     == self.pp.pathId \
      or not self.dl         == self.pp.dl:
      self.dataAreOld = True
      self.pathId = self.pp.pathId
      self.pathLength = self.pp.pathLength
      self.dl = self.pp.dl
      self.l = 0
      self.datas = list ()
    else:
      self.dataAreOld = False

  def init_pulse (self):
    if self.dataAreOld:
      self.pb.set_text ("Generating datas...")
      self.pb.set_fraction (0)
      glib.idle_add (self.getData_pulse)
    else:
      glib.idle_add (self.genPlot_pulse)
    return False

  def getData_pulse (self):
    d = [ self.l, ]
    d.extend (self.pp.client.problem.configAtParam (self.pathId, self.l))
    self.datas.append (d)
    self.l += self.dl
    if self.l < self.pathLength:
      self.pb.set_fraction (self.l / self.pathLength)
      return True
    else:
      self.pb.set_fraction (1)
      glib.idle_add (self.genPlot_pulse)
      return False

  def genPlot_pulse (self):
    self.pb.set_text ("Generating plots...")
    self.npdatas = np.matrix (self.datas)
    self.figure.clf ()
    gca = pylab.gca ()
    for elt in self.ys:
      pylab.plot (self.npdatas [:,self.x[1]], self.npdatas [:,elt[1]], label=elt[0])
    gca.set_xlabel (self.x[0])
    pylab.legend (loc='best')
    self.canvas.draw ()
    self.pb.set_text ("Idle")
    return False
