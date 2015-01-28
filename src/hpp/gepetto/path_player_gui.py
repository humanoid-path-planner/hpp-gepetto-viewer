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

class PathPlayerGui:

  def __init__(self, client, publisher):
    self.gladefile = "@PATH_PLAYER_GLADE_FILENAME@"
    self.glade = gtk.Builder ()
    self.glade.add_from_file (self.gladefile)
    self.glade.get_object ("MainWindow").connect("destroy", self.quit)
    self.publisher = publisher
    self.client = client
    self.l = 0.
    self.dt = 1. / 25.
    self.total_time = 1
    self.isPlaying  = False
    self.pauseRequest = False
    self.pathplayer = None
    self.createPlotCheckButton (self.glade.get_object ("VBoxYSelect"))
    self.fillComboBoxXSelect (self.glade.get_object ("XSelectList"))
    self.plotRefresher = _Gnuplot (self, self.glade.get_object ("ProgressBarPlot"))
    handlers = {
      "on_Time_value_changed": self.on_time_changed,
      "on_PathIndex_value_changed": self.on_pathindex_changed,
      "on_PlayButton_clicked": self.on_play_clicked,
      "on_PauseButton_clicked": self.on_pause_clicked,
      "on_PathScale_value_changed": self.on_pathscale_changed,
      "on_ButtonPlotRefresh_clicked": self.refreshPlot,
      }
    self.glade.connect_signals (handlers)
    self.refresh ()
    self.dl = self.pathLength * self.dt / self.total_time

  def refresh (self):
    nbPaths = self.client.problem.numberPaths ()
    if nbPaths > 0:
      self.glade.get_object ("PathIndex").set_range (0, nbPaths - 1)
      self.pathId = self.glade.get_object ("PathIndex").get_value_as_int ()
      self.pathLength = self.client.problem.pathLength (self.pathId)
      self.glade.get_object ("PathScale").set_range (0, self.pathLength)
    else:
      self.glade.get_object ("PathIndex").set_range (0, 0)
      self.pathLength = 0
    self.glade.get_object ("Time").set_value (self.total_time)

  def on_time_changed (self, w):
    self.total_time = w.get_value ()
    self.dl = self.pathLength * self.dt / self.total_time

  def on_pathindex_changed (self, w):
    self.pathId = w.get_value_as_int ()
    self.pathLength = self.client.problem.pathLength (self.pathId)
    self.glade.get_object ("PathScale").set_range (0, self.pathLength)
    self.dl = self.pathLength * self.dt / self.total_time

  def on_play_clicked (self, w):
    if not self.isPlaying:
      self.isPlaying = True
      glib.timeout_add (int (1000*self.dt), self.path_pulse)

  def on_pause_clicked (self, w):
    if self.isPlaying:
      self.pauseRequest = True

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
    self.glade.get_object ("PathScale").set_value (self.l)
    return True

  def createPlotCheckButton (self, w):
    self.yselectcb = list ()
    rank = 0
    for n in self.client.robot.getJointNames ():
      cb = gtk.CheckButton (label = n)
      self.yselectcb.append ((cb, rank))
      w.pack_end (cb)
      rank = rank + self.client.robot.getJointConfigSize (n)

  def fillComboBoxXSelect (self, w):
    rank = 0
    for n in self.client.robot.getJointNames ():
      w.append ([n,rank])
      rank = rank + self.client.robot.getJointConfigSize (n)

  def refreshPlot (self, w):
    pb = self.glade.get_object ("ProgressBarPlot")
    xselect = self.glade.get_object ("ComboBoxXSelect")
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
    self.glade.get_object ("MainWindow").show_all ()
    gtk.main ()

  def quit (self, window):
    self.glade.get_object ("PauseButton").clicked ()
    gtk.main_quit ()

class _Gnuplot:
  def __init__ (self, pp, progressbar):
    self.pp = pp
    self.pb = progressbar
    self.jobname = '/tmp/path' 

    self.l = 0
    self.dataAreOld = True
    self.pathLength = None
    self.pathId = None
    self.dl = None

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
    else:
      self.dataAreOld = False

  def init_pulse (self):
    if self.dataAreOld:
      self.pb.set_text ("Generating datas...")
      self.pb.set_fraction (0)
      with open ("%s.dat" % (self.jobname, ), 'w') as file:
        file.write ("Data are automatically generated by hpp.gepetto.PathPlayerGui from package hpp-gepetto-viewer\n")
      glib.idle_add (self.writeDATA_pulse)
    else:
      glib.idle_add (self.writeGPLT_pulse)
    return False

  def writeDATA_pulse (self):
    with open ("%s.dat" % (self.jobname, ), 'a') as file:
      file.write (str (self.l) + "\t")
      file.write ("\t".join ([str (v) for v in self.pp.client.problem.configAtParam (self.pathId, self.l)]))
      file.write ("\n")
    self.l += self.dl
    if self.l < self.pathLength:
      self.pb.set_fraction (self.l / self.pathLength)
      return True
    else:
      self.pb.set_fraction (1)
      glib.idle_add (self.writeGPLT_pulse)
      return False

  def writeGPLT_pulse (self):
    self.pb.set_text ("Generating plots...")
    gnuplot_filename = "%s.gplt" % (self.jobname, )
    gnu_plotcommand = "plot " + ", ".join (
        ["'%s.dat' using %i:%i title '%s'" % (self.jobname, self.x[1], elt[1], elt[0], ) for elt in self.ys]
        )
    with open(gnuplot_filename, "w") as gnuplot_file:
      gnuplot_file.write(
          """set style data lines\nset title "%s"\nset terminal png\nset output "%s.png"\n%s\n""" % (self.x[0], self.jobname, gnu_plotcommand, )
          )
    gnuplot_runc = "gnuplot %s.gplt" % (self.jobname, )
    os.system (gnuplot_runc)
    img_src = "%s.png" % (self.jobname, )
    self.pp.glade.get_object ("ImagePlot").set_from_file (img_src)
    self.pb.set_text ("Idle")
    return False
