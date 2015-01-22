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

import sys, pygtk, time, gtk, glib
pygtk.require("2.0")

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
    handlers = {
      "on_Time_value_changed": self.on_time_changed,
      "on_PathIndex_value_changed": self.on_pathindex_changed,
      "on_PlayButton_clicked": self.on_play_clicked,
      "on_PauseButton_clicked": self.on_pause_clicked,
      "on_PathScale_value_changed": self.on_pathscale_changed
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

  def show (self):
    self.glade.get_object ("MainWindow").show_all ()
    gtk.main ()

  def quit (self, window):
    self.glade.get_object ("PauseButton").clicked ()
    gtk.main_quit ()
