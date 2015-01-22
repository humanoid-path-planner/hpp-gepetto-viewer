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

from multiprocessing import Event, Lock, Process
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
    self.t = 0;
    self.dt = 0.01
    self.isPlaying  = Lock ()
    self.pauseRequest = Event ()
    self.pathplayer = None
    handlers = {
      "on_Speed_value_changed": self.on_speed_changed,
      "on_PathIndex_value_changed": self.on_pathindex_changed,
      "on_PlayButton_clicked": self.on_play_clicked,
      "on_PauseButton_clicked": self.on_pause_clicked,
      "on_PathScale_value_changed": self.on_pathscale_changed
      }
    self.glade.connect_signals (handlers)
    self.refresh ()

  def refresh (self):
    nbPaths = self.client.problem.numberPaths ()
    if nbPaths > 0:
      self.glade.get_object ("PathIndex").set_range (0, nbPaths - 1)
      self.pathId = self.glade.get_object ("PathIndex").get_value_as_int ()
      self.pathLength = self.client.problem.pathLength (self.pathId)
      self.glade.get_object ("PathScale").set_range (0, self.pathLength)
    else:
      self.glade.get_object ("PathIndex").set_range (0, 0)
    self.glade.get_object ("Speed").set_value (self.dt)

  def on_speed_changed (self, w):
    self.dt = w.get_value ()

  def on_pathindex_changed (self, w):
    self.pathId = w.get_value_as_int ()
    self.pathLength = self.client.problem.pathLength (self.pathId)
    self.glade.get_object ("PathScale").set_range (0, self.pathLength)

  def on_play_clicked (self, w):
    if self.isPlaying.acquire (False):
      self.pathplayer = Process (target = self.playpath)
      self.pathplayer.start ()

  def on_pause_clicked (self, w):
    if self.pathplayer is not None and self.pathplayer.is_alive ():
      self.pauseRequest.set ()
      self.pathplayer.join ()
      self.pauseRequest.clear ()
    self.pathplayer = None

  def on_pathscale_changed (self, w):
    if self.pathplayer is not None and self.pathplayer.is_alive ():
      return
    self.t = w.get_value ()
    q = self.client.problem.configAtParam (self.pathId, self.t)
    self.publisher.robotConfig = q
    self.publisher.publishRobots ()

  def playpath (self):
    while self.t < self.pathLength:
      q = self.client.problem.configAtParam (self.pathId, self.t)
      self.publisher.robotConfig = q
      self.publisher.publishRobots ()
      if self.pauseRequest.is_set ():
        break
      self.t += self.dt
      time.sleep (self.dt)
    self.isPlaying.release ()

  def show (self):
    self.glade.get_object ("MainWindow").show_all ()
    gtk.main ()

  def quit (self, window):
    self.glade.get_object ("PauseButton").clicked ()
    gtk.main_quit ()

