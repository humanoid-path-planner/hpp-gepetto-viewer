# Copyright (c) 2013, 2014 CNRS
# Author: Florent Lamiraux
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

import time
import numpy as np
import pickle as pk

## displays a path by sampling configurations along the path.
#
class PathPlayer (object):
    def __init__ (self, client, publisher, dt = 0.01, speed = 1) :
        self.client = client
        self.publisher = publisher
        self.dt = dt
        self.speed = speed

    def setDt(self,dt):
      self.dt = dt

    def setSpeed(self,speed) :
      self.speed = speed

    def __call__ (self, pathId) :
        length = self.client.problem.pathLength (pathId)
        t = 0
        while t < length :
            start = time.time()
            q = self.client.problem.configAtParam (pathId, t)
            self.publisher.robotConfig = q
            self.publisher.publishRobots ()
            t += (self.dt * self.speed)
            elapsed = time.time() - start
            if elapsed < self.dt :
              time.sleep(self.dt-elapsed)
            else :
              print("Warning : time step is shorter than computation time for robot geometry ("+str(elapsed)+")")

    def toFile(self, pathId, fname):
        length = self.client.problem.pathLength (pathId)
        t = 0
        tau = []
        while t < length :
            q = self.client.problem.configAtParam (pathId, t)
            tau.append(q)
            t += self.dt
        fh = open(fname,"wb")
        pk.dump(tau,fh)
        fh.close()

    def toFileAppend(self, pathId, fname):
        length = self.client.problem.pathLength (pathId)
        t = 0
        tau = []
        while t < length :
            q = self.client.problem.configAtParam (pathId, t)
            tau.append(q)
            t += self.dt
        fh = open(fname,"a")
        pk.dump(tau,fh)
        fh.close()

    def getTrajFromFile(self, fname):
        fh = open(fname,"rb")
        tau = []
        while 1:
            try:
                tau.append(pk.load(fh))
            except EOFError:
                break
        fh.close()
        return tau

    def fromFile(self, fname):
        self.tau = self.getTrajFromFile(fname)
        for tauK in self.tau:
            for q in tauK:
                self.publisher.robotConfig = q
                self.publisher.publishRobots ()
                time.sleep (self.dt)
