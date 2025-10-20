# BSD 2-Clause License

# Copyright (c) 2014-2025, CNRS - INRIA

# Author: Joseph Mirabel

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import math
import pickle as pk
import time

import hpp.corbaserver.client
import numpy as np
import omniORB.any


class PathPlayer:
    """
    displays a path by sampling configurations along the path.
    """

    def __init__(self, publisher, client=None, dt=0.01, speed=1):
        if client is None:
            client = hpp.corbaserver.Client()
        self.client = client
        self.publisher = publisher
        self.dt = dt
        self.speed = speed
        self.start = 0.0
        self.end = 1.0

    def setDt(self, dt):
        self.dt = dt

    def setSpeed(self, speed):
        self.speed = speed

    def __call__(self, pathId):
        length = self.end * self.client.problem.pathLength(pathId)
        t = self.start * self.client.problem.pathLength(pathId)
        while t < length:
            start = time.time()
            q = self.client.problem.configAtParam(pathId, t)
            self.publisher.robotConfig = q
            self.publisher.publishRobots()
            self.publisher.client.gui.refresh()
            t += self.dt * self.speed
            elapsed = time.time() - start
            if elapsed < self.dt:
                time.sleep(self.dt - elapsed)

    def displayPath(self, pathId, color=[0.85, 0.75, 0.15, 0.9], jointName=0):
        """
        Display the path of the desired joint
        By default, display the path of the root
        """
        if jointName == 0:
            if self.publisher.robot.rootJointType == "planar":
                jointName = self.publisher.robot.tf_root + "_joint"
        pathPos = []
        length = self.end * self.client.problem.pathLength(pathId)
        t = self.start * self.client.problem.pathLength(pathId)
        while t < length:
            q = self.client.problem.configAtParam(pathId, t)
            if jointName != 0:
                self.publisher.robot.setCurrentConfig(q)
                q = self.publisher.robot.getLinkPosition(jointName)
            pathPos = [*pathPos, q[:3]]
            t += self.dt
        if jointName == 0:
            jointName = "root"
        nameCurve = "path_" + str(pathId) + "_" + jointName
        self.publisher.client.gui.addCurve(nameCurve, pathPos, color)
        self.publisher.client.gui.addToGroup(nameCurve, self.publisher.sceneName)
        self.publisher.client.gui.refresh()

    def displayVelocityPath(self, pathId, jointName=0):
        """
        Display the path with a color variation according to the velocity along the path
        green for velocity close to 0, red for velocity close to the velocity bound.
        This method assume that the path is represented
        by instances of the KinodynamicPath class
        """
        if self.client.robot.getDimensionExtraConfigSpace() < 6:
            raise RuntimeError(
                "DisplayVelocityPath can only be used if the robot have at least "
                "6 extraDof storing velocity and acceleration of the root."
            )
        if jointName == 0:
            if self.publisher.robot.rootJointType == "planar":
                jointName = self.publisher.robot.tf_root + "_joint"
        vmax = omniORB.any.from_any(
            self.client.problem.getParameter("Kinodynamic/velocityBound")
        ) * math.sqrt(2)
        configSize = (
            self.client.robot.getConfigSize()
            - self.client.robot.getDimensionExtraConfigSpace()
        )
        last_q = 0
        length = self.end * self.client.problem.pathLength(pathId)
        if jointName == 0:
            nameCurve = "path_" + str(pathId) + "_root"
        else:
            nameCurve = "path_" + str(pathId) + "_" + jointName
        self.publisher.client.gui.createGroup(nameCurve)
        t = self.start * self.client.problem.pathLength(pathId)
        while t < length:
            q = self.client.problem.configAtParam(pathId, t)
            if jointName != 0:
                self.publisher.robot.setCurrentConfig(q)
                q = self.publisher.robot.getLinkPosition(jointName)
            if last_q != 0:
                name = nameCurve + "/line_" + str(t)
                v = (
                    math.sqrt(
                        q[configSize] * q[configSize]
                        + q[configSize + 1] * q[configSize + 1]
                        + q[configSize + 2] * q[configSize + 2]
                    )
                ) / vmax
                if v < 0.5:
                    color = [v * 2.0, 1, 0, 1]
                else:
                    color = [1, 2 - (v * 2.0), 0, 1]
                self.publisher.client.gui.addLine(name, last_q, q[0:3], color)
                self.publisher.client.gui.addToGroup(name, nameCurve)
            last_q = q[0:3]
            t += self.dt
        self.publisher.client.gui.addToGroup(nameCurve, self.publisher.sceneName)
        self.publisher.client.gui.refresh()

    def generatePlot(
        self, pathId, play=False, jointNames=None, cursor=False, filename=None
    ):
        """
        Generate a plot with the following datas:
        - abscissa: time.
        - ordinate: parameter values of the joint names.

        If play is True, the path is played when the data to be plotted are generated.

        If cursor is True, filename must be a string which will be formatted with one
        integer (frame index).
        For each frame, a cursor is drawn at the current time and the plot is written
        to the formatted filename.

        If cursor is False, only one plot is generated. Then,
        - If filename is None, the plot is shown in a new window;
        - If filename is a string, the plot is written to filename.
        """
        # First generate the datas
        length = self.end * self.client.problem.pathLength(pathId)
        t = self.start * self.client.problem.pathLength(pathId)
        ts = list()
        qs = list()
        while t < length:
            start = time.time()
            ts.append(t)
            try:
                qs.append(self.client.problem.configAtParam(pathId, t))
                if play:
                    self.publisher.robotConfig = qs[-1]
                    self.publisher.publishRobots()
                    self.publisher.client.gui.refresh()
            except Exception:
                qs.append([np.nan] * self.client.robot.getConfigSize())
                if play:
                    self.publisher.publishRobots()
                    self.publisher.client.gui.refresh()
            t += self.dt * self.speed
            elapsed = time.time() - start
            if elapsed < self.dt:
                time.sleep(self.dt - elapsed)

        # Filter the datas
        filter = list()
        labels = list()
        if jointNames is None:
            jointNames = self.publisher.robot.jointNames
        for jn in jointNames:
            rk = self.publisher.robot.rankInConfiguration[jn]
            sz = self.publisher.robot.getJointConfigSize(jn)
            filter.extend(list(range(rk, rk + sz)))
            if sz > 1:
                labels.extend([jn + str(x) for x in range(0, sz)])
            else:
                labels.append(jn)

        data = (np.array(qs)[:, filter]).transpose()

        import matplotlib.pyplot as plt

        fig = plt.figure()
        ax = fig.add_subplot(111)
        for d, label in zip(data, labels):
            ax.plot(ts, d, label=label)
        if cursor:
            for i in range(len(ts)):
                c = ax.axvline(x=ts[i], color="r", linestyle="--")
                fn = filename.format(i)
                fig.savefig(fn, dpi=150)
                c.remove()
        else:
            if filename is not None:
                fig.savefig(filename, dpi=150)
            else:
                plt.show()

    def toFile(self, pathId, fname):
        length = self.client.problem.pathLength(pathId)
        t = 0
        tau = []
        while t < length:
            q = self.client.problem.configAtParam(pathId, t)
            tau.append(q)
            t += self.dt * self.speed
        fh = open(fname, "wb")
        pk.dump(tau, fh)
        fh.close()

    def toFileAppend(self, pathId, fname):
        length = self.client.problem.pathLength(pathId)
        t = 0
        tau = []
        while t < length:
            q = self.client.problem.configAtParam(pathId, t)
            tau.append(q)
            t += self.dt
        fh = open(fname, "a")
        pk.dump(tau, fh)
        fh.close()

    def getTrajFromFile(self, fname):
        fh = open(fname, "rb")
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
                start = time.time()
                self.publisher.robotConfig = q
                self.publisher.publishRobots()
                self.publisher.client.gui.refresh()
                elapsed = time.time() - start
                if elapsed < self.dt:
                    time.sleep(self.dt - elapsed)
