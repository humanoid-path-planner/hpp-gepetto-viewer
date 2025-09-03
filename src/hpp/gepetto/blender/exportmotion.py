#!/usr/bin/env python

# BSD 2-Clause License

# Copyright (c) 2014-2025, CNRS - INRIA

# Author: Steve Tonneau

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


######################################################
#
# Script for exporting a motion computed with gepetto viewer to blender
#
###################################################

import gepetto.corbaserver.exporttoblender as etb


def exportState(viewer, robot, configuration, outData):
    """
    Export object position for a given robot configuration

    \\param robot the considered robot,
    \\param configuration current robot configuration
    \\param outData array containing each object position,
            indexed first by object then by frame
    """
    save = viewer.robotConfig
    viewer(configuration)
    etb.exportState(viewer, robot.getRobotName(), outData)
    viewer(save)
    # ~ robot.setCurrentConfig(configuration)
    # ~ objNames = set([])
    # ~ #retrieve object names
    # ~ for joint in robot.getAllJointNames():
    # ~ for obj in robot.getJointInnerObjects(joint):
    # ~ objNames.add(obj)
    # ~ for obj in robot.getJointOuterObjects(joint):
    # ~ objNames.add(obj)
    # ~ while len(objNames) > 0:
    # ~ obj = objNames.pop()
    # ~ if not outData.has_key(obj):
    # ~ outData[obj] = []
    # ~ objFrame = outData[obj]
    # ~ objFrame.append(robot.getObjectPosition(obj))


def writeDataToFile(robot, outData, filename):
    """
    Export object position for a given robot configuration

    \\param outData data computed by the exportState calls
    \\param filename name of the output file where to save the output
    """
    etb.writeDataToFile(robot.getRobotName(), outData, filename)


def exportStates(viewer, robot, configurations, filename):
    """
    Export object positions for a list of robot configurations

    \\param robot the considered robot,
    \\param configurations list of configurations to consider
    \\param filename name of the output file where to save the output
    """
    outData = {}
    for configuration in configurations:
        exportState(viewer, robot, configuration, outData)
    writeDataToFile(robot, outData, filename)


def exportPath(viewer, robot, problem, pathId, stepsize, filename):
    """
    Export object positions for a path

    \\param robot the considered robot,
    \\param problem the problem associated with the path computed for the robot
    \\param stepsize increment along the path
    \\param pathId if of the considered path
    \\param filename name of the output file where to save the output
    """
    length = problem.pathLength(pathId)
    t = 0
    tau = []
    dt = stepsize / length
    while t < length:
        q = problem.configAtParam(pathId, t)
        tau.append(q)
        t += dt
    exportStates(viewer, robot, tau, filename)
