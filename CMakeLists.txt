#!/usr/bin/env python

# Copyright (c) 2014 CNRS
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

#dependency: python-rospkg

cmake_minimum_required(VERSION 2.8.3)

SET(PROJECT_NAME hpp-gepetto-viewer)
set(PROJECT_DESCRIPTION "Display of hpp robots and obstacles in gepetto-viewer")

include(cmake/hpp.cmake)
include(cmake/python.cmake)


# Tells pkg-config to read cmake_plugin from pkg config file.
LIST(APPEND PKG_CONFIG_ADDITIONAL_VARIABLES cmake_plugin)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

FINDPYTHON()

ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 4")
ADD_OPTIONAL_DEPENDENCY("gepetto-viewer")
IF(GEPETTO_VIEWER_FOUND AND EXISTS ${GEPETTO_VIEWER_CMAKE_PLUGIN})
  INCLUDE(${GEPETTO_VIEWER_CMAKE_PLUGIN})
ENDIF()

ADD_SUBDIRECTORY(src)
