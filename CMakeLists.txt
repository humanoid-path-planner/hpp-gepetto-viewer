#!/usr/bin/env python

# Copyright (c) 2014, 2020 CNRS Author: Florent Lamiraux, Guilhem Saurel
#
# This file is part of hpp-gepetto-viewer. hpp-gepetto-viewer is free software:
# you can redistribute it and/or modify it under the terms of the GNU Lesser
# General Public License as published by the Free Software Foundation, either
# version 3 of the License, or (at your option) any later version.
#
# hpp-gepetto-viewer is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Lesser Public License
# for more details.  You should have received a copy of the GNU Lesser General
# Public License along with hpp-gepetto-viewer.  If not, see
# <http://www.gnu.org/licenses/>.

# dependency: python-rospkg

cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME hpp-gepetto-viewer)
set(PROJECT_DESCRIPTION "Display of hpp robots and obstacles in gepetto-viewer")

set(PROJECT_USE_CMAKE_EXPORT TRUE)

include(cmake/hpp.cmake)
include(cmake/python.cmake)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

add_project_dependency("hpp-corbaserver" REQUIRED)
add_project_dependency("gepetto-viewer")

if(NOT FINDPYTHON_ALREADY_CALLED)
  findpython()
endif()

add_library(${PROJECT_NAME} INTERFACE)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

add_subdirectory(src)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
