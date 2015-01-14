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
project(hpp-gepetto-viewer)
include(cmake/base.cmake)
include(cmake/python.cmake)

set(PROJECT_DESCRIPTION "Display of hpp robots and obstacles in gepetto-viewer")
set(PROJECT_URL "")

findpython()
setup_project()

add_required_dependency(hpp-corbaserver >= 3)

install(FILES
  src/hpp/gepetto/path_player.py
  src/hpp/gepetto/viewer.py
  src/hpp/gepetto/fakeviewer.py
  src/hpp/gepetto/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/gepetto
  )

install(FILES
  src/hpp/gepetto/manipulation/viewer.py
  src/hpp/gepetto/manipulation/fakeviewer.py
  src/hpp/gepetto/manipulation/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/gepetto/manipulation
  )

setup_project_finalize()
