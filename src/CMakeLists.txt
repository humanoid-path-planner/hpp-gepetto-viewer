#!/usr/bin/env python

# Copyright (c) 2017 CNRS
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

MACRO(INSTALL_PYTHON_FILES MODULE)
    SET(FIRST TRUE)
    FOREACH(PYFILE ${ARGV})
        IF(${FIRST})
          SET(FIRST FALSE)
        ELSE()
          PYTHON_INSTALL_ON_SITE(${MODULE} ${PYFILE})
        ENDIF()
    ENDFOREACH()
ENDMACRO()

INSTALL_PYTHON_FILES ("hpp/gepetto"
  path_player.py
  viewer.py
  viewer_factory.py
  __init__.py)

INSTALL_PYTHON_FILES ("hpp/gepetto/blender"
  exportmotion.py
  __init__.py)

INSTALL_PYTHON_FILES ("hpp/gepetto/manipulation" viewer.py
  viewer_factory.py
  __init__.py)

INSTALL_PYTHON_FILES ("hpp/gepetto/gui" path_graph.py
  __init__.py)

IF(gepetto-viewer_FOUND AND EXISTS ${GEPETTO_VIEWER_CMAKE_PLUGIN})
  GEPETTO_GUI_PYPLUGIN ("hpp/gepetto/gui/path_graph.py")
ENDIF()
