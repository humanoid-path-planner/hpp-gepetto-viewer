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

MACRO(INSTALL_PYTHON_FILES MODULE FILES)
    FOREACH(PYFILE ${FILES})
        PYTHON_INSTALL_ON_SITE(${MODULE} ${PYFILE})
    ENDFOREACH()
ENDMACRO()

PYTHON_INSTALL_ON_SITE ("hpp/gepetto" path_player.py)
PYTHON_INSTALL_ON_SITE ("hpp/gepetto" viewer.py)
PYTHON_INSTALL_ON_SITE ("hpp/gepetto" viewer_factory.py)
PYTHON_INSTALL_ON_SITE ("hpp/gepetto" __init__.py)

PYTHON_INSTALL_ON_SITE ("hpp/gepetto/blender" exportmotion.py)
PYTHON_INSTALL_ON_SITE ("hpp/gepetto/blender" __init__.py)

PYTHON_INSTALL_ON_SITE ("hpp/gepetto/manipulation" viewer.py)
PYTHON_INSTALL_ON_SITE ("hpp/gepetto/manipulation" viewer_factory.py)
PYTHON_INSTALL_ON_SITE ("hpp/gepetto/manipulation" __init__.py)

PYTHON_INSTALL_ON_SITE ("hpp/gepetto/gui" path_graph.py)
PYTHON_INSTALL_ON_SITE ("hpp/gepetto/gui" __init__.py)
