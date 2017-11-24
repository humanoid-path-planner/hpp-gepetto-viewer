MACRO(INSTALL_PYTHON_FILES MODULE FILES)
    FOREACH(PYFILE ${FILES})
        PYTHON_INSTALL_ON_SITE(${MODULE} ${PYFILE})
    ENDFOREACH()
ENDMACRO()

INSTALL_PYTHON_FILES("hpp/gepetto"
    path_player.py
    viewer.py
    viewer_factory.py
    __init__.py)

INSTALL_PYTHON_FILES("hpp/gepetto/blender"
  exportmotion.py
  __init__.py)

INSTALL_PYTHON_FILES("hpp/gepetto/manipulation"
    viewer.py
    viewer_factory.py
    __init__.py)

INSTALL_PYTHON_FILES("hpp/gepetto/gui"
    path_graph.py
    __init__.py)
