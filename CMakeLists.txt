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

cmake_minimum_required(VERSION 3.22)
project(hpp_gepetto_viewer VERSION 8.0.0 LANGUAGES C CXX)

set(PROJECT_USE_CMAKE_EXPORT TRUE)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
    message(STATUS "JRL cmakemodules found in 'cmake/' git submodule")
else()
    find_package(jrl-cmakemodules QUIET CONFIG)
    if(jrl-cmakemodules_FOUND)
        get_property(
            JRL_CMAKE_MODULES
            TARGET jrl-cmakemodules::jrl-cmakemodules
            PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        )
        message(
            STATUS
            "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}"
        )
    else()
        message(STATUS "JRL cmakemodules not found. Let's fetch it.")
        include(FetchContent)
        FetchContent_Declare(
            "jrl-cmakemodules"
            GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git"
        )
        FetchContent_MakeAvailable("jrl-cmakemodules")
        FetchContent_GetProperties(
            "jrl-cmakemodules"
            SOURCE_DIR JRL_CMAKE_MODULES
        )
    endif()
endif()

find_package(ament_cmake REQUIRED)

# jrl helpers (sans compute_project_args qui appellerait project() une 2e fois)
include("${JRL_CMAKE_MODULES}/python.cmake")
project(${PROJECT_NAME} ${PROJECT_ARGS})

option(USE_HPP_PYTHON "Add dependency to hpp-python" ON)
option(USE_CORBA "Enable dependency to hpp-manipulation-corba" OFF)
option(BUILD_HPP_RVIZ_PKGS "Build hpp_rviz (messages + RViz2 plugin)" OFF)

if(USE_CORBA)
    find_package(hpp-corbaserver REQUIRED)
    find_package(gepetto-viewer QUIET)
endif()

if(USE_HPP_PYTHON)
    find_package("hpp-python" REQUIRED)
endif()

if(NOT FINDPYTHON_ALREADY_CALLED)
    findpython()
endif()

if(BUILD_HPP_RVIZ_PKGS)
    find_package(rosidl_default_generators REQUIRED)
    find_package(std_msgs REQUIRED)

    set(HPP_RVIZ_MSG_TARGET "${PROJECT_NAME}")
    rosidl_generate_interfaces(${HPP_RVIZ_MSG_TARGET}
        src/hpp_rviz/msg/PathInfo.msg
        src/hpp_rviz/msg/PinocchioJoint.msg
        src/hpp_rviz/msg/HppVectorConfiguration.msg
        src/hpp_rviz/msg/HppWaypoint.msg
        DEPENDENCIES std_msgs
    )
    ament_export_dependencies(rosidl_default_runtime)

    rosidl_get_typesupport_target(cpp_typesupport_target
        ${HPP_RVIZ_MSG_TARGET} rosidl_typesupport_cpp
    )

    find_package(ament_cmake_ros REQUIRED)
    find_package(pluginlib REQUIRED)
    find_package(rviz_common REQUIRED)
    find_package(rviz_default_plugins REQUIRED)
    find_package(rviz_rendering REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)

    set(HPP_RVIZ_PLUGIN_TARGET "${PROJECT_NAME}_plugin")

    set(MOC_FILE_TMP
        src/hpp_rviz/include/hpp/display/trajectory.hpp
        src/hpp_rviz/include/hpp/panel/trajectory_slider.hpp
        src/hpp_rviz/include/hpp/doubleSlider.hpp
        src/hpp_rviz/include/hpp/tool/waypoint.hpp
        src/hpp_rviz/include/hpp/tool/interactiveWaypoint.hpp
        src/hpp_rviz/include/hpp/display/waypoint.hpp
        src/hpp_rviz/include/hpp/display/WaypointProperty.hpp
    )

    if(QT_VERSION_MAJOR EQUAL 6)
        qt6_wrap_cpp(MOC_FILES ${MOC_FILE_TMP})
    else()
        qt5_wrap_cpp(MOC_FILES ${MOC_FILE_TMP})
    endif()

    add_library(
        ${HPP_RVIZ_PLUGIN_TARGET}
        src/hpp_rviz/src/display/trajectory.cpp
        src/hpp_rviz/src/panel/trajectory_slider.cpp
        src/hpp_rviz/src/panel/ui.cpp
        src/hpp_rviz/src/panel/communication.cpp
        src/hpp_rviz/src/tool/waypoint.cpp
        src/hpp_rviz/src/display/waypoint.cpp
        ${MOC_FILES}
    )

    target_link_libraries(
        ${HPP_RVIZ_PLUGIN_TARGET}
        PRIVATE
            pluginlib::pluginlib
            rviz_common::rviz_common
            rviz_default_plugins::rviz_default_plugins
            rviz_rendering::rviz_rendering
            ${sensor_msgs_TARGETS}
            ${std_msgs_TARGETS}
            ${visualization_msgs_TARGETS}
            ${cpp_typesupport_target}
    )

    install(
        TARGETS ${HPP_RVIZ_PLUGIN_TARGET}
        EXPORT export_${HPP_RVIZ_PLUGIN_TARGET}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    install(DIRECTORY src/hpp_rviz/include/ DESTINATION include)
    install(FILES rviz_common_plugins.xml DESTINATION share/${PROJECT_NAME})
    install(
        FILES src/hpp_rviz/icons/Trajectory.png src/hpp_rviz/icons/Waypoint.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
    )
    ament_export_include_directories(include)
    ament_export_targets(export_${HPP_RVIZ_PLUGIN_TARGET})
    pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)
else()
    add_library(${PROJECT_NAME} INTERFACE)
    install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        DESTINATION lib
    )
endif()

add_subdirectory(src)

ament_package()
