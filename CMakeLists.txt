#
# Copyright (c) 2013 CNRS
# Authors: Florent Lamiraux
#
#
# This file is part of kpp-interface
# kpp-interface is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# kpp-interface is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# kpp-interface  If not, see
# <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 2.6)

SET(PROJECT_NAME kpp-interface)
SET(PROJECT_DESCRIPTION "Graphical interface for Humanoid Path Planner")
SET(PROJECT_URL https://github.com:laas/kpp-interface)

SET(${PROJECT_NAME}_HEADERS
  include/kpp/interface/command-init.hh
  include/kpp/interface/command-open-file.hh
  include/kpp/interface/command-planner-panel.hh
  include/kpp/interface/command-start-corbaserver.hh
  include/kpp/interface/interface.hh
  include/kpp/interface/planner-panel-controller.hh
  include/kpp/interface/planner-panel.hh
  include/kpp/interface/graphic-roadmap-delegate.hh
  include/kpp/interface/graphic-roadmap.hh
  )

SET(CXX_DISABLE_WERROR ON)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/kineo.cmake)

SETUP_PROJECT()

ADD_REQUIRED_DEPENDENCY("Kite >= 2.07")
ADD_REQUIRED_DEPENDENCY("hpp-util >= 0.6")
ADD_REQUIRED_DEPENDENCY("hpp-core >= 2.6")
ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 2.1")
ADD_REQUIRED_DEPENDENCY("hpp-kwsplus >= 2.3")

SET(PKG_CONFIG_LIBS "")

SET(KINEO_ADDON_SOURCES
  src/kppInterface.cpp
  src/kppCommandStartCorbaServer.cpp
  src/kppCommandPlannerPanel.cpp
  src/kppCommandInit.cpp
  src/kppPlannerPanelController.cpp
  src/kppPlannerPanel.cpp
  src/kwsGraphicRoadmapDelegate.cpp
  src/kwsGraphicRoadmap.cpp
  )
KINEO_ADDON(${PROJECT_NAME})
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} hpp-corbaserver)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} Kite)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
