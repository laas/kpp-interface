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

# Reinitialize directory where libs are installed since for Kineo addon, this
# directory is non standard: lib/modules/${UNAME_N}
SET(PKG_CONFIG_LIBS "")

SET(KINEO_ADDON_SOURCES
  kppInterface.cpp
  kppCommandStartCorbaServer.cpp
  kppCommandPlannerPanel.cpp
  kppCommandInit.cpp
  kppPlannerPanelController.cpp
  kppPlannerPanel.cpp
  kwsGraphicRoadmapDelegate.cpp
  kwsGraphicRoadmap.cpp
  )
KINEO_ADDON(${PROJECT_NAME})
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} hpp-corbaserver)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} Kite)
