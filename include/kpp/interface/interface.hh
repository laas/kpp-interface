//
// Copyright (c) 2005, 2006, 2007, 2008, 2009, 2010, 2011, 2012, 2013 CNRS
// Authors: David Flavigne, Thomas Moulard and Florent Lamiraux
//
// This file is part of kpp-interface
// kpp-interface is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// kpp-interface is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// kpp-interface  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef KPPINTERFACE_H
#define KPPINTERFACE_H

/**
  \brief Main class of package kppInterface
 */

/*****************************************
 INCLUDES
*******************************************/

#include <deque>
#include <map>

#include "KineoModuleManager/kppModuleInterface.h"
#include "KineoGUI/kppGUIModuleInterface.h"
#include "KineoView/kppViewModuleInterface.h"

#include <hpp/util/kitelab.hh>

#include <hpp/corbaserver/fwd.hh>
#include <hpp/core/fwd.hh>

HPP_KIT_PREDEF_CLASS(CkitNotification);
HPP_KIT_PREDEF_CLASS(CkwsRoadmap);
HPP_KIT_PREDEF_CLASS(CkppUICommand);
HPP_KIT_PREDEF_CLASS(CkppMainWindowController);
HPP_KIT_PREDEF_CLASS(CkwsGraphicRoadmap);
HPP_KIT_PREDEF_CLASS(CkppViewGeneral);

/*****************************************
 CLASS
*******************************************/

HPP_KIT_PREDEF_CLASS(CkppInterface);

class CkppInterface : public CkppModuleInterface, public CkppGUIModuleInterface, public CkppViewModuleInterface
{

public:

  static CkppInterfaceShPtr create();

  virtual ~CkppInterface();

  /**
     \brief Initialize the module.

     This methods implements CkppModuleInterface::activate.
     This method subscribes to notifications triggered in hppCore.
  */
  virtual ktStatus activate();

  /**
     \brief Name of the module

     This methods implements CkppModuleInterface::name.
  */
  virtual std::string name() const {return std::string("kppInterface");};

  /**
     \brief Add a new menu in KPP

     This methods implements CkppGUIModuleInterface::getMenuUICommandLists.
  */
  virtual void getMenuUICommandLists(const CkppMainWindowUICommandFactoryConstShPtr& inCommandFactory,
				     std::vector<CkppUICommandListShPtr> & outMenuCommandListVector);

  /**
     \brief Set pointer to hpp::core::Planner object.
  */
  void hppPlanner(hpp::core::Planner* inHppPlanner) {attHppPlanner = inHppPlanner;};

  /**
     \brief Get pointer to hpp::core::Planner object.
  */
  hpp::core::Planner* hppPlanner() { return attHppPlanner;};

  /**
     \brief Command to start corba server()
  */
  ktStatus startCorbaServer();

  /**
     \brief Access to the main window controller
  */
  CkppMainWindowControllerShPtr mainWindowController();

  /**
     \brief Adds a graphical representation of a roadmap in the interface
     \param inGraphicRoadmap Graphic roadmap that should be displayed.
     \param inIsRealTimeUpdated Says if the graphic roadmap will be built at run time or not

     \note If you want to see the roadmap building at run-time, you must add a CkppProgressDelegate to your roadmap builder.

     \return The graphic roadmap rank in the vector.
  */
  unsigned int addGraphicRoadmap(CkwsGraphicRoadmapShPtr inGraphicRoadmap, bool inIsRealTimeUpdated = false);

  /**
     \brief Removes the graphic roadmap at the given rank in the vector
     \param inRank Rank of the graphic roadmap to delete

     \note If you call this function without arguments (or with argument -1) it will remove all graphic roadmaps for this interface
  */
  void removeGraphicRoadmap(int inRank = -1);

  /**
     \brief Shows the graphic roadmap of the given rank.
     \param inRank Rank of the graphic roadmap to show

     \note If the roadmap is built at run-time, it will be automatically displayed.
     In this case, you must notify the addition of an edge by yourself. See class \link #CkwsGraphicRoadmap CkwsGraphicRoadmap\endlink for more details.
  */
  void showRoadmap(unsigned int inRank);

  /**
     \brief Hides the graphic roadmap of the given rank.
     \param inRank Rank of the graphic roadmap to show
  */
  void hideRoadmap(unsigned int inRank);
	
	/**
     \brief updates the viewGeneral
  */
	virtual void initializeViewGeneral (const CkppViewGeneralShPtr &i_viewGeneral);
	
	CkppViewGeneralShPtr viewGeneral()
	{
		return attViewGeneral;
	}

  CkppUICommandShPtr attCommandInitBase;

  CkppUICommandShPtr attCommandSetConfigBase;

protected:

  CkppInterface(hpp::core::Planner *inHppPlanner);

  /// command to start corba server
  CkppUICommandShPtr attStartCorbaServerCommand;

  CkppUICommandShPtr attCommandPlannerPanel;
  CkppUICommandShPtr attCommandOpenFile;

  /// Object that implements Corba interface of module "Stochastic Environment Path Planning"
  hpp::corbaServer::Server* attHppCorbaServer;

  bool corbaServerRunning;

  /**
     \name Acess to main window controller
     @{
  */

  /**
     \brief Get pointer to main window controller

     To get access to the model tree and other entities of the interface, we need to store
     a pointer to the main window controller. This is done in method CkppInterface::getMenuUICommandLists().
  */
  void getMainWindowController(const CkppMainWindowUICommandFactoryConstShPtr& inCommandFactory);


protected:

  /**
     \name Treat Notifications
     @{
  */

  /**
     \brief Add a Robot (device) to the interface
     \param inNotification :

     KPP Default actions when inserting a device in the device node are disabled to get the same behavior with or without interface.
  */
  virtual void hppAddRobot(const CkitNotificationConstShPtr& inNotification);
 /**
     \brief Add a Path to the interface
     \param inNotification :
  */
  virtual void hppAddPath(const CkitNotificationConstShPtr& inNotification);
 /**
     \brief Add a Obstacle to the interface
     \param inNotification :
  */
  virtual void hppAddObstacle(const CkitNotificationConstShPtr& inNotification);

 /**
     \brief Insert a list of obstacle to the interface
     \param inNotification :
  */
  virtual void hppSetObstacleList(const CkitNotificationConstShPtr& inNotification);

  /**
     \brief Delete a roadmap builder
     \param inNotification notification sent by hpp::core::Planner object.

     This notification is sent each time a roadmap builder is destroyed.
  */
  virtual void hppRemoveRoadmapBuilder(const CkitNotificationConstShPtr& inNotification);

  /**
     \brief Insert a graphic roadmap in the interface
     \param inNotification notification sent by hpp::core::Planner object.

     This method is called each time a new roadmap builder is created
     the roadmap of which is required to be displayed.
  */
  virtual void hppAddGraphicRoadmap(const CkitNotificationConstShPtr& inNotification);

  /// Refresh the view after a robot configuration has changed.
  virtual void hppSetCurrentConfig(const CkitNotificationConstShPtr&
				   inNotification);
  /**
     \brief Called when the interface has nothing to do.
  */
  virtual void onIdle(const CkitNotificationConstShPtr& inNotification);

 /**
     @}
  */

  /**
     \brief Initialize interface
     \param inKppInterface Weak pointer to itself

     This function basically stores a shared pointer to itself in CkppInterface object.
  */
  ktStatus init(const CkppInterfaceWkPtr& inKppInterface);

 private :
	 
	 CkppViewGeneralShPtr attViewGeneral;

  /**
     \brief hpp::core::Planner object associated to the interface

     Given at construction.
  */
  hpp::core::Planner *attHppPlanner;

  /**
     \brief Store weak pointer to main window controller
  */
  CkppMainWindowControllerWkPtr attMainWindowControllerWkPtr;

  /**
     \brief Weak pointer to itself
  */
  CkppInterfaceWkPtr attWeakPtr;

  /**
     \brief Store graphic existing roadmaps
  */
  std::deque<CkwsGraphicRoadmapShPtr> attGraphicRoadmaps;

  /**
     \brief Mapping between CkwsRoadmap and CkwsGraphicRoadmap.

     This mapping is used when a roadmap is deleted. It enables the object to retrieve the
     corresponding graphic roadmap if any.
  */
  std::map <CkwsRoadmapShPtr, CkwsGraphicRoadmapShPtr> attRoadmapMapping;

  /**
     \brief Retrieve rank in CkppInterface::attGraphicRoadmaps of given roadmap

     \param inRoadmap A shared pointer to a standard roadmap
     \retval outRank the rank of graphic roadmap corresponding to CkppInterface::attGraphicRoadmaps if any.
     \return true if the input roadmap corresponds to a graphic roadmap, false otherwise.

     This function tries to solve the following equation:

     attGraphicRoadmaps[outRank] = attRoadmapMapping[inRoadmap]

  */
  bool isRoadmapStoredAsGraphic(const CkwsRoadmapShPtr& inRoadmap, unsigned int& outRank);

  /**
     \brief Draws the entire roadmap upon notification.
     \param inNotification Received notification.

     \note notification CkppPlanPathCommand::DID_FINISH_BUILDING must be sent with a Shared pointer on a CkwsRoadmapBuilder object in order
     to know which roadmap have to be updated.

  */
  void graphicRoadmapHasBeenModified(const CkitNotificationConstShPtr& inNotification);


  /**
     \brief Removes all graphic roadmaps and all ChppProblem objects

     \param inNotification received notification
  */
  void removeAllRoadmapsAndProblems(const CkitNotificationConstShPtr& inNotification);

};

// function use to compile the KPP license
extern "C" KPP_ADDON_API int initializeModule(CkppModuleInterfaceShPtr& o_moduleInterface);

#endif
