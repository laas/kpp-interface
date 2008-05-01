/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
  Developed by Florent Lamiraux (LAAS-CNRS)

*/


#ifndef KPPINTERFACE_H
#define KPPINTERFACE_H

/**
  \brief Main class of package kppInterface
 */

/*****************************************
 INCLUDES
*******************************************/

#include "KineoModuleManager/kppModuleInterface.h"
#include "KineoGUI/kppGUIModuleInterface.h"
#include "KineoController/kppUICommand.h"

#include "hppCorbaServer/hppciServer.h"
#include "kppInterface/kwsGraphicRoadmap.h"
#include "kppInterface/kppCommandPlannerPanel.h"

#include <deque>

KIT_PREDEF_CLASS(CkppInterface);


/*****************************************
 CLASS
*******************************************/

class CkppInterface : public CkppModuleInterface, public CkppGUIModuleInterface
{
  
public:
  
  static CkppInterfaceShPtr create();

  virtual ~CkppInterface();
  
  // methods inherited from CkppModuleInterface
  
  virtual ktStatus activate();
  
  virtual std::string name() const {return std::string("kppInterface");};
  
  virtual void getMenuUICommandLists(const CkppMainWindowUICommandFactoryConstShPtr& inCommandFactory,
				     std::vector<CkppUICommandListShPtr> & outMenuCommandListVector);
  
  /**
     \brief Get pointer to ChppPlanner object.
  */
  ChppPlanner* hppPlanner() { return attHppPlanner;};

  /**
     \brief Command to start corba server()
  */
  ktStatus startCorbaServer();

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

  CkppUICommandShPtr attCommandInitBase;

  CkppUICommandShPtr attCommandSetConfigBase;

protected:
  
  CkppInterface(ChppPlanner *inHppPlanner);

  /// command to start corba server
  CkppUICommandShPtr attStartCorbaServerCommand;

  CkppUICommandShPtr attCommandPlannerPanel;

  ChppPlanner *attHppPlanner;

  /// Object that implements Corba interface of module "Stochastic Environment Path Planning"
  ChppciServer *attHppCorbaServer;
  
  bool corbaServerRunning;


protected:

  /**
     \name Treat Notifications
     @{
  */

  /**
     \brief call ChppPlanner::addObstacle() if a new geometry is added 
     \param inNotification notification from the interface
   */

  virtual void insertChild(const CkitNotificationConstShPtr& inNotification);
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
     \param inNotification notification sent by ChppPlanner object.

     This notification is sent each time a roadmap builder is destroyed.
  */
  virtual void hppRemoveRoadmapBuilder(const CkitNotificationConstShPtr& inNotification);

  /**
     \brief Insert a graphic roadmap in the interface
     \param inNotification notification sent by ChppPlanner object.

     This method is called each time a new roadmap builder is created 
     the roadmap of which is required to be displayed.
  */
  virtual void hppAddGraphicRoadmap(const CkitNotificationConstShPtr& inNotification);

  /**
     \brief Called when the interface has nothing to do.
  */
  virtual void onIdle(const CkitNotificationConstShPtr& inNotification);

 /**
     @}
  */

 private :

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
     \brief Draws the entire roadmap when the associated notification is received
     \param inNotification Received notification. 

     \note the notification must be sent with a Shared pointer on a CkwsRoadmapBuilder object in order
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
