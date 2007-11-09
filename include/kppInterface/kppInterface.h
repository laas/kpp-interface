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
  
  virtual void getMenuUICommandLists(const CkppMainWindowUICommandFactoryConstShPtr& i_commandFactory,
				     std::vector<CkppUICommandListShPtr> & o_menuCommandListVector);
  
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
     \param i_graphic_roadmap Graphic roadmap that should be displayed.
     \param i_isRealTimeUpdated Says if the graphic roadmap will be built at run time or not

     \note If you want to see the roadmap building at run-time, you must add a CkppProgressDelegate to your roadmap builder.

     \return The graphic roadmap rank in the vector.
  */
  unsigned int addGraphicRoadmap(CkwsGraphicRoadmapShPtr i_graphic_roadmap, bool i_isRealTimeUpdated = false);

  /**
     \brief Removes the graphic roadmap at the given rank in the vector
     \param i_rank Rank of the graphic roadmap to delete

     \note If you call this function without arguments (or with argument -1) it will remove all graphic roadmaps for this interface
  */
  void removeGraphicRoadmap( int i_rank = -1);

  /**
     \brief Shows the graphic roadmap of the given rank.
     \param i_rank Rank of the graphic roadmap to show

     \note If the roadmap is built at run-time, it will be automatically displayed.
     In this case, you must notify the addition of an edge by yourself. See class \link #CkwsGraphicRoadmap CkwsGraphicRoadmap\endlink for more details.
  */
  void showRoadmap(unsigned int i_rank);

  /**
     \brief Hides the graphic roadmap of the given rank.
     \param i_rank Rank of the graphic roadmap to show
  */
  void hideRoadmap(unsigned int i_rank);

protected:
  
  CkppInterface();

  /// command to start corba server
  CkppUICommandShPtr attStartCorbaServerCommand;

  ChppPlanner *attHppPlanner;

  /// Object that implements Corba interface of module "Stochastic Environment Path Planning"
  ChppciServer *attHppCorbaServer;
  
  bool corbaServerRunning;
  
  //vector of graphic roadmaps
  std::vector<CkwsGraphicRoadmapShPtr> m_graphic_roadmaps;

public:

  /**
     \name Treat Notifications
     @{
  */

  /**
     \brief Add a Robot (device) to the interface
     \param i_notification :
     
     KPP Default actions when inserting a device in the device node are disabled to get the same behavior with or without interface.
  */								
  virtual void hppAddRobot(const CkitNotificationConstShPtr& i_notification);
 /**
     \brief Add a Path to the interface
     \param i_notification :
  */	
  virtual void hppAddPath(const CkitNotificationConstShPtr& i_notification);
 /**
     \brief Add a Obstacle to the interface
     \param i_notification :
  */	
  virtual void hppAddObstacle(const CkitNotificationConstShPtr& i_notification);
 
 /**
     \brief Insert a list of obstacle to the interface
     \param i_notification :
  */	
  void hppSetObstacleList(const CkitNotificationConstShPtr& i_notification);

  /**
     \brief Called when the interface has nothing to do.
  */
  void onIdle(const CkitNotificationConstShPtr& i_notification);

 /**
     @}
  */

 private :

  /**
     \brief Draws the entire roadmap when the associated notification is received
     \param i_notification Received notification. 

     \note the notification must be sent with a Shared pointer on a CkwsRoadmapBuilder object in order
     to know which roadmap have to be updated.

  */
  void addRoadmap(const CkitNotificationConstShPtr& i_notification);

  
  /*
     \brief Draws the roadmap at its current state when the associated notification is received
     \param i_notification Received notification.

     \note the notification must be sent with a Shared pointer on a CkwsRoadmapBuilder object in order
     to know which roadmap have to be updated.
  */
  //void addEdge(const CkitNotificationConstShPtr& i_notification);

  /**
     \brief Removes all graphic roadmaps from the vector of roadmaps
     \param i_notification received notification
  */
  void removeAllRoadmaps(const CkitNotificationConstShPtr& i_notification);

};

// function use to compile the KPP license
extern "C" KPP_ADDON_API int initializeModule(CkppModuleInterfaceShPtr& o_moduleInterface);

#endif
