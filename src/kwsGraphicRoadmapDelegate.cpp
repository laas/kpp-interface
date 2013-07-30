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

/*****************************************
 INCLUDES
*******************************************/
#include <iostream>
#include <ctime>
#include <string>
#include <sstream>
#include <cstdio>

#include "kpp/interface/graphic-roadmap-delegate.hh"
#include "KineoGUI/kppController.h"
#include "KineoGUI/kppMainWindowController.h"
#include "KineoGUI/kppMainFrame.h"
#include "KineoWorks2/kwsNode.h"

#include "kwsIO/kwsioConfig.h"

// Select verbosity at configuration by setting CXXFLAGS="... -DDEBUG=[1 or 2]"
#if DEBUG==2
#define ODEBUG2(x) std::cout << "CkwsGraphicRoadmapDelegate:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CkwsGraphicRoadmapDelegate:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "CkwsGraphicRoadmapDelegate:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

//using namespace std;

const CkitNotification::TType 
CkwsGraphicRoadmapDelegate::DID_MODIFY_THE_ROADMAP(CkitNotification::makeID());

const std::string CkwsGraphicRoadmapDelegate::ROADMAPBUILDER_KEY("ROADMAPBUILER");

// Counter of existing objects of this class
unsigned int CkwsGraphicRoadmapDelegate::nbObjects = 0;

KIT_PREDEF_CLASS( CkppComponent );

//constructor
CkwsGraphicRoadmapDelegate::CkwsGraphicRoadmapDelegate(){
#if DEBUG==2
  nbObjects++;  
  std::cout << "CkwsGraphicRoadmapDelegate: constructor: nb existing objects = " 
	    << nbObjects << std::endl;
#endif
}

//Destructor
CkwsGraphicRoadmapDelegate::~CkwsGraphicRoadmapDelegate()
{
#if DEBUG==2
  nbObjects--;
  std::cout << "CkwsGraphicRoadmapDelegate: constructor: nb existing objects = " 
	    << nbObjects << std::endl;
#endif
}

//what the delegate must do before the builder starts solving the problem
void CkwsGraphicRoadmapDelegate::builderWillStartBuilding (const CkwsRoadmapBuilderShPtr &inRdmBuilder, 
							   CkwsPathShPtr &inOutPath){

  nbSuccessfulShoots=0;
  /* 
     Store weak pointer to roadmap builder
  */
  attRoadmapBuilderWkPtr = inRdmBuilder;
  CkwsRdmBuilderDelegate::builderWillStartBuilding (inRdmBuilder,inOutPath);
}

//what the delegate must do after the builder finished solving the problem
void CkwsGraphicRoadmapDelegate::builderDidFinishBuilding (const CkwsRoadmapBuilderShPtr &inRdmBuilder, 
							   CkwsPathShPtr &inOutPath, ktStatus inSuccess)
{
  CkitNotificationShPtr notification = 
    CkitNotification::createWithShPtr<CkwsRoadmapBuilder>(CkppPlanPathCommand::DID_FINISH_BUILDING, 
							  inRdmBuilder);
    CkitNotificator::defaultNotificator()->notify(notification);
    CkwsRdmBuilderDelegate::builderDidFinishBuilding(inRdmBuilder, inOutPath, inSuccess);
}

//allows user to stop the builder (called repeatedly during the process).
bool CkwsGraphicRoadmapDelegate::plannerShouldStopPlanning (const CkwsRoadmapBuilderConstShPtr &inRdmBuilder){


  //CkppController::yield();
  return CkwsRdmBuilderDelegate::plannerShouldStopPlanning (inRdmBuilder);

}

//allows user to prevent builder from expanding a node/configuration
bool CkwsGraphicRoadmapDelegate::builderShouldExploreTowards (const CkwsRoadmapBuilderConstShPtr &inRdmBuilder, 
							      const CkwsConfig &inConfig){

  //CkppController::yield();
  return CkwsRdmBuilderDelegate::builderShouldExploreTowards (inRdmBuilder, inConfig);

}

//what the delegate must do after the builder adds a node.
void CkwsGraphicRoadmapDelegate::builderDidAddNode (const CkwsRoadmapBuilderConstShPtr &inRdmBuilder, 
						    const CkwsNodeConstShPtr &inNode)
{
  ODEBUG2(":builderDidAddNode, config = " << inNode->config());
  /**

     Send a notification with const roadmap builder as argument 
     This notification is caught by CkppInterface.

  */
  CkitNotificationShPtr notification = 
    CkitNotification::create(CkwsGraphicRoadmapDelegate::DID_MODIFY_THE_ROADMAP);
  notification->constShPtrValue<CkwsRoadmapBuilder>(ROADMAPBUILDER_KEY, inRdmBuilder);
  CkitNotificator::defaultNotificator()->notify(notification);

  /*

    Send another notification caught I do not know by whom

  */

  /*
    Retrieve non constant shared pointer to roadmap builder
    This is a horrible hack
  */
  CkwsRoadmapBuilderShPtr roadmapBuilder = attRoadmapBuilderWkPtr.lock();
  notification = 
    CkitNotification::createWithShPtr<CkwsRoadmapBuilder>(CkppPlanPathCommand::DID_ADD_NODE_TO_ROADMAP, 
							  roadmapBuilder);
  CkitNotificator::defaultNotificator()->notify(notification);
  CkppController::yield();
  
  CkwsRdmBuilderDelegate::builderDidAddNode (inRdmBuilder,inNode);
}

//what the delegate must do after the builder adds an edge.
void CkwsGraphicRoadmapDelegate::builderDidAddEdge (const CkwsRoadmapBuilderConstShPtr &inRdmBuilder, 
						    const CkwsEdgeConstShPtr &inEdge){

  ODEBUG2(":builderDidAddEdge called");
  /**

     Send a notification with const roadmap builder as argument 
     This notification is caught by CkppInterface.

  */
  CkitNotificationShPtr notification = 
    CkitNotification::create(CkwsGraphicRoadmapDelegate::DID_MODIFY_THE_ROADMAP);
  notification->constShPtrValue<CkwsRoadmapBuilder>(ROADMAPBUILDER_KEY, inRdmBuilder);
  CkitNotificator::defaultNotificator()->notify(notification);

  /*

    Send another notification caught I do not know by whom

  */

  /*
    Retrieve non constant shared pointer to roadmap builder
    This is a horrible hack
  */
  CkwsRoadmapBuilderShPtr roadmapBuilder = attRoadmapBuilderWkPtr.lock();
  notification = 
    CkitNotification::createWithShPtr<CkwsRoadmapBuilder>(CkppPlanPathCommand::DID_ADD_EDGE_TO_ROADMAP, 
							  roadmapBuilder);
  CkitNotificator::defaultNotificator()->notify(notification);
  CkppController::yield();

  CkwsRdmBuilderDelegate::builderDidAddEdge (inRdmBuilder,inEdge);
}

//what the delegate must do after the builder modifies the path.
void CkwsGraphicRoadmapDelegate::builderDidModifyPath (const CkwsRoadmapBuilderConstShPtr &inRdmBuilder, 
						       const CkwsPathConstShPtr &i_path)
{

  //CkppController::yield();
  CkwsRdmBuilderDelegate::builderDidModifyPath (inRdmBuilder,i_path);

}
