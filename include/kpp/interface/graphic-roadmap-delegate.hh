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

#ifndef KWS_GRAPHIC_ROADMAP_DELEGATE_H
#define KWS_GRAPHIC_ROADMAP_DELEGATE_H


/*****************************************
 INCLUDES
*******************************************/

#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoWorks2/kwsRdmBuilderDelegate.h"
#include "KineoWorks2/kwsDiffusingRdmBuilder.h"
#include "KineoWorks2/kwsDevice.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoWorks2/kwsRoadmap.h"
#include "KineoGUI/kppProgressDelegate.h"
#include "KineoController/kppPlanPathCommand.h"
#include "KineoUtility/kitNotification.h"
#include "KineoUtility/kitNotificator.h"

#include <iostream>
#include <string>
#include <sstream>
#include <ctime>

KIT_PREDEF_CLASS(CkwsGraphicRoadmapDelegate);

using namespace std ;

/**
   \addtogroup kppInterface_graphic_Roadmap
   @{
*/

/*************************************
CLASS
**************************************/
/**
   \brief Implements a class which allows to follow a roadmapbuilder evolution

   see class CkwsRdmBuilderDelegate in the KineoWorks Reference Guide
   
*/
class CkwsGraphicRoadmapDelegate : public CkwsRdmBuilderDelegate {

 public : 
  /**
     \brief Notification Identificator
     
     Sent when an edge has been added to the roadmap
  */
  static const CkitNotification::TType DID_MODIFY_THE_ROADMAP;

  /**
     \brief Notification parameter string indentifier

     This key identifies the roadmap builder argument of notification 
     CkwsRdmBuilderDelegate::DID_MODIFY_THE_ROADMAP.
  */
  static const std::string ROADMAPBUILDER_KEY;


  /**
     \brief Destructor
   */
  ~CkwsGraphicRoadmapDelegate();

  /**
     \brief Constructor
   */ 
  CkwsGraphicRoadmapDelegate();

  /**
     \name Methods reimplemented from CkwsRdmBuilderDelegate

     @{
   */

  /**
     \brief Called before roadmap construction.

     \param inRdmBuilder Roadmap builder the delegate is attached to.
     \param inOutPath Input path, used to enrich the roadmap.

     Store weak pointer to roadmap builder the delegate is attached to.
  */
  virtual void builderWillStartBuilding (const CkwsRoadmapBuilderShPtr &inRdmBuilder, 
					 CkwsPathShPtr &inOutPath);

  /**
     \brief Called after end of roadmap construction.

     \param inRdmBuilder Roadmap builder the delegate is attached to.
     \param inOutPath Path found in case of success.
     \param inSuccess whether the roadmap builder succeeded in finding a path.

     Send notification CkppPlanPathCommand::DID_FINISH_BUILDING.
  */
  virtual void builderDidFinishBuilding (const CkwsRoadmapBuilderShPtr &inRdmBuilder, 
					 CkwsPathShPtr &inOutPath, ktStatus inSuccess);

  /**
     \brief Called repeatedly during the process to allow to interrupt planning
  */
  virtual bool plannerShouldStopPlanning (const CkwsRoadmapBuilderConstShPtr &inRdmBuilder);

  /**
     \brief Called repeatedly during the building process to allow the implementer to prevent the builder from expanding the roadmap towards a configuration
  */
  virtual bool builderShouldExploreTowards (const CkwsRoadmapBuilderConstShPtr &inRdmBuilder, 
					    const CkwsConfig &inConfig);

  /**
     \brief Called whenever a new node is added to the roadmap
  */
  virtual void builderDidAddNode (const CkwsRoadmapBuilderConstShPtr &inRdmBuilder, 
				  const CkwsNodeConstShPtr &inNode);

  /**
     \brief Called whenever a new edge is added to the roadmap 
  */
  virtual void builderDidAddEdge (const CkwsRoadmapBuilderConstShPtr &inRdmBuilder, 
				  const CkwsEdgeConstShPtr &inEdge);

  /**
     \brief Called whenever a path is modified by the builder
  */
  virtual void builderDidModifyPath (const CkwsRoadmapBuilderConstShPtr &inRdmBuilder, 
				     const CkwsPathConstShPtr &i_path);

  /**
     @}
  */
 private :

  long int nbSuccessfulShoots;

  /**
     \brief Weak pointer to roadmap builder
  */
  CkwsRoadmapBuilderWkPtr attRoadmapBuilderWkPtr;

  /**
     \if 0
     \brief Counter of objects of this class
  */
  static unsigned int nbObjects;

  /**
     \endif
  */
};

/**
   @}
*/
#endif
