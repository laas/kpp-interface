/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
*/

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

/*************************************
CLASS
**************************************/
/**
   \brief Implements a class which allows to follow a roadmapbuilder evolution

   see class CkwsRdmBuilderDelegate in the KineoWorks Reference Guide
   
*/
class CkwsGraphicRoadmapDelegate : public CkwsRdmBuilderDelegate{

 public : 

  /**
     \brief Destructor
   */
  ~CkwsGraphicRoadmapDelegate();

  /**
     \brief Constructor
   */ 
  CkwsGraphicRoadmapDelegate();

  /**
     These methods are virtual methods from CkwsRdmBuilderDelegate, reimplemented.
   */
  virtual void builderWillStartBuilding (const CkwsRoadmapBuilderShPtr &i_builder, CkwsPathShPtr &io_path);
  virtual void builderDidFinishBuilding (const CkwsRoadmapBuilderShPtr &i_builder, CkwsPathShPtr &io_path, ktStatus i_success);
  virtual bool plannerShouldStopPlanning (const CkwsRoadmapBuilderConstShPtr &i_builder);
  virtual bool builderShouldExploreTowards (const CkwsRoadmapBuilderConstShPtr &i_builder, const CkwsConfig &i_cfg);
  virtual void builderDidAddNode (const CkwsRoadmapBuilderConstShPtr &i_builder, const CkwsNodeConstShPtr &i_node);
  virtual void builderDidAddEdge (const CkwsRoadmapBuilderConstShPtr &i_builder, const CkwsEdgeConstShPtr &i_edge);
  virtual void builderDidModifyPath (const CkwsRoadmapBuilderConstShPtr &i_builder, const CkwsPathConstShPtr &i_path);

 protected : 
 private :
  long int nbSuccessfulShoots;
  CkwsRoadmapBuilderShPtr m_builder;

};

#endif
