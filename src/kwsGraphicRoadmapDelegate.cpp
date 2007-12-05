/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
*/


/*****************************************
 INCLUDES
*******************************************/

#include "kppInterface/kwsGraphicRoadmapDelegate.h"
#include "KineoGUI/kppMainWindowController.h"
#include "KineoGUI/kppMainFrame.h"
#include <iostream>
#include <ctime>
#include <string>
#include <sstream>
#include <cstdio>
using namespace std;

KIT_PREDEF_CLASS( CkppComponent );

//constructor
CkwsGraphicRoadmapDelegate::CkwsGraphicRoadmapDelegate(const std::string& i_title, const StringArray& i_stepNames):CkppProgressDelegate(i_title,i_stepNames){

  cout<<"Create Delegate ..."<<endl;
  
}

//create method
CkwsGraphicRoadmapDelegateShPtr CkwsGraphicRoadmapDelegate::create(const std::string& i_title, const StringArray& i_stepNames){

  CkwsGraphicRoadmapDelegate*  ptr = new CkwsGraphicRoadmapDelegate(i_title,i_stepNames);
  CkwsGraphicRoadmapDelegateShPtr shPtr(ptr);
  
  cout<<"Creating Delegate - Done"<<endl;

  return shPtr;

}

//what the delegate must do before the builder starts solving the problem
void CkwsGraphicRoadmapDelegate::willStartBuilding (const CkwsRoadmapBuilderShPtr &i_builder, CkwsPathShPtr &io_path){

  nbSuccessfulShoots=0;

  m_builder = i_builder;
  cancellable(true);
  
  start();

  CkwsRdmBuilderDelegate::willStartBuilding (i_builder,io_path);
}

//what the delegate must do after the builder finished solving the problem
void CkwsGraphicRoadmapDelegate::didFinishBuilding (const CkwsRoadmapBuilderShPtr &i_builder, CkwsPathShPtr &io_path, ktStatus i_success){
    
  CkitNotificationShPtr notification = CkitNotification::createWithShPtr<CkwsRoadmapBuilder>(CkppPlanPathCommand::DID_FINISH_BUILDING, m_builder);
  CkitNotificator::defaultNotificator()->notify(notification);

  CkwsRdmBuilderDelegate::didFinishBuilding (i_builder,io_path,i_success);

  finish();

}

//allows user to stop the builder (called repeatedly during the process).
bool CkwsGraphicRoadmapDelegate::shouldStopBuilding (const CkwsRoadmapBuilderConstShPtr &i_builder){

  if(cancellable()){return cancelled();}
  return CkwsRdmBuilderDelegate::shouldStopBuilding (i_builder);

}

//allows user to prevent builder from expanding a node/configuration
bool CkwsGraphicRoadmapDelegate::shouldExploreTowards (const CkwsRoadmapBuilderConstShPtr &i_builder, const CkwsConfig &i_cfg){

  return CkwsRdmBuilderDelegate::shouldExploreTowards (i_builder, i_cfg);

}

//what the delegate must do after the builder adds a node.
void CkwsGraphicRoadmapDelegate::didAddNode (const CkwsRoadmapBuilderConstShPtr &i_builder, const CkwsNodeConstShPtr &i_node){

  nbSuccessfulShoots++;
  std::stringstream display;
  display << "Building Roadmap - ";
  display << nbSuccessfulShoots;
  display << " nodes added\n";
  refresh();
  report(nbSuccessfulShoots,display.str());

  CkitNotificationShPtr notification = CkitNotification::createWithShPtr<CkwsRoadmapBuilder>(CkppPlanPathCommand::DID_ADD_NODE_TO_ROADMAP, m_builder);
  CkitNotificator::defaultNotificator()->notify(notification);

  CkwsRdmBuilderDelegate::didAddNode (i_builder,i_node);

}

//what the delegate must do after the builder adds an edge.
void CkwsGraphicRoadmapDelegate::didAddEdge (const CkwsRoadmapBuilderConstShPtr &i_builder, const CkwsEdgeConstShPtr &i_edge){

  CkitNotificationShPtr notification = CkitNotification::createWithShPtr<CkwsRoadmapBuilder>(CkppPlanPathCommand::DID_ADD_EDGE_TO_ROADMAP, m_builder);
  CkitNotificator::defaultNotificator()->notify(notification);

  CkwsRdmBuilderDelegate::didAddEdge (i_builder,i_edge);

}

//what the delegate must do after the builder modifies the path.
void CkwsGraphicRoadmapDelegate::didModifyPath (const CkwsRoadmapBuilderConstShPtr &i_builder, const CkwsPathConstShPtr &i_path){

  CkwsRdmBuilderDelegate::didModifyPath (i_builder,i_path);

}
