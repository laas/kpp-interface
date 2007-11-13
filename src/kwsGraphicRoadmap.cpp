/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigne

*/

#include "kppInterface/kwsGraphicRoadmap.h"

#include "KineoWorks2/kwsNode.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoModel/kppColor.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoModel/kppJointComponent.h"
#include "KineoWorks2/kwsJoint.h"

/* GL includes */
#include <GL/gl.h>
#include <GL/glu.h>

#include <iostream>

using namespace std;


CkwsGraphicRoadmap::CkwsGraphicRoadmap(){



}

//-------------------------------------------------------------------------------------------------------------------------------------------------

CkwsGraphicRoadmap::~CkwsGraphicRoadmap(){

  if(m_isDisplayed){
    cout<<"erasing roadmap"<<endl;
    CkppViewGeneral::getInstance()->viewportGraphicMap()->remove( CkppViewGraphicMap::OVERLAY_3D, m_weakPtr.lock());
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------

void CkwsGraphicRoadmap::render(){

  if(m_isDisplayed){
    if(isRealTimeUpdated){
      glPushAttrib(GL_ENABLE_BIT);
      glEnable(GL_LINE_SMOOTH);
      glEnable(GL_BLEND);
      
      glPushName(m_savedGraphicID);
      
      drawRoadmap();
      
      glPopName();
      glPopAttrib();
      
    }else{
      if(finished){
	
	// anti-aliasing
	glPushAttrib(GL_ENABLE_BIT);
	glEnable(GL_LINE_SMOOTH); 
	glEnable(GL_BLEND);
	
	glPushName(m_savedGraphicID);
	
	drawRoadmap();
	
	glPopName();
	glPopAttrib();
      }
      
    }

  }
  
}

//-------------------------------------------------------------------------------------------------------------------------------------------------

CkwsGraphicRoadmapShPtr CkwsGraphicRoadmap::create(const CkwsRoadmapShPtr & i_roadmap,const std::string &inName){

  CkwsGraphicRoadmap * roadmapPtr = new CkwsGraphicRoadmap();
  CkwsGraphicRoadmapShPtr inRoadmap(roadmapPtr);

  if(inRoadmap ->init( inRoadmap, i_roadmap ) != KD_OK ){
   
    inRoadmap.reset();
    
  }

  return inRoadmap;

}

//-------------------------------------------------------------------------------------------------------------------------------------------------


ktStatus CkwsGraphicRoadmap::init(const CkwsGraphicRoadmapWkPtr& i_ptr,const CkwsRoadmapShPtr & i_roadmap){

  ktStatus success = KD_ERROR;
  m_weakPtr = i_ptr;
  isRealTimeUpdated=false;
  finished = false;
  m_isDisplayed = false;
  m_isJointDisplayed = false;
  success = CkppViewGraphic::init( i_ptr );

  m_kwsRoadmap = i_roadmap;
    
  return success;

}


//-------------------------------------------------------------------------------------------------------------------------------------------------

void CkwsGraphicRoadmap::drawRoadmap(){

  // Retrieve vector of joints that should be displayed. This is done here in order to allow users to choose to display or hide each joint's roadmap even after the end of building
  CkwsDeviceShPtr rdmDevice = kwsRoadmap()->device();
  CkwsDevice::TJointVector jointVector;
  rdmDevice->getJointVector(jointVector);
  std::vector<CkwsJointShPtr> displayJointVector;
  for (unsigned int iJoint=0; iJoint<jointVector.size(); iJoint++) {
    CkppJointComponentShPtr kppJoint = KIT_DYNAMIC_PTR_CAST(CkppJointComponent, jointVector[iJoint]);
    if (kppJoint) {
      if (kppJoint->doesDisplayPath()) {
	displayJointVector.push_back(jointVector[iJoint]);
      }
    }
  }
  if (displayJointVector.size() == 0) {
    std::cout << "CkwsPlusRoadmap::compute: no joint to display." << std::endl;
    return;
  }

  //Drawing edges
  for (unsigned int iJoint=0; iJoint < displayJointVector.size(); iJoint++) {
    for(int i=0; i<kwsRoadmap()->countNodes(); i++){//throught the roadmap
      
      CkwsNodeShPtr currentNode = kwsRoadmap()->node(i);
      
      for(int j=0; j<currentNode->countOutEdges(); j++){//throught each node of the roadmap
	
	CkwsJointShPtr kwsJoint = displayJointVector[iJoint];
	CkwsConfig current(kwsRoadmap()->node(i)->config());//current configuration : edge start
	CkwsConfig next(kwsRoadmap()->node(i)->outEdge(j)->endNode()->config());//next configuration : edge end
	
	rdmDevice->setCurrentConfig(current);
	CkitMat4 jointPosition = kwsJoint->currentPosition();
	double x1 = jointPosition(0,3);
	double y1 = jointPosition(1,3);
	double z1 = jointPosition(2,3);
	
	rdmDevice->setCurrentConfig(next);
	jointPosition = kwsJoint->currentPosition();
	double x2 = jointPosition(0,3);
	double y2 = jointPosition(1,3);
	double z2 = jointPosition(2,3);
	
	glPushAttrib(GL_ENABLE_BIT);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	
	glColor4fv(&(CkppColor::DARK_RED)[0]);
	
	//drawing an edge 
	glBegin(GL_LINES);
	glVertex3f(x1,y1,z1);
	glVertex3f(x2,y2,z2);
	glEnd();
	

	//drawing a point for the current node
	glBegin(GL_POINTS);
	glVertex3f(x1,y1,z1);
	glEnd();
	glPointSize(4.f);

/*	glBegin(GL_QUAD_STRIP);
	glVertex3f(x1-1.,y1-1.,z1-1.);
	glVertex3f(x1-1.,y1-1.,z1+1.);
	glVertex3f(x1-1.,y1+1.,z1-1.);
	glVertex3f(x1-1.,y1+1.,z1+1.);
	glVertex3f(x1+1.,y1+1.,z1-1.);
	glVertex3f(x1+1.,y1+1.,z1+1.);
	glVertex3f(x1+1.,y1-1.,z1-1.);
	glVertex3f(x1+1.,y1-1.,z1+1.);
	glEnd();
	glBegin(GL_QUAD_STRIP);
	glVertex3f(x1-1.,y1+1.,z1+1.);
	glVertex3f(x1+1.,y1+1.,z1+1.);
	glVertex3f(x1-1.,y1-1.,z1+1.);
	glVertex3f(x1+1.,y1-1.,z1+1.);
	glVertex3f(x1-1.,y1-1.,z1-1.);
	glVertex3f(x1+1.,y1-1.,z1-1.);
	glVertex3f(x1-1.,y1+1.,z1-1.);
	glVertex3f(x1+1.,y1+1.,z1-1.);
	glEnd();*/
	glLineWidth(1.f);
      }
      
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------

void CkwsGraphicRoadmap::SetRealTimeUpdate(bool rtu){

  isRealTimeUpdated=rtu;
  if(isRealTimeUpdated) m_isDisplayed=true;

}

//-------------------------------------------------------------------------------------------------------------------------------------------------

bool CkwsGraphicRoadmap::GetRealTimeUpdate(){

  return isRealTimeUpdated;

}

//-------------------------------------------------------------------------------------------------------------------------------------------------

void CkwsGraphicRoadmap::drawNotifRoadmap(const CkitNotificationConstShPtr& i_notification){
  m_isDisplayed = true;
  if(i_notification->type() == CkppPlanPathCommand::DID_FINISH_BUILDING) finished = true;
  CkppMainWindowController::getInstance()->graphicWindowController()->viewWindow()->redraw(CkppViewCanvas::NOW);    
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
