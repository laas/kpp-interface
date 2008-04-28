/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigne

*/

#include "kppInterface/kwsGraphicRoadmap.h"
#include "kppInterface/kwsGraphicRoadmapDelegate.h"

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

// Select verbosity at configuration by setting CXXFLAGS="... -DDEBUG=[1 or 2]"
#if DEBUG==2
#define ODEBUG2(x) std::cout << "kwsGraphicRoadmap:" << x << std::endl
#define ODEBUG1(x) std::cerr << "kwsGraphicRoadmap:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "kwsGraphicRoadmap:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif


CkwsGraphicRoadmap::CkwsGraphicRoadmap(){



}

//-------------------------------------------------------------------------------------------------------------------------------------------------

CkwsGraphicRoadmap::~CkwsGraphicRoadmap(){

  if(m_isDisplayed){
    cout<<"erasing roadmap"<<endl;
    CkppViewGeneral::getInstance()->viewportGraphicMap()->remove( CkppViewGraphicMap::OVERLAY_3D, m_weakPtr.lock());
  }
  m_kwsRoadmap.reset();
  m_weakPtr.reset();
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

CkwsGraphicRoadmapShPtr CkwsGraphicRoadmap::create(const CkwsRoadmapBuilderShPtr & i_roadmapBuilder,const std::string &inName){

  CkwsGraphicRoadmap * graphicRoadmapPtr = new CkwsGraphicRoadmap();
  CkwsGraphicRoadmapShPtr graphicRoadmapShPtr(graphicRoadmapPtr);
  CkwsGraphicRoadmapWkPtr graphicRoadmapWkPtr(graphicRoadmapShPtr);

  if (graphicRoadmapPtr->init(graphicRoadmapWkPtr, i_roadmapBuilder ) != KD_OK ) {
    graphicRoadmapShPtr.reset();
  }
  return graphicRoadmapShPtr;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------


ktStatus CkwsGraphicRoadmap::init(const CkwsGraphicRoadmapWkPtr& inGrRdmWkPtr,const CkwsRoadmapBuilderShPtr & inRoadmapBuilder){

  ktStatus success = KD_ERROR;
  m_weakPtr = inGrRdmWkPtr;
  isRealTimeUpdated=false;
  finished = false;
  m_isDisplayed = false;
  m_isJointDisplayed = false;
  success = CkppViewGraphic::init(inGrRdmWkPtr);

  if (inRoadmapBuilder) {
    inRoadmapBuilder->addDelegate(new CkwsGraphicRoadmapDelegate());
  }
  else {
    ODEBUG1(" kwsGraphicRoadmap: roadmap builder is NULL");
    return KD_ERROR;
  }

  m_kwsRoadmap = inRoadmapBuilder->roadmap();
    
  cout<<"Initializing GraphicRoadmap - Done"<<endl;

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
	
	std::vector<CkitMat4> jointPositions;
	double x1,y1,z1,x2,y2,z2 = 0;
	if(KD_OK == current.getJointMatVector(jointPositions)){
	  CkitMat4 jointPosition = jointPositions.at(iJoint);
	  x1 = jointPosition(0,3);
	  y1 = jointPosition(1,3);
	  z1 = jointPosition(2,3);
	}

	if(KD_OK == next.getJointMatVector(jointPositions)){
	  CkitMat4 jointPosition = jointPositions.at(iJoint);
	  x2 = jointPosition(0,3);
	  y2 = jointPosition(1,3);
	  z2 = jointPosition(2,3);
	}

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
