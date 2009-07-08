/*
  Copyright 2008 CNRS-LAAS

  Authors: David Flavigne and Florent Lamiraux

*/

#include "kppInterface/kwsGraphicRoadmap.h"
#include "kppInterface/kppInterface.h"
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

unsigned int CkwsGraphicRoadmap::nbObjects = 0;

using namespace std;

// Select verbosity at configuration by setting CXXFLAGS="... -DDEBUG=[1 or 2]"
#if DEBUG==2
#define ODEBUG2(x) std::cout << "CkwsGraphicRoadmap:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CkwsGraphicRoadmap:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "CkwsGraphicRoadmap:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif


CkwsGraphicRoadmap::CkwsGraphicRoadmap(const std::string& inName) :
  attName(inName)
{
  nbObjects++;
  ODEBUG2(" constructor: nb existing objects: " << nbObjects);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------

CkwsGraphicRoadmap::~CkwsGraphicRoadmap(){

  nbObjects--;
  ODEBUG2(" destructor: nb existing objects: " << nbObjects);
  if(attIsDisplayed){
    ODEBUG2("erasing roadmap");
    CkppViewGeneral::getInstance()->viewportGraphicMap()->remove( CkppViewGraphicMap::SCENE_3D, attWeakPtr.lock());
  }
  attWeakPtr.reset();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------

void CkwsGraphicRoadmap::render(){

  ODEBUG2(":render called");
  if(attIsDisplayed){
    if(isRealTimeUpdated){
      glPushAttrib(GL_ENABLE_BIT);
      glEnable(GL_LINE_SMOOTH);
      glEnable(GL_BLEND);

      glPushName(m_savedGraphicID);

      drawRoadmap();

      glPopName();
      glPopAttrib();

    }else{
      if(attFinished){

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

CkwsGraphicRoadmapShPtr CkwsGraphicRoadmap::create(const CkwsRoadmapBuilderShPtr & inRoadmapBuilder,
						   const CkppInterfaceWkPtr& inKppInterface,
						   const std::string &inName){

  CkwsGraphicRoadmap * graphicRoadmapPtr = new CkwsGraphicRoadmap(inName);
  CkwsGraphicRoadmapShPtr graphicRoadmapShPtr(graphicRoadmapPtr);
  CkwsGraphicRoadmapWkPtr graphicRoadmapWkPtr(graphicRoadmapShPtr);

  if (graphicRoadmapPtr->init(graphicRoadmapWkPtr, inKppInterface, inRoadmapBuilder) != KD_OK ) {
    graphicRoadmapShPtr.reset();
  }
  return graphicRoadmapShPtr;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------


ktStatus CkwsGraphicRoadmap::init(const CkwsGraphicRoadmapWkPtr& inGrRdmWkPtr,
				  const CkppInterfaceWkPtr& inKppInterface,
				  const CkwsRoadmapBuilderShPtr & inRoadmapBuilder){

  ktStatus success = KD_ERROR;
  attWeakPtr = inGrRdmWkPtr;
  isRealTimeUpdated=false;
  attFinished = false;
  attIsDisplayed = false;
  attKppInterface = inKppInterface;

  success = CkppViewGraphic::init(inGrRdmWkPtr);
//  CkppViewGraphic::isAlwaysDisplayed(false);
  if (inRoadmapBuilder) {
    inRoadmapBuilder->addDelegate(new CkwsGraphicRoadmapDelegate());
  }
  else {
    ODEBUG1(" kwsGraphicRoadmap: roadmap builder is NULL");
    return KD_ERROR;
  }

  attKwsRoadmap = inRoadmapBuilder->roadmap();

  ODEBUG2(":init - Done");

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
    ODEBUG2(":drawRoadmap: no joint to display.");
    return;
  }

  //Drawing edges
  for (unsigned int iJoint=0; iJoint < displayJointVector.size(); iJoint++) {
    for(unsigned int i=0; i<kwsRoadmap()->countNodes(); i++){//throught the roadmap

      CkwsNodeShPtr currentNode = kwsRoadmap()->node(i);

      for(unsigned int j=0; j<currentNode->countOutEdges(); j++){//throught each node of the roadmap

	CkwsJointShPtr kwsJoint = displayJointVector[iJoint];
	CkwsConfig current(kwsRoadmap()->node(i)->config());//current configuration : edge start
	CkwsConfig next(kwsRoadmap()->node(i)->outEdge(j)->endNode()->config());//next configuration : edge end

	std::vector<CkitMat4> jointPositions;
	double x1 = 0.;
	double y1 = 0.;
	double z1 = 0.;
	double x2 = 0.;
	double y2 = 0.;
	double z2 = 0.;

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

	glColor4fv(&(CkppColor::DARK_GREEN)[0]);

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
  if(isRealTimeUpdated) attIsDisplayed=true;

}

//-------------------------------------------------------------------------------------------------------------------------------------------------

bool CkwsGraphicRoadmap::GetRealTimeUpdate(){

  return isRealTimeUpdated;

}

//-------------------------------------------------------------------------------------------------------------------------------------------------

void CkwsGraphicRoadmap::drawNotifRoadmap(const CkitNotificationConstShPtr& inNotification)
{
  ODEBUG2(":drawNotifRoadmap called");
  attIsDisplayed = true;
  if(inNotification->type() == CkppPlanPathCommand::DID_FINISH_BUILDING) {
    attFinished = true;
  }
  attKppInterface.lock()->mainWindowController()->graphicWindowController()->viewWindow()->redraw(CkppViewCanvas::NOW);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
