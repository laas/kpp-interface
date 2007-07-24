/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigne

*/

#include "kppInterface/kwsGraphicRoadmap.h"

#include "KineoWorks2/kwsNode.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoModel/kppColor.h"

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
  success = CkppViewGraphic::init( i_ptr );

  m_kwsRoadmap = i_roadmap;
    
  return success;

}


//-------------------------------------------------------------------------------------------------------------------------------------------------

void CkwsGraphicRoadmap::drawEdge(const CkwsEdgeShPtr& i_edge){

  CkwsConfig Start = i_edge->startNode()->config();
  CkwsConfig End = i_edge->endNode()->config();
	
  glPushAttrib(GL_ENABLE_BIT);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);

  glColor4fv(&(CkppColor::DARK_RED)[0]);

  glBegin(GL_LINES);
  glVertex3f(Start.dofValue(0),Start.dofValue(1),Start.dofValue(2));
  glVertex3f(End.dofValue(0),End.dofValue(1),End.dofValue(2));
  glEnd();
  
  glLineWidth(1.f);
	
}

//-------------------------------------------------------------------------------------------------------------------------------------------------

void CkwsGraphicRoadmap::drawLastEdge(){

  if(m_kwsRoadmap->countNodes()){
    CkwsNodeShPtr lastNode =m_kwsRoadmap->node(m_kwsRoadmap->countNodes()-2);

      if(lastNode->countOutEdges()){
	CkwsEdgeShPtr lastEdge = lastNode->outEdge(lastNode->countOutEdges()-1);
	drawEdge(lastEdge);	
      }else if(lastNode->countInEdges()){
	CkwsEdgeShPtr lastEdge = lastNode->inEdge(lastNode->countInEdges()-1);
	drawEdge(lastEdge);
      }
  }
  else cout<<"no nodes in the roadmap"<<endl;

}

//-------------------------------------------------------------------------------------------------------------------------------------------------

void CkwsGraphicRoadmap::drawRoadmap(){

  for(int i=0; i<m_kwsRoadmap->countNodes(); i++){//throught the roadmap

    CkwsNodeShPtr currentNode = m_kwsRoadmap->node(i);
    
    for(int j=0; j<currentNode->countOutEdges(); j++){//throught each node of the roadmap

      CkwsEdgeShPtr currentEdge = currentNode->outEdge(j);
      drawEdge(currentEdge);
	
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

void CkwsGraphicRoadmap::drawLastNotifEdge(const CkitNotificationConstShPtr& i_notification){

  m_isDisplayed = true;
  CkppMainWindowController::getInstance()->graphicWindowController()->viewWindow()->redraw(CkppViewCanvas::NOW);

}

//-------------------------------------------------------------------------------------------------------------------------------------------------

void CkwsGraphicRoadmap::drawNotifRoadmap(const CkitNotificationConstShPtr& i_notification){
    
  m_isDisplayed = true;
  finished = true;
  CkppMainWindowController::getInstance()->graphicWindowController()->viewWindow()->redraw(CkppViewCanvas::NOW);    

}

//-------------------------------------------------------------------------------------------------------------------------------------------------
