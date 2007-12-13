/*
 *  Copyright
 */



/*****************************************
 INCLUDES
*******************************************/

#include <iostream>

#include "KineoGUI/kppMainWindowUICommandFactory.h"
#include "KineoController/kppUICommandList.h"
#include "KineoController/kppUICommand.h"
#include "KineoController/kppCommandEnvironment.h"
#include "KineoController/kppCommand.h"

#include "KineoModel/kppModelTree.h"
#include "KineoModel/kppGeometryNode.h"
#include "KineoModel/kppDeviceNode.h"
#include "KineoModel/kppPathNode.h"
#include "KineoModel/kppPathComponent.h"
#include "KineoModel/kppJointComponent.h"


#include "KineoController/kppInsertComponentCommand.h"
#include "KineoController/kppInsertSolidComponentCommand.h"
#include "KineoController/kppDocument.h"
#include "KineoGUI/kppMainWindowController.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoWorks2/kwsRdmBuilderDelegate.h"

#include "KineoUtility/kitNotification.h"
#include "KineoUtility/kitNotificator.h"

#include "KineoWX/kwxIdleNotification.h"

#include "hppCore/hppProblem.h"

#include "kppInterface/kppInterface.h"
#include "kppInterface/kppCommandStartCorbaServer.h"
#include "kppInterface/kppCommandSetConfig.h"
#include "kppInterface/kppCommandInit.h"

#include "KineoKCDModel/kppKCDBox.h"

#include "KineoKCDModel/kppKCDAssembly.h"

using namespace std;

/*****************************************
 DEFINES
*******************************************/

/*****************************************
 METHODS
*******************************************/

// ==========================================================================

CkppInterfaceShPtr CkppInterface::create()
{
  ChppPlanner *hppPlanner = new ChppPlanner();
  CkppInterface* ptr = new CkppInterface(hppPlanner);

  CkppInterfaceShPtr shPtr(ptr);
  return shPtr;
}

// ==========================================================================

CkppInterface::CkppInterface(ChppPlanner *inHppPlanner) : attHppPlanner(inHppPlanner)
{
  attHppCorbaServer = NULL;
  corbaServerRunning = 0;

  CkitNotificator::defaultNotificator()->subscribe<CkppInterface>(CkppComponent::DID_INSERT_CHILD, this , &CkppInterface::insertChild);
}

// ==========================================================================

CkppInterface::~CkppInterface()
{
  delete attHppPlanner;
  if (attHppCorbaServer != NULL) {
    delete attHppCorbaServer;
  }
  corbaServerRunning = 0;
  removeGraphicRoadmap();
  CkitNotificator::defaultNotificator()->unsubscribe(this);
}

// ==========================================================================

void CkppInterface::getMenuUICommandLists(const CkppMainWindowUICommandFactoryConstShPtr& i_commandFactory,
					  std::vector<CkppUICommandListShPtr> & o_menuCommandListVector)
{
  // ---  LIST --- //

  CkppUICommandListShPtr hppUICommandList = CkppUICommandList::create("HPP");

  attStartCorbaServerCommand = CkppUICommand::create(CkppCommandStartCorbaServer::create(this),
						     i_commandFactory->environment(),
						     "Start CORBA Server",
						     "start corba server");

  attCommandPlannerPanel = CkppUICommand::create(CkppCommandPlannerPanel::create(this,i_commandFactory),
					     i_commandFactory->environment(),
					     "Planner Configuration",
					     "Displays a configuration panel for Path Planning");

  attCommandSetConfigBase = CkppUICommand::create(CkppCommandSetConfig::create(this),
						  i_commandFactory->environment(),
						  "Set INIT/GOAL config",
						  "");

  attCommandInitBase = CkppUICommand::create(CkppCommandInit::create(this),
						  i_commandFactory->environment(),
						  "initialize Planner",
						  "");

  hppUICommandList->appendCommand(attCommandPlannerPanel) ;
  if (attStartCorbaServerCommand) {
    hppUICommandList->appendCommand(attStartCorbaServerCommand) ; 
  } else {
    std::cerr << "CkppInterface: cannot create menu item \"Start CORBA Server\"." << std::endl;
  }
  std::cerr << "CkppInterface: create menu \"HPP\"." << std::endl;
  o_menuCommandListVector.push_back(hppUICommandList);

}

// ==========================================================================

ktStatus CkppInterface::activate()
{

  // Subscribe to periodic function in order to process Corba requests.
  //hppPlanner->subscribe(ID_IDLE, cbHppPeriodicEvent, (void*)this);

  // Subscribe to HPP event ID_HPP_ADD_ROBOT.
  CkitNotificator::defaultNotificator()->subscribe< CkppInterface >(ChppPlanner::ID_HPP_ADD_ROBOT,
								       this,
								       &CkppInterface::hppAddRobot);

  // Subscribe to HPP event ID_HPP_ADD_OBSTACLE triggered when an obstacle is added to hppPlanner object.
  CkitNotificator::defaultNotificator()->subscribe< CkppInterface >(ChppPlanner::ID_HPP_ADD_OBSTACLE,
								       this,
								       &CkppInterface::hppAddObstacle);

  // Subscribe to HPP event ID_HPP_ADD_PATH triggered when a path is added to hppPlanner object.
  CkitNotificator::defaultNotificator()->subscribe< CkppInterface >(ChppProblem::ID_HPP_ADD_PATH,
									this,
									&CkppInterface::hppAddPath);

  // Subscribe to HPP event ID_HPP_SET_OBSTACLE_LIST triggered when a list of obstacles is attached
  // to hppPlanner object.
  CkitNotificator::defaultNotificator()->subscribe< CkppInterface >(ChppPlanner::ID_HPP_SET_OBSTACLE_LIST,
								    this,
								    &CkppInterface::hppSetObstacleList);

  CkitNotificator::defaultNotificator()->subscribe< CkppInterface >(CkwxIdleNotification::TYPE,
								    this,
								    &CkppInterface::onIdle);  
  // debug
  // cerr<<"kppInterface activated."<<endl;
  return KD_OK;
}

// ==========================================================================

ktStatus CkppInterface::startCorbaServer()
{
  int argc=1;
  char *argv[1] = {"KineoPathPlanner"};

  if (corbaServerRunning != 0) {
    cerr << "CkppInterface: Corba Server already started" << endl;
    return KD_ERROR;
  }
  if (!attHppPlanner) {
    cerr << "You need to create a Planner object first." << endl;
    return KD_ERROR;
  }
  attHppCorbaServer = new ChppciServer(attHppPlanner);
  if (!attHppCorbaServer) {
    cerr << "CkppInterface::startCorbaServer: allocation of ChppciServer object failed." << endl;
    return KD_ERROR;
  }

  if (attHppCorbaServer->startCorbaServer(argc, argv) == KD_OK) {
    corbaServerRunning = 1;
    cout << "Corba server is now running." << endl;
  } else {
    corbaServerRunning = 0;
    cerr << "CkppInterface: failed to start Corba server" << endl;
    return KD_ERROR;
  }
  return KD_OK;
}


// ==========================================================================

void CkppInterface::hppAddRobot(const CkitNotificationConstShPtr& i_notification)
{
  // retrieve the object with key
  ChppPlanner *planner = (ChppPlanner*)(i_notification->objectPtr< ChppPlanner >());
  CkppDeviceComponentShPtr  device(i_notification->shPtrValue<CkppDeviceComponent>(ChppPlanner::ROBOT_KEY));
  
  //debug
  //cout<<"hppAddRobot called."<<endl;
 
  CkppMainWindowController* wincontroller = CkppMainWindowController::getInstance() ; // temporary function KPP
  CkppModelTreeShPtr modelTree = wincontroller->document()->modelTree();
  
  //debug
  // cout<<"adding device..."<<endl;
  
  CkppInsertComponentCommandShPtr insertCommand;

  // Get notificator instance
  CkitNotificatorShPtr notificator = CkitNotificator::defaultNotificator();
  
  //debug
  //cout<<" device solid components: "<<device->countSolidComponentRefs()<<endl;
  
   for( unsigned int i=0; i<device->countSolidComponentRefs(); i++)
     {
       notificator->unsubscribe(CkppComponent::DID_INSERT_CHILD, 
				device->solidComponentRef(i)->referencedSolidComponent().get());
       notificator->unsubscribe(CkppComponent::DID_REMOVE_CHILD, 
				device->solidComponentRef(i)->referencedSolidComponent().get());

       insertCommand = CkppInsertSolidComponentCommand::create();
       insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::PARENT_COMPONENT), 
 				CkppComponentShPtr(modelTree->geometryNode()) );
       insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::INSERTED_COMPONENT), 
				 CkppComponentShPtr(device->solidComponentRef(i)->referencedSolidComponent()));
       insertCommand->doExecute() ;
     }

   // temporary: deactivate updating outer list
   notificator->unsubscribe(CkppComponent::DID_INSERT_CHILD, device.get());
   notificator->unsubscribe(CkppComponent::DID_REMOVE_CHILD, device.get());

   insertCommand = CkppInsertComponentCommand::create();
   insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::PARENT_COMPONENT), 
			     CkppComponentShPtr(modelTree->deviceNode()));
   insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::INSERTED_COMPONENT), 
			     CkppComponentShPtr(device) );
   insertCommand->doExecute();
 
}


// ==========================================================================

void CkppInterface::hppAddPath(const CkitNotificationConstShPtr& i_notification)
{
  // retrieve the object with key
  ChppProblem *problem = (ChppProblem*)(i_notification->objectPtr< ChppProblem >());
  CkwsPathShPtr  path(i_notification->shPtrValue<CkwsPath>(ChppProblem::PATH_KEY));
  CkppDeviceComponentShPtr hppDevice(i_notification->shPtrValue<CkppDeviceComponent>(ChppProblem::DEVICE_KEY));
  unsigned int path_id = i_notification->unsignedIntValue(ChppProblem::PATH_ID_KEY);

  //debug
  //cout<<"hppAddPath called."<<endl;
 
  CkppMainWindowController* wincontroller = CkppMainWindowController::getInstance() ; // temporary function KPP
  CkppModelTreeShPtr modelTree = wincontroller->document()->modelTree();

  CkppInsertComponentCommandShPtr insertCommand;

  // create a path component
  char path_name [128];
  sprintf(path_name, "Path %s %d", hppDevice->name().c_str(),  path_id);

  // insert robot
  cout<<"adding path "<<path_name<<" ..."<<endl;

  CkppPathComponentShPtr kppPath = CkppPathComponent::create(path, path_name);

  insertCommand = CkppInsertComponentCommand::create();
  insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::PARENT_COMPONENT), 
                            CkppComponentShPtr(modelTree->pathNode()));
  insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::INSERTED_COMPONENT), 
                            CkppComponentShPtr(kppPath) );
  insertCommand->doExecute();
     
}


// ==========================================================================

void CkppInterface::hppAddObstacle(const CkitNotificationConstShPtr& i_notification)
{
  cout<<" adding obstacles ... "<<endl;

  ChppPlanner *planner = (ChppPlanner*)(i_notification->objectPtr< ChppPlanner >());
  std::vector<CkcdObjectShPtr>*  obstacleList(i_notification->ptrValue< std::vector<CkcdObjectShPtr> >(ChppPlanner::OBSTACLE_KEY));
  
  CkppMainWindowController* wincontroller = CkppMainWindowController::getInstance() ; // temporary function KPP
  CkppModelTreeShPtr modelTree = wincontroller->document()->modelTree();

  CkppInsertComponentCommandShPtr insertCommand;

  unsigned int nbObstacles = obstacleList->size();

  if (nbObstacles == 0) {
    std::cerr << "CkppInterface::hppAddObstacle: obstacle list is empty." << std::endl;
    return;
  }

  CkppKCDAssemblyShPtr kcdAssembly;

  unsigned int iObstacle = nbObstacles-1;

  CkcdObjectShPtr obstacle = (*obstacleList)[iObstacle];

  CkppGeometryNodeShPtr geomNode = modelTree->geometryNode();
  
  CkppGeometryComponentShPtr geomComponent;
  for (unsigned int i=0; i<geomNode->countChildComponents(); i++){
      geomComponent = KIT_DYNAMIC_PTR_CAST( 
	  CkppGeometryComponent, geomNode->childComponent(i));
      CkcdObjectShPtr obj;
      obj = KIT_DYNAMIC_PTR_CAST(CkcdObject, geomComponent);
      if (obj && (obj == obstacle)){
	  //std::cout << "the obstacle is already registered" << std::endl; 
	  return;
      }
  }

  CkppKCDPolyhedronShPtr hppPolyhedron;

//   cout<<"nbObstacles "<<nbObstacles<<endl;

  // Test if obstacle is a polyhedron
  if (hppPolyhedron = boost::dynamic_pointer_cast<CkppKCDPolyhedron>(obstacle)) {
    CkppSolidComponentRefShPtr poly = CkppSolidComponentRef::create(hppPolyhedron);
    // modelTree->geometryNode()->addChildComponent(poly);
    // cerr<<" adding hppPolyhedron."<<endl;

    CkitNotificator::defaultNotificator()->unsubscribe(CkppComponent::DID_INSERT_CHILD,
						       hppPolyhedron.get());
    CkitNotificator::defaultNotificator()->unsubscribe(CkppComponent::DID_REMOVE_CHILD,
						       hppPolyhedron.get());

    insertCommand = CkppInsertSolidComponentCommand::create();
    insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::PARENT_COMPONENT), 
			      CkppComponentShPtr(modelTree->geometryNode()) );
    insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::INSERTED_COMPONENT), 
			      CkppComponentShPtr(poly->referencedSolidComponent()) );
    insertCommand->doExecute();

  } else if (kcdAssembly = boost::dynamic_pointer_cast<CkppKCDAssembly>(obstacle)) {
    CkppSolidComponentRefShPtr assembly = CkppSolidComponentRef::create(kcdAssembly);
    
    CkitNotificator::defaultNotificator()->unsubscribe(CkppComponent::DID_INSERT_CHILD,
						       kcdAssembly.get());
    CkitNotificator::defaultNotificator()->unsubscribe(CkppComponent::DID_REMOVE_CHILD,
						       kcdAssembly.get());
    
    insertCommand = CkppInsertSolidComponentCommand::create();
    insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::PARENT_COMPONENT), 
			      CkppComponentShPtr(modelTree->geometryNode()) );
    insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::INSERTED_COMPONENT), 
			      CkppComponentShPtr(assembly->referencedSolidComponent()) );
    insertCommand->doExecute();
    
  } else {
    cerr << "CkppInterface::setObstacleList: obstacle "
	 << iObstacle << "is of undefined type." << endl;
  }

  cout<<"obstacle added."<<endl;

}


// ==========================================================================

void CkppInterface::hppSetObstacleList(const CkitNotificationConstShPtr& i_notification)
{
  cout<<" adding obstacles ... "<<endl;

  ChppPlanner *planner = (ChppPlanner*)(i_notification->objectPtr< ChppPlanner >());
  std::vector<CkcdObjectShPtr>*  obstacleList(i_notification->ptrValue< std::vector<CkcdObjectShPtr> >(ChppPlanner::OBSTACLE_KEY));
  
  CkppMainWindowController* wincontroller = CkppMainWindowController::getInstance() ; // temporary function KPP
  CkppModelTreeShPtr modelTree = wincontroller->document()->modelTree();

  CkppInsertComponentCommandShPtr insertCommand;

  unsigned int nbObstacles = obstacleList->size();

  if (nbObstacles == 0) {
    std::cerr << "CkppInterface::hppSetObstacleList: obstacle list is empty." << std::endl;
    return;
  }

  CkppKCDAssemblyShPtr kcdAssembly;

  for (unsigned int iObstacle = 0; iObstacle < nbObstacles; iObstacle++) {

    CkcdObjectShPtr obstacle = (*obstacleList)[iObstacle];
    CkppKCDPolyhedronShPtr hppPolyhedron;

    // cout<<"nbObstacles "<<nbObstacles<<endl;

    // Test if obstacle is a polyhedron
    if (hppPolyhedron = boost::dynamic_pointer_cast<CkppKCDPolyhedron>(obstacle)) {
      CkppSolidComponentRefShPtr poly = CkppSolidComponentRef::create(hppPolyhedron);
      // modelTree->geometryNode()->addChildComponent(poly);
      // cerr<<" adding hppPolyhedron."<<endl;
      
      insertCommand = CkppInsertSolidComponentCommand::create();
      insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::PARENT_COMPONENT), 
				CkppComponentShPtr(modelTree->geometryNode()) );
      insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::INSERTED_COMPONENT), 
				CkppComponentShPtr(poly->referencedSolidComponent()) );
      insertCommand->doExecute();
      
    } else if (kcdAssembly = boost::dynamic_pointer_cast<CkppKCDAssembly>(obstacle)) {
      CkppSolidComponentRefShPtr assembly = CkppSolidComponentRef::create(kcdAssembly);
      insertCommand = CkppInsertSolidComponentCommand::create();
      insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::PARENT_COMPONENT), 
				CkppComponentShPtr(modelTree->geometryNode()) );
      insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::INSERTED_COMPONENT), 
				CkppComponentShPtr(assembly->referencedSolidComponent()) );
      insertCommand->doExecute();
      
    } else {
      cerr << "CkppInterface::setObstacleList: obstacle "
	   << iObstacle << "is of undefined type." << endl;
    }
  }
  cout<<"Obstacle list added."<<endl;

}

void CkppInterface::onIdle(const CkitNotificationConstShPtr& i_notification)
{
  if (corbaServerRunning) {
    attHppCorbaServer->processRequest(false);
  }
}


// ==========================================================================

unsigned int CkppInterface::addGraphicRoadmap(CkwsGraphicRoadmapShPtr i_graphic_roadmap, bool i_isRealTimeUpdated){

  i_graphic_roadmap->SetRealTimeUpdate(i_isRealTimeUpdated);

  m_graphic_roadmaps.push_back ( i_graphic_roadmap );

  if(i_isRealTimeUpdated){

    cout<<"The Roadmap will be updated at run time"<<endl; 
    CkppViewGeneral::getInstance()->viewportGraphicMap()->insert( CkppViewGraphicMap::OVERLAY_3D, m_graphic_roadmaps.back() );   
    CkitNotificator::defaultNotificator()->subscribe< CkppInterface >(CkppPlanPathCommand::DID_ADD_EDGE_TO_ROADMAP, this , &CkppInterface::addRoadmap);

  }
  else{
    cout<<"The Roadmap will be updated at the end of building"<<endl; 
    CkppViewGeneral::getInstance()->viewportGraphicMap()->insert( CkppViewGraphicMap::OVERLAY_3D, m_graphic_roadmaps.back() ); 
    CkitNotificator::defaultNotificator()->subscribe< CkppInterface >(CkppPlanPathCommand::DID_FINISH_BUILDING, this, &CkppInterface::addRoadmap);
  }

  CkitNotificator::defaultNotificator()->subscribe< CkppInterface >(CkppWindowController::DID_CHANGE_DOCUMENT,this,&CkppInterface::removeAllRoadmaps);

  return m_graphic_roadmaps.size();

}

// ==========================================================================

void CkppInterface::removeGraphicRoadmap( int i_rank){

  if(i_rank >= (int)m_graphic_roadmaps.size()) cout<<"The rank "<< i_rank <<" is too high ( size of roadmaps vector is "<<m_graphic_roadmaps.size()<<" )"<<endl;
  else if(i_rank < 0){

    for(int i=0; i<m_graphic_roadmaps.size();i++ ){
      
      if(m_graphic_roadmaps[i]->isDisplayed()) CkppViewGeneral::getInstance()->viewportGraphicMap()->remove( CkppViewGraphicMap::OVERLAY_3D, m_graphic_roadmaps[i]);
      m_graphic_roadmaps.erase(m_graphic_roadmaps.begin()+i);

    }

  }
  else{

    if(m_graphic_roadmaps[i_rank]->isDisplayed()) CkppViewGeneral::getInstance()->viewportGraphicMap()->remove( CkppViewGraphicMap::OVERLAY_3D, m_graphic_roadmaps[i_rank]);
    m_graphic_roadmaps.erase(m_graphic_roadmaps.begin()+i_rank);

  }

}


// ==========================================================================

void CkppInterface::showRoadmap(unsigned int i_rank){
  
  if(!m_graphic_roadmaps[i_rank]->isDisplayed()){

    CkppViewGeneral::getInstance()->viewportGraphicMap()->insert( CkppViewGraphicMap::OVERLAY_3D, m_graphic_roadmaps[i_rank] );
    m_graphic_roadmaps[i_rank]->isDisplayed(true);
    CkppMainWindowController::getInstance()->graphicWindowController()->viewWindow()->redraw(CkppViewCanvas::NOW);

  }else cout<<"This roadmap is already displayed"<<endl;

}


// ==========================================================================

void CkppInterface::hideRoadmap(unsigned int i_rank){
  
  if(m_graphic_roadmaps[i_rank]->isDisplayed()){
    
    CkppViewGeneral::getInstance()->viewportGraphicMap()->remove( CkppViewGraphicMap::OVERLAY_3D, m_graphic_roadmaps[i_rank]);
    m_graphic_roadmaps[i_rank]->isDisplayed(false);
    CkppMainWindowController::getInstance()->graphicWindowController()->viewWindow()->redraw(CkppViewCanvas::NOW);

  }else cout<<"This roadmap is already hidden"<<endl;
}

// ==========================================================================

void CkppInterface::addRoadmap(const CkitNotificationConstShPtr& i_notification){

  bool found = false;

  if(i_notification->objectShPtr< CkwsRoadmapBuilder >()){
    //We are searching in the vector for the graphic roadmap that has his intern kwsRoadmap shared pointer identical to the modified roadmap
    for(std::vector<CkwsGraphicRoadmapShPtr>::iterator it = m_graphic_roadmaps.begin() ; it<m_graphic_roadmaps.end() ; it++){
      CkwsRoadmapBuilderShPtr rdmBuilder = i_notification->objectShPtr< CkwsRoadmapBuilder >();
      if((*it)->kwsRoadmap() == rdmBuilder->roadmap()){
	found = true;
	(*it)->drawNotifRoadmap(i_notification);
      }
    }
    
    if(!found) cout<<"There is no graphical roadmap for the modified kwsRoadmap"<<endl;
  }

}

// ==========================================================================

void CkppInterface::removeAllRoadmaps(const CkitNotificationConstShPtr& i_notification){

  removeGraphicRoadmap();

}

// ------------- initializing module ------------------
// extern Function : First called by KineoWorks to Load the Module

int initializeModule(CkppModuleInterfaceShPtr& o_moduleInterface)
{
  cerr<<" initializing module ... "<<endl;
  o_moduleInterface = CkppInterface::create();
  return 0;
}

// ==========================================================================

// DEBUG BLOC - Keep For Eiichi
// CkppJointComponentShPtr kppJoint;
// ChppBodyShPtr hppBody;
// std::vector< CkcdObjectShPtr > outer;
// cout<<"1. device has "<<device->countJointComponents()<<" joints."<<endl;
// for(unsigned int i=0; i<device->countJointComponents(); i++){
//     kppJoint = device->jointComponent(i);
//     // hppBody = KIT_DYNAMIC_PTR_CAST(ChppBody, kppJoint->kwsKCDBody());
//     hppBody = KIT_DYNAMIC_PTR_CAST(ChppBody, kppJoint->kwsJoint()->attachedBody());
//     hppBody->getOuterObjects(outer);
//     cout<<"body "<<hppBody->name()<<" has "<<hppBody->outerObjects().size()<<" outer objects and "
// 	<<outer.size()<<" exact outer objects"<<endl;
//   }
//-------------------------------------------------------------------

void CkppInterface::insertChild(const CkitNotificationConstShPtr& i_notification)
{
    //std::cout << "DID_INSERT_CHILD received"  << std::endl;

    CkppComponentShPtr child(i_notification->shPtrValue< CkppComponent >(CkppComponent::CHILD_KEY)); 

    CkppGeometryComponentShPtr geomComponent;
    geomComponent = KIT_DYNAMIC_PTR_CAST(CkppGeometryComponent, child);
    
    if (geomComponent){
	CkcdObjectShPtr obj = KIT_DYNAMIC_PTR_CAST(CkcdObject, geomComponent);
	if (obj){
	    // check whether obj is a part of robots
	    ChppPlanner *planner = hppPlanner();
	    for (unsigned int i=0; i<planner->getNbHppProblems(); i++){
		CkppDeviceComponentShPtr robot;
		robot = planner->robotIthProblem(i);
		if (robot){
		    for (unsigned int j=0; j<robot->countSolidComponentRefs();
			 j++){
			CkppSolidComponentRefShPtr scref;
			scref = robot->solidComponentRef(j);
			CkppSolidComponentShPtr solid;
			solid = scref->referencedSolidComponent();
			CkcdObjectShPtr obj2;
			obj2 = KIT_DYNAMIC_PTR_CAST(CkcdObject, solid);
			if (obj == obj2){
			    //std::cout << "obj was a part of robot" << std::endl;
			    return;
			}
		    }
		}
	    }
	    planner->addObstacle(obj);
	}
    }
}
