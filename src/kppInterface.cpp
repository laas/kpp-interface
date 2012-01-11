/*
 *  Copyright
 */



/*****************************************
 INCLUDES
*******************************************/

#include <iostream>
#include <sstream>

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
#include "KineoModel/kppSMLinearComponent.h"


#include "KineoController/kppInsertComponentCommand.h"
#include "KineoController/kppInsertSolidComponentCommand.h"
#include "KineoController/kppDocument.h"
#include "KineoGUI/kppMainWindowController.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoWorks2/kwsRdmBuilderDelegate.h"
#include "KineoWorks2/kwsPath.h"

#include "KineoUtility/kitNotification.h"
#include "KineoUtility/kitNotificator.h"

#include "KineoWX/kwxIdleNotification.h"

#include <hpp/util/debug.hh>

#include "hpp/core/problem.hh"
#include <hpp/corbaserver/server.hh>

#include "kppInterface/kwsGraphicRoadmap.h"
#include "kppInterface/kppInterface.h"
#include "kppInterface/kppCommandStartCorbaServer.h"
#include "kppInterface/kppCommandSetConfig.h"
#include "kppInterface/kppCommandInit.h"
#include "kppInterface/kppCommandPlannerPanel.h"

#include "KineoKCDModel/kppKCDBox.h"
#include "KineoKCDModel/kppKCDAssembly.h"

#include <kwsPlus/directPath/directPathVector.h>

using namespace std;

/*****************************************
 DEFINES
*******************************************/

// Select verbosity at configuration by setting CXXFLAGS="... -DDEBUG=[1 or 2]"
#if DEBUG==2
#define ODEBUG2(x) std::cout << "CkppInterface:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CkppInterface:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "CkppInterface:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

/*****************************************
 METHODS
*******************************************/

static
CkwsPathShPtr wrapPathInDirectPathVector (CkwsPathShPtr path)
{
  CkwsPathShPtr outPath = CkwsPath::create (path->device ());
  std::vector< CkwsDirectPathConstShPtr > dpVector;
  for (unsigned int i=0; i < path->countDirectPaths (); i++) {
    CkwsDirectPathConstShPtr dp = path->directPath (i);
    std::cout << "i=" << i << std::endl;
    assert (dp);
    dpVector.push_back (dp);
  }
  CdirectPathVectorShPtr directPath =
    CdirectPathVector::create (*(dpVector.front ()->configAtStart ()),
			       *(dpVector.back ()->configAtEnd()),
			       CkppSMLinearComponent::create (),
			       dpVector);
  assert (outPath->appendDirectPath (directPath) == KD_OK);
  return outPath;
}

// ==========================================================================

CkppInterfaceShPtr CkppInterface::create()
{
  hpp::core::Planner *hppPlanner = new hpp::core::Planner(false);
  CkppInterface* ptr = new CkppInterface(hppPlanner);

  CkppInterfaceShPtr shPtr(ptr);
  CkppInterfaceWkPtr wkPtr(shPtr);
  if (ptr->init(wkPtr) != KD_OK) {
    shPtr.reset();
  }
  return shPtr;
}

ktStatus CkppInterface::init(const CkppInterfaceWkPtr& inKppInterface)
{
  attWeakPtr=inKppInterface;
  return KD_OK;
}

// ==========================================================================

CkppInterface::CkppInterface(hpp::core::Planner *inHppPlanner) : attHppPlanner(inHppPlanner)
{
  int argc=1;

  const char *argv[1] = {"Kite"};

  attHppCorbaServer = NULL;
  corbaServerRunning = 0;
  attGraphicRoadmaps.clear();
  attHppCorbaServer = new hpp::corbaServer::Server (inHppPlanner, argc, argv);

  // If the  module is compiled with this pre-compilation symbol, start Corba
  // server when module is initialized.
#ifdef KPP_INTERFACE_START_ALL
  if (startCorbaServer() != KD_OK)
    {
      ODEBUG1("CkppInterface: failed to start Corba server.");
      assert (0 && "Failed to start Corba server.");
    }
#endif
}

// ==========================================================================

CkppInterface::~CkppInterface()
{
  delete attHppPlanner;
  if (attHppCorbaServer != NULL) {
    delete attHppCorbaServer;
    attHppCorbaServer = NULL;
  }
  attCommandInitBase.reset();
  attCommandSetConfigBase.reset();
  attStartCorbaServerCommand.reset();
  attCommandPlannerPanel.reset();
  corbaServerRunning = 0;
  removeGraphicRoadmap();
  attGraphicRoadmaps.clear();
  CkitNotificator::defaultNotificator()->unsubscribe(this);
}

// ==========================================================================

void CkppInterface::getMainWindowController(const CkppMainWindowUICommandFactoryConstShPtr& inCommandFactory)
{
  attMainWindowControllerWkPtr = inCommandFactory->mainWindowController();
}

// ==========================================================================

CkppMainWindowControllerShPtr CkppInterface::mainWindowController()
{
  CkppMainWindowControllerShPtr mainWindowController = attMainWindowControllerWkPtr.lock();
  if (!mainWindowController) {
    ODEBUG1(":mainWindowController: Cannot get the main window controller.");
    ODEBUG1(":mainWindowController:  One possible cause of this problem is that");
    ODEBUG1(":mainWindowController:  you redefined method getMenuUICommandLists() in ");
    ODEBUG1(":mainWindowController:  a class deriving from CkppInterface but did");
    ODEBUG1(":mainWindowController:  not call CkppInterface::getMainWindowController().");
    exit(-1);
  }
  return mainWindowController;
}

// ==========================================================================

void CkppInterface::getMenuUICommandLists(const CkppMainWindowUICommandFactoryConstShPtr& inCommandFactory,
					  std::vector<CkppUICommandListShPtr> & outMenuCommandListVector)
{
  /*
    Get and store pointer to the main window controller. This is the only place where we
    have access to this object
  */
  getMainWindowController(inCommandFactory);

  // ---  LIST --- //

  CkppUICommandListShPtr hppUICommandList = CkppUICommandList::create("HPP");

  attStartCorbaServerCommand = CkppUICommand::create(CkppCommandStartCorbaServer::create(this),
						     inCommandFactory->environment(),
						     "Start CORBA Server",
						     "start corba server");

  attCommandPlannerPanel = CkppUICommand::create(CkppCommandPlannerPanel::create(this,inCommandFactory),
					     inCommandFactory->environment(),
					     "Planner Configuration",
					     "Displays a configuration panel for Path Planning");

  attCommandSetConfigBase = CkppUICommand::create(CkppCommandSetConfig::create(this),
						  inCommandFactory->environment(),
						  "Set INIT/GOAL config",
						  "");

  attCommandInitBase = CkppUICommand::create(CkppCommandInit::create(this),
						  inCommandFactory->environment(),
						  "initialize Planner",
						  "");

  hppUICommandList->appendCommand(attCommandPlannerPanel) ;

  if (attStartCorbaServerCommand) {
    hppUICommandList->appendCommand(attStartCorbaServerCommand) ;
  } else {
    std::cerr << "CkppInterface: cannot create menu item \"Start CORBA Server\"." << std::endl;
  }
  outMenuCommandListVector.push_back(hppUICommandList);

}

// ==========================================================================

ktStatus CkppInterface::activate()
{

  // Subscribe to periodic function in order to process Corba requests.
  //hppPlanner->subscribe(ID_IDLE, cbHppPeriodicEvent, (void*)this);

  // Subscribe to HPP event ID_HPP_ADD_ROBOT.
  CkitNotificator::defaultNotificator()->subscribe< CkppInterface >(hpp::core::Planner::ID_HPP_ADD_ROBOT,
								       this,
								       &CkppInterface::hppAddRobot);

  // Subscribe to HPP event ID_HPP_ADD_OBSTACLE triggered when an obstacle is added to hppPlanner object.
  CkitNotificator::defaultNotificator()->subscribe< CkppInterface >(hpp::core::Planner::ID_HPP_ADD_OBSTACLE,
								       this,
								       &CkppInterface::hppAddObstacle);

  // Subscribe to HPP event ID_HPP_ADD_PATH triggered when a path is added to hppPlanner object.
  CkitNotificator::defaultNotificator()->subscribe< CkppInterface >(ChppProblem::ID_HPP_ADD_PATH,
									this,
									&CkppInterface::hppAddPath);

  // Subscribe to HPP event ID_HPP_SET_OBSTACLE_LIST triggered when a list of obstacles is attached
  // to hppPlanner object.
  CkitNotificator::defaultNotificator()->subscribe< CkppInterface >(hpp::core::Planner::ID_HPP_SET_OBSTACLE_LIST,
								    this,
								    &CkppInterface::hppSetObstacleList);

  // Subscribe to HPP event ID_HPP_REMOVE_ROADMAPBUILDER triggered when a roadmap builder is destroyed
  CkitNotificator::defaultNotificator()->subscribe< CkppInterface >(hpp::core::Planner::ID_HPP_REMOVE_ROADMAPBUILDER,
								    this,
								    &CkppInterface::hppRemoveRoadmapBuilder);

  // Subscribe to HPP event ID_HPP_ADD_ROADMAPBUILDER triggered when a new roadmap should be displayed
  CkitNotificator::defaultNotificator()->subscribe< CkppInterface >(hpp::core::Planner::ID_HPP_ADD_ROADMAPBUILDER,
								    this,
								    &CkppInterface::hppAddGraphicRoadmap);


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

  if (corbaServerRunning != 0) {
    ODEBUG1("CkppInterface: Corba Server already started");
    return KD_ERROR;
  }
  if (!attHppPlanner) {
    ODEBUG1("You need to create a Planner object first.");
    return KD_ERROR;
  }
  if (attHppCorbaServer->startCorbaServer() == KD_OK) {
    corbaServerRunning = 1;
    ODEBUG1("Corba server is now running.");
  } else {
    corbaServerRunning = 0;
    ODEBUG1("CkppInterface: failed to start Corba server");
    return KD_ERROR;
  }
  return KD_OK;
}


// ==========================================================================

void CkppInterface::hppAddRobot(const CkitNotificationConstShPtr& inNotification)
{
  // retrieve the object with key
  CkppDeviceComponentShPtr
    device(inNotification->shPtrValue<CkppDeviceComponent>
	   (hpp::core::Planner::ROBOT_KEY));

  //debug
  //cout<<"hppAddRobot called."<<endl;

  CkppModelTreeShPtr
    modelTree = mainWindowController()->document()->modelTree();

  //before adding device, we check if it's already in the model tree
  CkppComponentShPtr deviceNode = modelTree->deviceNode();
  bool canAddDevice = true;
  for(unsigned int i=0; i<deviceNode->countChildComponents(); i++){
    if(device == KIT_DYNAMIC_PTR_CAST
       (CkppDeviceComponent, deviceNode->childComponent(i)))
      canAddDevice = false;
  }

  //debug
  // cout<<"adding device..."<<endl;
  if(canAddDevice){
    CkppInsertComponentCommandShPtr insertCommand;

    // Get notificator instance
    CkitNotificatorShPtr notificator = CkitNotificator::defaultNotificator();

    for( unsigned int i=0; i<device->countSolidComponentRefs(); i++)
      {
	CkppComponentShPtr componentToInsert
	  (device->solidComponentRef(i)->referencedSolidComponent());

	// If component is in a tree, extract it first.
	if (CkppComponentShPtr parentComp = componentToInsert->parent ()) {
	  parentComp->removeChildComponent (componentToInsert);
	}

	notificator->unsubscribe (CkppComponent::DID_INSERT_CHILD,
				  componentToInsert.get());
	notificator->unsubscribe (CkppComponent::DID_REMOVE_CHILD,
				  componentToInsert.get());

	insertCommand = CkppInsertSolidComponentCommand::create();
	insertCommand->paramValue
	  (insertCommand->parameter(CkppInsertComponentCommand::
				    PARENT_COMPONENT),
	   CkppComponentShPtr(modelTree->geometryNode()) );
	insertCommand->paramValue
	  (insertCommand->parameter(CkppInsertComponentCommand::
				    INSERTED_COMPONENT), componentToInsert);
	hppDout (info, "Attempting to insert "<< componentToInsert->name ()
		 << " into "
		 << modelTree->geometryNode ()->name () <<".");
	insertCommand->doExecute() ;
      }

    // temporary: deactivate updating outer list
    notificator->unsubscribe(CkppComponent::DID_INSERT_CHILD, device.get());
    notificator->unsubscribe(CkppComponent::DID_REMOVE_CHILD, device.get());

    // If device is in a tree, extract it first.
    if (CkppComponentShPtr parent = device->parent ()) {
      parent->removeChildComponent (device);
    }

    insertCommand = CkppInsertComponentCommand::create();
    insertCommand->paramValue
      (insertCommand->parameter(CkppInsertComponentCommand::PARENT_COMPONENT),
       deviceNode);
    insertCommand->paramValue
      (insertCommand->parameter(CkppInsertComponentCommand::INSERTED_COMPONENT),
       CkppComponentShPtr (device));
    hppDout (info, "Attempting to insert " << device->name () << " into "
	     << deviceNode->name () <<".");
    insertCommand->doExecute();
    hppDout (info, "Steering method component: "
	     <<device->steeringMethodComponent ()->name ());
  }
}


// ==========================================================================

void CkppInterface::hppAddPath(const CkitNotificationConstShPtr& inNotification)
{
  // retrieve the object with key
  CkwsPathShPtr  path(inNotification->shPtrValue<CkwsPath>(ChppProblem::PATH_KEY));
  CkppDeviceComponentShPtr hppDevice(inNotification->shPtrValue<CkppDeviceComponent>(ChppProblem::DEVICE_KEY));
  unsigned int path_id = inNotification->unsignedIntValue(ChppProblem::PATH_ID_KEY);

  //debug
  //cout<<"hppAddPath called."<<endl;

  CkppModelTreeShPtr modelTree = mainWindowController()->document()->modelTree();

  CkppInsertComponentCommandShPtr insertCommand;

  // create a path component
  char path_name [128];
  sprintf(path_name, "Path %s %d", hppDevice->name().c_str(),  path_id);

  // insert robot
  ODEBUG2("adding path " << path_name << " ...");

  CkwsPathShPtr singleDPPath = wrapPathInDirectPathVector (path);
  CkppPathComponentShPtr kppPath =
    CkppPathComponent::create(singleDPPath, path_name);

  insertCommand = CkppInsertComponentCommand::create();
  insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::PARENT_COMPONENT),
                            CkppComponentShPtr(modelTree->pathNode()));
  insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::INSERTED_COMPONENT),
                            CkppComponentShPtr(kppPath) );
  insertCommand->doExecute();

}


// ==========================================================================

void CkppInterface::hppAddObstacle(const CkitNotificationConstShPtr& inNotification)
{
  std::vector<CkcdObjectShPtr>*
    obstacleList(inNotification->ptrValue< std::vector<CkcdObjectShPtr> >
		 (hpp::core::Planner::OBSTACLE_KEY));
  CkppDocumentShPtr document = mainWindowController()->document();
  if (!document) return;
  CkppModelTreeShPtr modelTree = document->modelTree();

  CkppInsertComponentCommandShPtr insertCommand;

  unsigned int nbObstacles = obstacleList->size();

  if (nbObstacles == 0) {
    hppDout (error, "CkppInterface::hppAddObstacle: obstacle list is empty.");
    return;
  }

  CkppKCDAssemblyShPtr kcdAssembly;

  unsigned int iObstacle = nbObstacles-1;

  CkcdObjectShPtr obstacle = (*obstacleList)[iObstacle];

  CkppGeometryNodeShPtr geomNode = modelTree->geometryNode();

  hppDout(info, "the number of child = " << geomNode->countChildComponents());

  CkppGeometryComponentShPtr geomComponent;
  for (unsigned int i=0; i<geomNode->countChildComponents(); i++){
      geomComponent = KIT_DYNAMIC_PTR_CAST
	(CkppGeometryComponent, geomNode->childComponent(i));
      CkcdObjectShPtr obj;
      obj = KIT_DYNAMIC_PTR_CAST(CkcdObject, geomComponent);
      if (obj && (obj == obstacle)){
	return;
      }
  }

  CkppKCDPolyhedronShPtr hppPolyhedron;

  CkppSolidComponentShPtr solidComp;
  // Test if obstacle is a solid component
  if (solidComp = boost::dynamic_pointer_cast<CkppSolidComponent>(obstacle)) {
    // if solid component has a parent, detach it from parent
    if (CkppComponentShPtr parent = solidComp->parent ()) {
      parent->removeChildComponent (solidComp);
    }
    CkppSolidComponentRefShPtr poly = CkppSolidComponentRef::create(solidComp);
    // modelTree->geometryNode()->addChildComponent(poly);
    // cerr<<" adding solidComp."<<endl;
    CkitNotificator::defaultNotificator()->unsubscribe
      (CkppComponent::DID_INSERT_CHILD, solidComp.get());
    CkitNotificator::defaultNotificator()->unsubscribe
      (CkppComponent::DID_REMOVE_CHILD, solidComp.get());

    insertCommand = CkppInsertSolidComponentCommand::create();
    insertCommand->paramValue(insertCommand->parameter
			      (CkppInsertComponentCommand::PARENT_COMPONENT),
			      CkppComponentShPtr(modelTree->geometryNode ()));
    insertCommand->paramValue(insertCommand->parameter
			      (CkppInsertComponentCommand::INSERTED_COMPONENT),
			      CkppComponentShPtr
			      (poly->referencedSolidComponent ()));
    insertCommand->doExecute();
    hppDout(info, "obstacle added.");
  } else {
    hppDout(notice, ":hppAddObstacle: obstacle " << iObstacle
	    << "is of undefined type.");
  }
}


// ==========================================================================

void CkppInterface::hppSetObstacleList(const CkitNotificationConstShPtr& inNotification)
{
  ODEBUG2(" adding obstacles ... ");

  std::vector<CkcdObjectShPtr>*  obstacleList(inNotification->ptrValue< std::vector<CkcdObjectShPtr> >(hpp::core::Planner::OBSTACLE_KEY));

  CkppModelTreeShPtr modelTree = mainWindowController()->document()->modelTree();

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
/*    if (hppPolyhedron = boost::dynamic_pointer_cast<CkppKCDPolyhedron>(obstacle)) {
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
*/

    if(CkppSolidComponentShPtr solid = KIT_DYNAMIC_PTR_CAST(CkppSolidComponent, obstacle))
    {
      insertCommand = CkppInsertSolidComponentCommand::create();
      insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::PARENT_COMPONENT),
				CkppComponentShPtr(modelTree->geometryNode()) );
      insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::INSERTED_COMPONENT),
				CkppComponentShPtr(solid));
      insertCommand->doExecute();
    }
    else
    {
      cerr << "CkppInterface::setObstacleList: obstacle is not a solid component." << endl;
    }
    }
}

bool CkppInterface::isRoadmapStoredAsGraphic(const CkwsRoadmapShPtr& inRoadmap, unsigned int& outRank)
{
  // If no graphic roadmap is mapped to input roadmap, return false.
  if (attRoadmapMapping.count(inRoadmap) == 0) {
    return false;
  }

  // Otherwise, get graphic roadmap
  CkwsGraphicRoadmapShPtr graphicRoadmap = attRoadmapMapping[inRoadmap];

  // Find rank in attGraphicRoadmaps.
  for (unsigned int rank=0; rank<attGraphicRoadmaps.size(); rank++) {
    if (attGraphicRoadmaps[rank] == graphicRoadmap) {
      outRank = rank;
      return true;
    }
  }
  ODEBUG1(":isRoadmapStoredAsGraphic:    input roadmap is mapped to a graphic roadmap ");
  ODEBUG1(":isRoadmapStoredAsGraphic:    not found in attGraphicRoadmaps.");
  return false;
}


void CkppInterface::hppRemoveRoadmapBuilder(const CkitNotificationConstShPtr& inNotification)
{
  // Retrieve hpp::core::Planner object
  hpp::core::Planner *planner = (hpp::core::Planner*)(inNotification->objectPtr< hpp::core::Planner >());

  // Retrieve roadmap that is about to be destroyed
  unsigned int rank = inNotification->unsignedIntValue(hpp::core::Planner::ROADMAP_KEY);

  unsigned int nbProblem = planner->getNbHppProblems();
  if (rank >= nbProblem) {
    ODEBUG1(":hppRemoveRoadmap wrong rank: " << rank << " should be less than " << nbProblem);
    return;
  }

  CkwsRoadmapBuilderShPtr roadmapBuilder = hppPlanner()->roadmapBuilderIthProblem(rank);

  if (!roadmapBuilder) {
    // Nothing to do
    ODEBUG2(":hppRemoveRoadmap: no roadmap builder at given rank.");
    return;
  }

  CkwsRoadmapShPtr roadmap = roadmapBuilder->roadmap();

  if (!roadmap) {
    ODEBUG2(":hppRemoveRoadmap: no roadmap at given rank.");
    return;
  }

  unsigned int roadmapRank;
  if (isRoadmapStoredAsGraphic(roadmap, roadmapRank)) {
    ODEBUG2(":hppRemoveRoadmapBuilder removing roadmap: " << attGraphicRoadmaps[roadmapRank]->name());
    removeGraphicRoadmap(roadmapRank);
    attRoadmapMapping.erase(roadmap);
  }
  return;
}


void CkppInterface::hppAddGraphicRoadmap(const CkitNotificationConstShPtr& inNotification)
{
  // Retrieve hpp::core::Planner object
  hpp::core::Planner *planner = (hpp::core::Planner*)(inNotification->objectPtr< hpp::core::Planner >());

  // Retrieve roadmap that is about to be destroyed
  unsigned int rank = inNotification->unsignedIntValue(hpp::core::Planner::ROADMAP_KEY);

  unsigned int nbProblem = planner->getNbHppProblems();
  if (rank >= nbProblem) {
    ODEBUG1(":hppAddGraphicRoadmap wrong rank: " << rank << " should be less than " << nbProblem);
    return;
  }

  CkwsRoadmapBuilderShPtr roadmapBuilder = hppPlanner()->roadmapBuilderIthProblem(rank);

  if (!roadmapBuilder) {
    // Nothing to do
    ODEBUG1(":hppAddGraphicRoadmap: no roadmap builder at given rank.");
    return;
  }

  // Build graphic roadmap
  std::stringstream roadmapName;
  roadmapName << "graphic roadmap " << rank;

  CkwsGraphicRoadmapShPtr graphicRoadmap =
    CkwsGraphicRoadmap::create(roadmapBuilder, attWeakPtr, roadmapName.str());

  //roadmapBuilder->addDelegate(new CkwsGraphicRoadmapDelegate);
  addGraphicRoadmap(graphicRoadmap, true);

  ODEBUG2(":hppAddGraphicRoadmap: " << roadmapName.str() << " created");
}


void CkppInterface::onIdle(const CkitNotificationConstShPtr& inNotification)
{
  if (corbaServerRunning) {
    attHppCorbaServer->processRequest(false);
  }
}


// ==========================================================================

unsigned int CkppInterface::addGraphicRoadmap(CkwsGraphicRoadmapShPtr inGraphicRoadmap, bool inIsRealTimeUpdated)
{

  inGraphicRoadmap->SetRealTimeUpdate(inIsRealTimeUpdated);

  attGraphicRoadmaps.push_front (inGraphicRoadmap);

  if(inIsRealTimeUpdated){

    ODEBUG2("The Roadmap will be updated at run time");
    attViewGeneral->viewportGraphicMap()->insert(CkppViewGraphicMap::SCENE_3D, attGraphicRoadmaps.back());
    CkitNotificator::defaultNotificator()->
      subscribe<CkppInterface>(CkwsGraphicRoadmapDelegate::DID_MODIFY_THE_ROADMAP,
			       this ,
			       &CkppInterface::graphicRoadmapHasBeenModified);

  }
  else{
    ODEBUG2("The Roadmap will be updated at the end of building");
    attViewGeneral->viewportGraphicMap()->insert( CkppViewGraphicMap::OVERLAY_3D, attGraphicRoadmaps.back());
    CkitNotificator::defaultNotificator()->
      subscribe< CkppInterface >(CkppPlanPathCommand::DID_FINISH_BUILDING,
				 this, &CkppInterface::graphicRoadmapHasBeenModified);
  }

#if 0
  /*
     Do not delete all roadmap and problems each time somthing has changed since
     this causes erratic behaviors using hppCorbaServer
  */
  CkitNotificator::defaultNotificator()->subscribe< CkppInterface >(CkppWindowController::DID_CHANGE_DOCUMENT,
								    this,&CkppInterface::removeAllRoadmapsAndProblems);
#endif

  // Get CkwsRoadmap object corresponding to input graphic roadmap
  // and store it in mapping.
  CkwsRoadmapShPtr roadmap = inGraphicRoadmap->kwsRoadmap();
  attRoadmapMapping[roadmap] = inGraphicRoadmap;


  return attGraphicRoadmaps.size();

}

// ==========================================================================

void CkppInterface::initializeViewGeneral (const CkppViewGeneralShPtr &i_viewGeneral)
{
	attViewGeneral = i_viewGeneral;
}

// ==========================================================================

void CkppInterface::removeGraphicRoadmap( int inRank){

  if(inRank >= (int)attGraphicRoadmaps.size()) {
    ODEBUG2("The rank "<< inRank <<" is too high ( size of roadmaps vector is "
	    << attGraphicRoadmaps.size()<<" )");
  }
  else if(inRank < 0){

    for(unsigned int i=0; i<attGraphicRoadmaps.size();i++ ){

      if(attGraphicRoadmaps[i]->isDisplayed()) attViewGeneral->viewportGraphicMap()->remove( CkppViewGraphicMap::OVERLAY_3D, attGraphicRoadmaps[i]);
      attGraphicRoadmaps.erase(attGraphicRoadmaps.begin()+i);

    }

  }
  else{

    if(attGraphicRoadmaps[inRank]->isDisplayed()) attViewGeneral->viewportGraphicMap()->remove( CkppViewGraphicMap::OVERLAY_3D, attGraphicRoadmaps[inRank]);
    attGraphicRoadmaps.erase(attGraphicRoadmaps.begin()+inRank);

  }

}


// ==========================================================================

void CkppInterface::showRoadmap(unsigned int inRank){

  if(!attGraphicRoadmaps[inRank]->isDisplayed()){

    attViewGeneral->viewportGraphicMap()->insert( CkppViewGraphicMap::OVERLAY_3D, attGraphicRoadmaps[inRank] );
    attGraphicRoadmaps[inRank]->isDisplayed(true);
    mainWindowController()->graphicWindowController()->viewWindow()->redraw(CkppViewCanvas::NOW);

  }else {
    ODEBUG1("This roadmap is already displayed");
  }
}


// ==========================================================================

void CkppInterface::hideRoadmap(unsigned int inRank){

  if(attGraphicRoadmaps[inRank]->isDisplayed()){

    attViewGeneral->viewportGraphicMap()->remove( CkppViewGraphicMap::OVERLAY_3D, attGraphicRoadmaps[inRank]);
    attGraphicRoadmaps[inRank]->isDisplayed(false);
    mainWindowController()->graphicWindowController()->viewWindow()->redraw(CkppViewCanvas::NOW);

  } else {
    ODEBUG1("This roadmap is already hidden");
  }
}

// ==========================================================================

void CkppInterface::graphicRoadmapHasBeenModified(const CkitNotificationConstShPtr& inNotification)
{
  ODEBUG2(":graphicRoadmapHasBeenModified called");
  bool found = false;

  CkwsRoadmapBuilderConstShPtr rdmBuilder =
    inNotification->constShPtrValue<CkwsRoadmapBuilder>(CkwsGraphicRoadmapDelegate::ROADMAPBUILDER_KEY);

  if(rdmBuilder) {
    /* We look in the vector for the graphic roadmap that has his intern
       kwsRoadmap shared pointer identical to the modified roadmap */
    for(std::deque<CkwsGraphicRoadmapShPtr>::iterator it = attGraphicRoadmaps.begin() ;
	it<attGraphicRoadmaps.end() ; it++) {
      if((*it)->kwsRoadmap() == rdmBuilder->roadmap()){
	found = true;
	(*it)->drawNotifRoadmap(inNotification);
      }
    }

    if(!found) {
      ODEBUG1(":graphicRoadmapHasBeenModified:    No graphical roadmap for the modified kwsRoadmap,");
      ODEBUG1(":graphicRoadmapHasBeenModified:    " << attGraphicRoadmaps.size() <<" roadmaps in vector.");
    }
  }
  else {
    ODEBUG1(":graphicRoadmapHasBeenModified: no roadmap builder in notification");
  }
}

// ==========================================================================

void CkppInterface::removeAllRoadmapsAndProblems(const CkitNotificationConstShPtr& inNotification)
{

  ODEBUG1("removing all roadmaps and all problems");
  removeGraphicRoadmap();
  attGraphicRoadmaps.clear();

  /* Remove all problems in hpp::core::Planner object. */
  while(KD_OK==hppPlanner()->removeHppProblem()){}

}

// ------------- initializing module ------------------
// extern Function : First called by KineoWorks to Load the Module

int initializeModule(CkppModuleInterfaceShPtr& o_moduleInterface)
{
  std::cerr << " initializing module ... " << std::endl;
  o_moduleInterface = CkppInterface::create();
  return 0;
}
