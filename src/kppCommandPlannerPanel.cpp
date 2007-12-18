// COPYRIGHT 


/*****************************************
 INCLUDES
*******************************************/

#include "kppInterface/kppCommandPlannerPanel.h"
#include "kppInterface/kppPlannerPanelController.h"
#include "kppInterface/kppPlannerPanel.h"

#include "KineoController/kppSetPropertyCommand.h"
#include "KineoModel/kppComponentParameter.h"
#include "KineoModel/kppValue.h"
#include "KineoModel/kppComponentClassFilter.h"
#include "KineoModel/kppComponent.h"
#include "KineoModel/kppSolidComponent.h"
#include "KineoGUI/kppMainWindowController.h"

/*****************************************
 DEFINES
*******************************************/

KIT_PREDEF_CLASS( CkppPlannerPanelController );

/*****************************************
 METHODS
*******************************************/



// ==========================================================================

CkppCommandPlannerPanel::CkppCommandPlannerPanel(CkppInterface *kpp,const CkppMainWindowUICommandFactoryConstShPtr& i_commandFactory_arg)
{
  attKpp = kpp;
  i_commandFactory = i_commandFactory_arg;
}


// ==========================================================================

CkppCommandPlannerPanel::CkppCommandPlannerPanel(const CkppCommandPlannerPanel& i_command) :
  CkppPlanPathCommand(i_command)
{
  attKpp = i_command.attKpp;
  commandPlannerPanel = i_command.commandPlannerPanel;
  i_commandFactory = i_command.i_commandFactory;
}


// ==========================================================================

CkppCommandPlannerPanel::~CkppCommandPlannerPanel()
{
  attKpp = NULL;
}


// ==========================================================================

CkppCommandPlannerPanelShPtr CkppCommandPlannerPanel::create(CkppInterface *kpp,const CkppMainWindowUICommandFactoryConstShPtr& i_commandFactory)
{
  CkppCommandPlannerPanel*  ptr = new CkppCommandPlannerPanel(kpp,i_commandFactory);
  CkppCommandPlannerPanelShPtr shPtr(ptr);
  
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }

  return shPtr;
}

// ==========================================================================

CkppCommandPlannerPanelShPtr CkppCommandPlannerPanel::createCopy(const CkppCommandPlannerPanelConstShPtr& i_command)
{
  CkppCommandPlannerPanel*  ptr = new CkppCommandPlannerPanel(*i_command);
  CkppCommandPlannerPanelShPtr shPtr(ptr);
  
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }
  
  return shPtr;
}


// ==========================================================================

ktStatus CkppCommandPlannerPanel::init(const CkppCommandPlannerPanelWkPtr& i_weakPtr)
{
  ktStatus success = CkppPlanPathCommand::init(i_weakPtr);
  
  if(KD_OK == success)
	{
	  m_weakPtr = i_weakPtr;
	}

  return success;
}


// ==========================================================================


CkppCommandShPtr CkppCommandPlannerPanel::clone() const
{
  return CkppCommandPlannerPanel::createCopy(m_weakPtr.lock());
}


// ==========================================================================

bool CkppCommandPlannerPanel::isUndoable() const
{
  return true;
}


// ==========================================================================

unsigned int CkppCommandPlannerPanel::countParameters() const
{
  return PARAMETER_COUNT;
}


// ==========================================================================

CkppParameterConstShPtr CkppCommandPlannerPanel::parameter(unsigned int i_rank) const
{
 
  CkppParameterShPtr result;
  CkppComponentShPtr nullComponent;

  switch(i_rank)
    {  
/*    case MODELTREE : 
      result = CkppComponentParameter::create( "Model Tree", CkppComponentClassFilter< CkppModelTree >());
      break;
    case PATH : 
      result = CkppComponentParameter::create( "Path", CkppComponentClassFilter< CkppPathComponent >());
      break;
    case DEVICE : 
      result = CkppComponentParameter::create( "Device", CkppComponentClassFilter< CkppDeviceComponent >());
      break; */
    default:
      KPP_ASSERT( false );
    }
  
  return result;
}


// ==========================================================================

ktStatus CkppCommandPlannerPanel::doExecute()
{

  // write here what your command does

//---------- Initializing Problem ------------------------------------------------------
/*  CkppModelTreeShPtr modelTree(KIT_DYNAMIC_PTR_CAST(CkppModelTree,paramValue(parameter(MODELTREE)).componentValue()));
  CkppPathComponentShPtr pathComponent;
  CkppDeviceComponentShPtr deviceComponent;

  if(modelTree->deviceNode()){
    deviceComponent = KIT_DYNAMIC_PTR_CAST(CkppDeviceComponent,paramValue(parameter(DEVICE)).componentValue());
  }

  cout<<"retreiving parameters - DONE"<<endl;

  attKpp->hppPlanner()->initializeProblem();//deviceComponent); 

  cout<<"initialize problem - DONE"<<endl;

  CkwsGraphicRoadmapShPtr kwsGraphicRoadmap = CkwsGraphicRoadmap::create(attKpp->hppPlanner()->roadmapBuilderIthProblem(0),"Graphic Roadmap") ;

  attKpp->addGraphicRoadmap(kwsGraphicRoadmap,true);

  cout<<"create a graphic roadmap - DONE"<<endl;
//----------------------------------------------------------------------------------------


//----------- Set Configs ----------------------------------------------------------------  
  CkppPathNodeShPtr pathNode = modelTree->pathNode() ;
  unsigned int rank=0 ;  

  if(pathNode->countChildComponents() == 0){
    cerr << "ERROR - CkppCommandSetConfig::doExecute() : there is NO PATH yet  " << endl ;
    return KD_ERROR ;
  }
 
 
   pathComponent = KIT_DYNAMIC_PTR_CAST( CkppPathComponent, pathNode->childComponent(pathNode->countChildComponents()-1)); 

  if (!pathComponent){
    cerr << "ERROR - CkppCommandSetConfig::doExecute() : could NOT find the path  " << endl ;
    return KD_ERROR ;
  }

  CkwsConfigShPtr initConfig ;
  initConfig =  pathComponent->kwsPath()->configAtStart() ;
  
  if(!initConfig){
    cerr << "ERROR - CkppCommandSetConfig::doExecute() : INIT config not found" << endl ;
    return KD_ERROR;
  }
   
  //set the Init Config of the problem 
  if (attKpp->hppPlanner()->initConfIthProblem(0, initConfig) == KD_ERROR) {
    std::cerr << "ERROR - ChppRRTPlanner::initConfig could not set initial configuration of robot box." << std::endl;
    return KD_ERROR;
  }
      
  CkwsConfigShPtr goalConfig ;
  goalConfig =  pathComponent->kwsPath()->configAtEnd() ;

  if(!goalConfig){
    cerr << "ERROR - CkppCommandSetConfig::doExecute() : GOAL config not found" << endl ;
    return KD_ERROR ;
  }
   
  //set the Goal Config of the problem  
  if (attKpp->hppPlanner()->goalConfIthProblem(0, goalConfig) == KD_ERROR) {
    std::cerr << "ERROR - ChppRRTPlanner::goalConfig : could not set initial configuration of robot box." << std::endl;
    return KD_ERROR;
  }*/
//----------------------------------------------------------------------------------------


//----------- Displaying Panel -----------------------------------------------------------  
  commandPlannerPanel = i_commandFactory->makeShowPanelUICommand(
							     CkppDefaultCreateFactory< CkppWindowControllerShPtr, CkppPlannerPanelController >::create(),
							     "Planner Configuration", //leave empty and the panel window title will be used (optional)
							     "Displays a configuration panel for planners", //help string (optional)
							     "" //use CkppModuleInterface::pathToResourceFile() if called from an Add-On (needed for toolbar commands only)
							     );
  
  if(commandPlannerPanel->execute() == KD_ERROR) cout<<"Error Executing Command !"<<endl; 
  //to do this, we must be sure that the current panel controller is a PlannerPanelController : currently guaranteed by the "execute" command on the previous line
  CkppPlannerPanelControllerShPtr panelController = KIT_DYNAMIC_PTR_CAST(CkppPlannerPanelController,i_commandFactory->mainWindowController()->panelController());
  CkwsDiffusingRdmBuilderShPtr RRT = KIT_DYNAMIC_PTR_CAST(CkwsDiffusingRdmBuilder,attKpp->hppPlanner()->roadmapBuilderIthProblem(0));
  panelController->getPanel()->setRdmBuilder(RRT); 
  panelController->getPanel()->setInterface(attKpp);

//------------------------------------------------------------------------------------------

  return KD_OK ;
 
}


// ==========================================================================
