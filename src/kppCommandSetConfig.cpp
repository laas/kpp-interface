// COPYRIGHT 


/*****************************************
 INCLUDES
*******************************************/


#include "kppInterface/kppCommandSetConfig.h"

#include "KineoWorks2/kwsPath.h"
#include "KineoController/kppSetPropertyCommand.h"
#include "KineoModel/kppComponentParameter.h"
#include "KineoModel/kppValue.h"
#include "KineoModel/kppComponentClassFilter.h"
#include "KineoModel/kppComponent.h"
#include "KineoModel/kppSolidComponent.h"

#include "KineoModel/kppPathNode.h"
#include "KineoModel/kppPathComponent.h"

#include "KineoController/kppDocument.h"
#include "KineoGUI/kppMainWindowController.h" 

#include "KineoModel/kppModelTree.h"

#include "hpp/core/planner.hh"

/*****************************************
 DEFINES
*******************************************/

/*****************************************
 METHODS
*******************************************/



// ==========================================================================

CkppCommandSetConfig::CkppCommandSetConfig(CkppInterface *kpp)
{
  attKpp = kpp;
}


// ==========================================================================

CkppCommandSetConfig::CkppCommandSetConfig(const CkppCommandSetConfig& inCommand) :
  CkppCommand(inCommand)
{
  attKpp = inCommand.attKpp;
}


// ==========================================================================

CkppCommandSetConfig::~CkppCommandSetConfig()
{
  attKpp = NULL;
  attWeakPtr.reset();
}


// ==========================================================================

CkppCommandSetConfigShPtr CkppCommandSetConfig::create(CkppInterface *kpp)
{
  CkppCommandSetConfig*  ptr = new CkppCommandSetConfig(kpp);
  CkppCommandSetConfigShPtr shPtr(ptr);
  
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }

  return shPtr;
}

// ==========================================================================

CkppCommandSetConfigShPtr CkppCommandSetConfig::createCopy(const CkppCommandSetConfigConstShPtr& inCommand)
{
  CkppCommandSetConfig*  ptr = new CkppCommandSetConfig(*inCommand);
  CkppCommandSetConfigShPtr shPtr(ptr);
  
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }
  
  return shPtr;
}


// ==========================================================================

ktStatus CkppCommandSetConfig::init(const CkppCommandSetConfigWkPtr& inWeakPtr)
{
  ktStatus success = CkppCommand::init(inWeakPtr);
  
  if(KD_OK == success)
	{
	  attWeakPtr = inWeakPtr;
	}

  return success;
}


// ==========================================================================


CkppCommandShPtr CkppCommandSetConfig::clone() const
{
  return CkppCommandSetConfig::createCopy(attWeakPtr.lock());
}


// ==========================================================================

bool CkppCommandSetConfig::isUndoable() const
{
  return true;
}


// ==========================================================================

unsigned int CkppCommandSetConfig::countParameters() const
{
  return PARAMETER_COUNT;
}


// ==========================================================================

CkppParameterConstShPtr CkppCommandSetConfig::parameter(unsigned int inRank) const
{
 
  CkppParameterShPtr result;
  CkppComponentShPtr nullComponent;

  switch(inRank) {   
  case PATH:
    result = CkppComponentParameter::create("selected path", CkppComponentClassFilter<CkppPathComponent>());
    break;
  default:
    KIT_ASSERT( false );
  }
  
  return result;
}


// ==========================================================================

ktStatus CkppCommandSetConfig::doExecute()
{
  CkppComponentShPtr component(paramValue(parameter(PATH)).componentValue());

  CkppPathComponentShPtr pathComponent = KIT_DYNAMIC_PTR_CAST(CkppPathComponent, component);

  if (!pathComponent){
    std::cerr << "ERROR - CkppCommandSetConfig::doExecute() : could NOT find the path  " << std::endl ;
    return KD_ERROR ;
  }

  CkwsConfigShPtr initConfig ;
  initConfig =  pathComponent->kwsPath()->configAtStart() ;
  
  if(!initConfig){
    std::cerr << "ERROR - CkppCommandSetConfig::doExecute() : INIT config does NOT EXIST because there is NO directPath FOUND" << std::endl ;
    return KD_ERROR;
  }
  
  attKpp->hppPlanner()->initConfIthProblem(attKpp->hppPlanner()->getNbHppProblems()-1, initConfig);


  CkwsConfigShPtr goalConfig ;
  goalConfig =  pathComponent->kwsPath()->configAtEnd() ;

  if(!goalConfig){
    std::cerr << "ERROR - CkppCommandSetConfig::doExecute() : GOAL config doest NOT EXIST because there is NO directPath FOUND" << std::endl ;
    return KD_ERROR ;
  }

  attKpp->hppPlanner()->goalConfIthProblem(attKpp->hppPlanner()->getNbHppProblems()-1, goalConfig);

  return KD_OK ;
 
}


// ==========================================================================
