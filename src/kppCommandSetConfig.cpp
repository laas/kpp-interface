// COPYRIGHT 


/*****************************************
 INCLUDES
*******************************************/


#include "kppInterface/kppCommandSetConfig.h"

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

CkppCommandSetConfig::CkppCommandSetConfig(const CkppCommandSetConfig& i_command) :
  CkppCommand(i_command)
{
  attKpp = i_command.attKpp;
}


// ==========================================================================

CkppCommandSetConfig::~CkppCommandSetConfig()
{
  attKpp = NULL;
  m_weakPtr.reset();
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

CkppCommandSetConfigShPtr CkppCommandSetConfig::createCopy(const CkppCommandSetConfigConstShPtr& i_command)
{
  CkppCommandSetConfig*  ptr = new CkppCommandSetConfig(*i_command);
  CkppCommandSetConfigShPtr shPtr(ptr);
  
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }
  
  return shPtr;
}


// ==========================================================================

ktStatus CkppCommandSetConfig::init(const CkppCommandSetConfigWkPtr& i_weakPtr)
{
  ktStatus success = CkppCommand::init(i_weakPtr);
  
  if(KD_OK == success)
	{
	  m_weakPtr = i_weakPtr;
	}

  return success;
}


// ==========================================================================


CkppCommandShPtr CkppCommandSetConfig::clone() const
{
  return CkppCommandSetConfig::createCopy(m_weakPtr.lock());
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

CkppParameterConstShPtr CkppCommandSetConfig::parameter(unsigned int i_rank) const
{
 
  CkppParameterShPtr result;
  CkppComponentShPtr nullComponent;

  switch(i_rank) {   
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
    cerr << "ERROR - CkppCommandSetConfig::doExecute() : could NOT find the path  " << endl ;
    return KD_ERROR ;
  }

  CkwsConfigShPtr initConfig ;
  initConfig =  pathComponent->kwsPath()->configAtStart() ;
  
  if(!initConfig){
    cerr << "ERROR - CkppCommandSetConfig::doExecute() : INIT config does NOT EXIST because there is NO directPath FOUND" << endl ;
    return KD_ERROR;
  }
  
  attKpp->hppPlanner()->initConfIthProblem(attKpp->hppPlanner()->getNbHppProblems()-1, initConfig);


  CkwsConfigShPtr goalConfig ;
  goalConfig =  pathComponent->kwsPath()->configAtEnd() ;

  if(!goalConfig){
    cerr << "ERROR - CkppCommandSetConfig::doExecute() : GOAL config doest NOT EXIST because there is NO directPath FOUND" << endl ;
    return KD_ERROR ;
  }

  attKpp->hppPlanner()->goalConfIthProblem(attKpp->hppPlanner()->getNbHppProblems()-1, goalConfig);

  return KD_OK ;
 
}


// ==========================================================================
