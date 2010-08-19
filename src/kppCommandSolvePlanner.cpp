

/*****************************************
 INCLUDES
*******************************************/


#include "kppInterface/kppCommandSolvePlanner.h"

#include "KineoController/kppSetPropertyCommand.h"
#include "KineoModel/kppComponentParameter.h"
#include "KineoModel/kppValue.h"
#include "KineoModel/kppComponentClassFilter.h"
#include "KineoModel/kppComponent.h"
#include "KineoModel/kppSolidComponent.h"
#include "KineoModel/kppConfigComponent.h"

#include "hpp/core/planner.hh"

/*****************************************
 DEFINES
*******************************************/

/*****************************************
 METHODS
*******************************************/



// ==========================================================================

CkppCommandSolvePlanner::CkppCommandSolvePlanner(CkppInterface *kpp)
{
  attKpp = kpp;
}


// ==========================================================================

CkppCommandSolvePlanner::CkppCommandSolvePlanner(const CkppCommandSolvePlanner& inCommand) :
  CkppCommand(inCommand)
{
  attKpp = inCommand.attKpp;
}


// ==========================================================================

CkppCommandSolvePlanner::~CkppCommandSolvePlanner()
{
  
}


// ==========================================================================

CkppCommandSolvePlannerShPtr CkppCommandSolvePlanner::create(CkppInterface *kpp)
{
  CkppCommandSolvePlanner*  ptr = new CkppCommandSolvePlanner(kpp);
  CkppCommandSolvePlannerShPtr shPtr(ptr);
  
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }

  return shPtr;
}

// ==========================================================================

CkppCommandSolvePlannerShPtr CkppCommandSolvePlanner::createCopy(const CkppCommandSolvePlannerConstShPtr& inCommand)
{
  CkppCommandSolvePlanner*  ptr = new CkppCommandSolvePlanner(*inCommand);
  CkppCommandSolvePlannerShPtr shPtr(ptr);
  
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }
  
  return shPtr;
}


// ==========================================================================

ktStatus CkppCommandSolvePlanner::init(const CkppCommandSolvePlannerWkPtr& inWeakPtr)
{
  ktStatus success = CkppCommand::init(inWeakPtr);
  
  if(KD_OK == success)
	{
	  attWeakPtr = inWeakPtr;
	}

  return success;
}


// ==========================================================================


CkppCommandShPtr CkppCommandSolvePlanner::clone() const
{
  return CkppCommandSolvePlanner::createCopy(attWeakPtr.lock());
}


// ==========================================================================

bool CkppCommandSolvePlanner::isUndoable() const
{
  return true;
}


// ==========================================================================

unsigned int CkppCommandSolvePlanner::countParameters() const
{
  return PARAMETER_COUNT;
}


// ==========================================================================

CkppParameterConstShPtr CkppCommandSolvePlanner::parameter(unsigned int inRank) const
{
 
  CkppParameterShPtr result;
  CkppComponentShPtr nullComponent;

  switch(inRank)
    {   
    default:
      KIT_ASSERT( false );
    }
  
  return result;
}


// ==========================================================================


ktStatus CkppCommandSolvePlanner::doExecute()
{
  if (attKpp->hppPlanner() == NULL) {
    std::cerr << "You need to create a Planner object first." << std::endl;
    return KD_ERROR;
  }

   // set config  x y theta
  //attKpp->hppPlanner()->initConfig(1.54724, -0.915675, 2.0899) ;
  //attKpp->hppPlanner()->goalConfig( -1.0892,  1.46143, -2.92461);

  attKpp->hppPlanner()->solve() ;

  return KD_OK ;
 
}


// ==========================================================================
