// COPYRIGHT 

/*
*
*
*  DO NOT WRITE IN THIS FILE
*
*
*
*/



/*****************************************
 INCLUDES
*******************************************/

#include "KineoController/kppSetPropertyCommand.h"
#include "KineoModel/kppComponentParameter.h"
#include "KineoModel/kppValue.h"
#include "KineoModel/kppComponentClassFilter.h"
#include "KineoModel/kppComponent.h"
#include "KineoModel/kppSolidComponent.h"

#include "hpp/core/planner.hh"

#include "kppInterface/kppCommandInit.h"
#include "kppInterface/kwsGraphicRoadmap.h"
#include "kppInterface/kwsGraphicRoadmapDelegate.h"

/*****************************************
 DEFINES
*******************************************/

/*****************************************
 METHODS
*******************************************/



// ==========================================================================

CkppCommandInit::CkppCommandInit(CkppInterface *kpp)
{
  attKpp = kpp;
}


// ==========================================================================

CkppCommandInit::CkppCommandInit(const CkppCommandInit& inCommand) :
  CkppCommand(inCommand)
{
  attKpp = inCommand.attKpp;
}


// ==========================================================================

CkppCommandInit::~CkppCommandInit()
{
  attKpp = NULL;
}


// ==========================================================================

CkppCommandInitShPtr CkppCommandInit::create(CkppInterface *kpp)
{
  CkppCommandInit*  ptr = new CkppCommandInit(kpp);
  CkppCommandInitShPtr shPtr(ptr);
  
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }

  return shPtr;
}

// ==========================================================================

CkppCommandInitShPtr CkppCommandInit::createCopy(const CkppCommandInitConstShPtr& inCommand)
{
  CkppCommandInit*  ptr = new CkppCommandInit(*inCommand);
  CkppCommandInitShPtr shPtr(ptr);
  
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }
  
  return shPtr;
}


// ==========================================================================

ktStatus CkppCommandInit::init(const CkppCommandInitWkPtr& inWeakPtr)
{
  ktStatus success = CkppCommand::init(inWeakPtr);
  
  if(KD_OK == success)
	{
	  attWeakPtr = inWeakPtr;
	}

  return success;
}


// ==========================================================================


CkppCommandShPtr CkppCommandInit::clone() const
{
  return CkppCommandInit::createCopy(attWeakPtr.lock());
}


// ==========================================================================

bool CkppCommandInit::isUndoable() const
{
  return true;
}


// ==========================================================================

unsigned int CkppCommandInit::countParameters() const
{
  return PARAMETER_COUNT;
}


// ==========================================================================

CkppParameterConstShPtr CkppCommandInit::parameter(unsigned int inRank) const
{
 
  CkppParameterShPtr result;
  CkppComponentShPtr nullComponent;

  switch(inRank)
    {   
    case DEVICE : 
      result = CkppComponentParameter::create( "Device", CkppComponentClassFilter< CkppDeviceComponent >());
      break;
    default:
      KIT_ASSERT( false );
    }
  
  return result;
}


// ==========================================================================

ktStatus CkppCommandInit::doExecute()
{

  // write here what your command does
  CkppDeviceComponentShPtr deviceComponent;

  deviceComponent = KIT_DYNAMIC_PTR_CAST(CkppDeviceComponent,paramValue(parameter(DEVICE)).componentValue());
    

  attKpp->hppPlanner()->addHppProblemAtBeginning(deviceComponent, 0.02);

  return KD_OK ;
 
}


// ==========================================================================
