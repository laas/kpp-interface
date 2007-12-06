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


#include "kppInterface/kppCommandInit.h"
#include "kppInterface/kwsGraphicRoadmap.h"
#include "kppInterface/kwsGraphicRoadmapDelegate.h"
#include "hppRRTPlanner/hppRRTPlanner.h"

#include "KineoController/kppSetPropertyCommand.h"
#include "KineoModel/kppComponentParameter.h"
#include "KineoModel/kppValue.h"
#include "KineoModel/kppComponentClassFilter.h"
#include "KineoModel/kppComponent.h"
#include "KineoModel/kppSolidComponent.h"

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

CkppCommandInit::CkppCommandInit(const CkppCommandInit& i_command) :
  CkppCommand(i_command)
{
  attKpp = i_command.attKpp;
}


// ==========================================================================

CkppCommandInit::~CkppCommandInit()
{
  
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

CkppCommandInitShPtr CkppCommandInit::createCopy(const CkppCommandInitConstShPtr& i_command)
{
  CkppCommandInit*  ptr = new CkppCommandInit(*i_command);
  CkppCommandInitShPtr shPtr(ptr);
  
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }
  
  return shPtr;
}


// ==========================================================================

ktStatus CkppCommandInit::init(const CkppCommandInitWkPtr& i_weakPtr)
{
  ktStatus success = CkppCommand::init(i_weakPtr);
  
  if(KD_OK == success)
	{
	  m_weakPtr = i_weakPtr;
	}

  return success;
}


// ==========================================================================


CkppCommandShPtr CkppCommandInit::clone() const
{
  return CkppCommandInit::createCopy(m_weakPtr.lock());
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

CkppParameterConstShPtr CkppCommandInit::parameter(unsigned int i_rank) const
{
 
  CkppParameterShPtr result;
  CkppComponentShPtr nullComponent;

  switch(i_rank)
    {   
    case MODELTREE : 
      result = CkppComponentParameter::create( "Model Tree", CkppComponentClassFilter< CkppModelTree >());
      break;
    case PATH : 
      result = CkppComponentParameter::create( "Path", CkppComponentClassFilter< CkppPathComponent >());
      break;
    case DEVICE : 
      result = CkppComponentParameter::create( "Device", CkppComponentClassFilter< CkppDeviceComponent >());
      break;
    default:
      KPP_ASSERT( false );
    }
  
  return result;
}


// ==========================================================================

ktStatus CkppCommandInit::doExecute()
{

  // write here what your command does
  CkppDeviceComponentShPtr deviceComponent;

  deviceComponent = KIT_DYNAMIC_PTR_CAST(CkppDeviceComponent,paramValue(parameter(DEVICE)).componentValue());
    

  attKpp->hppPlanner()->addHppProblem(deviceComponent);

  return KD_OK ;
 
}


// ==========================================================================
