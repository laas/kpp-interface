// COPYRIGHT KINEOCAM 2007

#include "kppInterface/kppCommandStartCorbaServer.h"
#include "kppInterface/kppInterface.h"

#include "KineoController/kppSetPropertyCommand.h"
#include "KineoModel/kppComponentParameter.h"
#include "KineoModel/kppValue.h"
#include "KineoModel/kppComponentClassFilter.h"
#include "KineoModel/kppComponent.h"
#include "KineoModel/kppSolidComponent.h"

#include "KineoModel/kppDeviceComponent.h"
#include "KineoModel/kppAnchorJointComponent.h"
#include "KineoModel/kppFreeFlyerJointComponent.h"
#include "KineoModel/kppRotationJointComponent.h"
#include "KineoModel/kppSolidComponentRef.h"
#include "KineoModel/kppModelTree.h"
#include "KineoModel/kppGeometryNode.h"
#include "KineoModel/kppDeviceNode.h"
#include "KineoWorks2/kwsJointDof.h"

#include "hppCore/hppPlanner.h"

#include "KineoController/kppInsertComponentCommand.h"

#include "KineoUtility/kitNotificator.h"

using namespace std;

#include <iostream>


CkppCommandStartCorbaServer::CkppCommandStartCorbaServer(CkppInterface *kpp)
{
  attKpp = kpp;
  // no op

  // attKpp->printTest(3);
    
}

CkppCommandStartCorbaServer::CkppCommandStartCorbaServer(const CkppCommandStartCorbaServer& i_command) :
  CkppCommand(i_command)
{
  attKpp = i_command.attKpp;
  // no op
}

CkppCommandStartCorbaServer::~CkppCommandStartCorbaServer()
{
  // no op
}

CkppCommandStartCorbaServerShPtr CkppCommandStartCorbaServer::create(CkppInterface *kpp)
{
  CkppCommandStartCorbaServer*	ptr = new CkppCommandStartCorbaServer(kpp);
  CkppCommandStartCorbaServerShPtr shPtr(ptr);
	
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }

  return shPtr;
}

CkppCommandStartCorbaServerShPtr CkppCommandStartCorbaServer::createCopy(const CkppCommandStartCorbaServerConstShPtr& i_command)
{
  CkppCommandStartCorbaServer*	ptr = new CkppCommandStartCorbaServer(*i_command);
  CkppCommandStartCorbaServerShPtr		shPtr(ptr);
  ptr->attCommandStr = i_command->attCommandStr;
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }

  return shPtr;
}

ktStatus CkppCommandStartCorbaServer::init(const CkppCommandStartCorbaServerWkPtr& i_weakPtr)
{
  ktStatus		success = CkppCommand::init(i_weakPtr);

  if(KD_OK == success)
    {
      m_weakPtr = i_weakPtr;
    }

  return success;
}

CkppCommandShPtr CkppCommandStartCorbaServer::clone() const
{
  return CkppCommandStartCorbaServer::createCopy(m_weakPtr.lock());
}

bool CkppCommandStartCorbaServer::isUndoable() const
{
  return true;
}

unsigned int CkppCommandStartCorbaServer::countParameters() const
{
  return PARAMETER_COUNT;
}

CkppParameterConstShPtr CkppCommandStartCorbaServer::parameter(unsigned int i_rank) const
{
  CkppParameterShPtr				result;
  CkppComponentShPtr				nullComponent;

  switch(i_rank)
    {
    case MODEL_TREE:
      result = CkppComponentParameter::create(
					      "model tree",
					      CkppComponentClassFilter< CkppModelTree >());
      break;

    default:
      KIT_ASSERT( false );
    }

  return result;
}

ktStatus CkppCommandStartCorbaServer::doExecute()
{
  return attKpp->startCorbaServer();
}


// -------------- execute commands ----------------

