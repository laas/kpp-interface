//
// Copyright (c) 2005, 2006, 2007, 2008, 2009, 2010, 2011, 2012, 2013 CNRS
// Authors: Florent Lamiraux
//
// This file is part of kpp-interface
// kpp-interface is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// kpp-interface is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// kpp-interface  If not, see
// <http://www.gnu.org/licenses/>.

#include "kpp/interface/command-start-corbaserver.hh"
#include "kpp/interface/interface.hh"

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

#include "hpp/core/planner.hh"

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

CkppCommandStartCorbaServer::CkppCommandStartCorbaServer(const CkppCommandStartCorbaServer& inCommand) :
  CkppCommand(inCommand)
{
  attKpp = inCommand.attKpp;
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

CkppCommandStartCorbaServerShPtr CkppCommandStartCorbaServer::createCopy(const CkppCommandStartCorbaServerConstShPtr& inCommand)
{
  CkppCommandStartCorbaServer*	ptr = new CkppCommandStartCorbaServer(*inCommand);
  CkppCommandStartCorbaServerShPtr		shPtr(ptr);
  ptr->attCommandStr = inCommand->attCommandStr;
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }

  return shPtr;
}

ktStatus CkppCommandStartCorbaServer::init(const CkppCommandStartCorbaServerWkPtr& inWeakPtr)
{
  ktStatus		success = CkppCommand::init(inWeakPtr);

  if(KD_OK == success)
    {
      attWeakPtr = inWeakPtr;
    }

  return success;
}

CkppCommandShPtr CkppCommandStartCorbaServer::clone() const
{
  return CkppCommandStartCorbaServer::createCopy(attWeakPtr.lock());
}

bool CkppCommandStartCorbaServer::isUndoable() const
{
  return true;
}

unsigned int CkppCommandStartCorbaServer::countParameters() const
{
  return PARAMETER_COUNT;
}

CkppParameterConstShPtr CkppCommandStartCorbaServer::parameter(unsigned int inRank) const
{
  CkppParameterShPtr				result;
  CkppComponentShPtr				nullComponent;

  switch(inRank)
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

