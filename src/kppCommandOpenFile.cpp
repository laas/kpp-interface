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

#include "kpp/interface/command-openfile.hh" 

#include "KineoModel/kppComponentParameter.h"
#include "KineoModel/kppGeometryNode.h"
#include "KineoModel/kppComponentClassFilter.h"
#include "KineoModel/kppFileParameter.h"

#include "KineoKCDModel/kppKCDPolyhedron.h"

#include "kprParserXML/kprParserManager.h"

#include <iostream>
using namespace std ;

CkppCommandOpenFile::CkppCommandOpenFile(CkppInterface * inInterface)
{

  attInterface = inInterface;

}
CkppCommandOpenFile::CkppCommandOpenFile(const CkppCommandOpenFile& inCommand) :
  CkppCommand(inCommand)
{

  attInterface = inCommand.attInterface;

}
CkppCommandOpenFile::~CkppCommandOpenFile()
{
  attInterface = NULL;
}
CkppCommandOpenFileShPtr CkppCommandOpenFile::create(CkppInterface * inInterface)
{
  CkppCommandOpenFile*  ptr = new CkppCommandOpenFile(inInterface);
  CkppCommandOpenFileShPtr shPtr(ptr);
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }
  return shPtr;
}
CkppCommandOpenFileShPtr CkppCommandOpenFile::createCopy(const CkppCommandOpenFileConstShPtr& inCommand)
{
  CkppCommandOpenFile*  ptr = new CkppCommandOpenFile(*inCommand);
  CkppCommandOpenFileShPtr shPtr(ptr);
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }
  return shPtr;
}
ktStatus CkppCommandOpenFile::init(const CkppCommandOpenFileWkPtr& inWeakPtr)
{
  ktStatus success = CkppCommand::init(inWeakPtr);
  if(KD_OK == success)
	{
	  attWeakPtr = inWeakPtr;
	}
  return success;
}
CkppCommandShPtr CkppCommandOpenFile::clone() const
{
  return CkppCommandOpenFile::createCopy(attWeakPtr.lock());
}
bool CkppCommandOpenFile::isUndoable() const
{
  return true;
}
unsigned int CkppCommandOpenFile::countParameters() const
{
  return PARAMETER_COUNT;
}
CkppParameterConstShPtr CkppCommandOpenFile::parameter(unsigned int inRank) const
{
  CkppParameterShPtr result;
  CkppComponentShPtr nullComponent;
  switch(inRank)
    {
     case FILE_PATH: 
       result = CkppFileParameter::create("file","Choose a file to load", "*.kxml","/home/dflavign/GEPETTO/KineoWorks/","/home/dflavign/GEPETTO/KineoWorks/BugTrap_2D_Problem1.kxml", CkppFileParameter::OPEN );
       break;
    default:
      KIT_ASSERT( false );
    }
  return result;
}
ktStatus CkppCommandOpenFile::doExecute()
{
  std::string filename(paramValue(parameter(FILE_PATH)).stringValue());
  return attInterface->hppPlanner()->parseFile(filename);
  
}

