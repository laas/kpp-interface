/*
 *  Copyright
 */

#include "kppInterface/kppCommandOpenFile.h" 

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

