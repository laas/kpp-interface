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

  ktStatus success = KD_OK;
  CkppComponentShPtr modelTreeComponent;
  CkprParserManagerShPtr parser = CkprParserManager::defaultManager();

   std::string filename(paramValue(parameter(FILE_PATH)).stringValue());
  
//  std::string filename("/home/dflavign/GEPETTO/KineoWorks/BugTrap_2D_Problem1.kxml");
//  std::string filename("/home/dflavign/GEPETTO/KineoWorks/da2.kxml");
//  std::string filename("/home/sdalibar/test_kineo/da.kxml");

  if(filename.length() == 0){ cout<<"no filename"<<endl; return KD_ERROR;}
  
  if(KD_ERROR == parser->loadComponentFromFile(filename, modelTreeComponent)){ cout<<"unable to load file "<<filename<<endl; return KD_ERROR; }
  
  CkppModelTreeShPtr modelTree = KIT_DYNAMIC_PTR_CAST(CkppModelTree,modelTreeComponent);
  if(!modelTree){ cout<<"no modelTree"<<endl; return KD_ERROR;}

  if(!modelTree->deviceNode()) cout<<"No devices"<<endl;
  else{ 
    for(unsigned int i = 0; i< modelTree->deviceNode()->countChildComponents(); i++){
      
      CkppDeviceComponentShPtr deviceComponent = KIT_DYNAMIC_PTR_CAST(CkppDeviceComponent,modelTree->deviceNode()->childComponent(i));
      
      if(deviceComponent){
 	CkitNotificationShPtr notification = CkitNotification::createWithPtr<ChppPlanner>(ChppPlanner::ID_HPP_ADD_ROBOT, attInterface->hppPlanner());
 	notification->shPtrValue<CkppDeviceComponent>(ChppPlanner::ROBOT_KEY, deviceComponent);
 	CkitNotificator::defaultNotificator()->notify(notification);
      }
      
    }
  }


  if(!modelTree->geometryNode()) cout<<"No geometries"<<endl;
  else{cout<<"geometries"<<endl;
    for(unsigned int i = 0; i< modelTree->geometryNode()->countChildComponents(); i++){
      
      cout<<i<<" "<<endl;
      
      CkcdObjectShPtr kcdObject = KIT_DYNAMIC_PTR_CAST(CkcdObject, modelTree->geometryNode()->childComponent(i));
      if(kcdPolyhedron) attInterface->hppPlanner()->addObstacle(kcdObject);
      else cout<<"Cannot cast component to kcdPolyhedron"<<endl;
    }
  }
  
  if(!modelTree->pathNode()) cout<<"No paths"<<endl;
  else{
    
    for(unsigned int i = 0; i< modelTree->pathNode()->countChildComponents(); i++){
      CkppPathComponentShPtr pathComponent = KIT_DYNAMIC_PTR_CAST(CkppPathComponent,modelTree->pathNode()->childComponent(i));

      CkitNotificationShPtr notification = CkitNotification::create ( ChppProblem::ID_HPP_ADD_PATH);
      notification->shPtrValue<CkwsPath> (ChppProblem::PATH_KEY, pathComponent->kwsPath() );

//      searching the corresponding device component

      notification->shPtrValue<CkppDeviceComponent> ( ChppProblem::DEVICE_KEY, pathComponent->deviceComponent() );
      notification->unsignedIntValue ( ChppProblem::PATH_ID_KEY, i );
      CkitNotificator::defaultNotificator()->notify ( notification );

    }

  }

  return KD_OK ;
}

