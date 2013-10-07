//
// Copyright (c) 2005, 2006, 2007, 2008, 2009, 2010, 2011, 2012, 2013 CNRS
// Authors: David Flavigne
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

/*****************************************
 INCLUDES
*******************************************/

#include "KineoController/kppSetPropertyCommand.h"
#include "KineoModel/kppComponentParameter.h"
#include "KineoModel/kppValue.h"
#include "KineoModel/kppComponentClassFilter.h"
#include "KineoModel/kppComponent.h"
#include "KineoModel/kppSolidComponent.h"
#include "KineoGUI/kppMainWindowController.h"
#include "KineoController/kppUICommand.h"

#include "hpp/core/planner.hh"

#include "kpp/interface/command-planner-panel.hh"
#include "kpp/interface/planner-panel.hh"
#include "kpp/interface/planner-panel-controller.hh"

/*****************************************
 DEFINES
*******************************************/

HPP_KIT_PREDEF_CLASS( CkppPlannerPanelController );

/*****************************************
 METHODS
*******************************************/



// ==========================================================================

CkppCommandPlannerPanel::CkppCommandPlannerPanel(CkppInterface *kpp,const CkppMainWindowUICommandFactoryConstShPtr& inCommandFactory)
{
  attKpp = kpp;
  attCommandFactory = inCommandFactory;
}


// ==========================================================================

CkppCommandPlannerPanel::CkppCommandPlannerPanel(const CkppCommandPlannerPanel& inCommand) :
  CkppPlanPathCommand(inCommand)
{
  attKpp = inCommand.attKpp;
  commandPlannerPanel = inCommand.commandPlannerPanel;
  attCommandFactory = inCommand.attCommandFactory;
}


// ==========================================================================

CkppCommandPlannerPanel::~CkppCommandPlannerPanel()
{
  attKpp = NULL;
}


// ==========================================================================

CkppCommandPlannerPanelShPtr CkppCommandPlannerPanel::create(CkppInterface *kpp,const CkppMainWindowUICommandFactoryConstShPtr& inCommandFactory)
{
  CkppCommandPlannerPanel*  ptr = new CkppCommandPlannerPanel(kpp,inCommandFactory);
  CkppCommandPlannerPanelShPtr shPtr(ptr);
  
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }

  return shPtr;
}

// ==========================================================================

CkppCommandPlannerPanelShPtr CkppCommandPlannerPanel::createCopy(const CkppCommandPlannerPanelConstShPtr& inCommand)
{
  CkppCommandPlannerPanel*  ptr = new CkppCommandPlannerPanel(*inCommand);
  CkppCommandPlannerPanelShPtr shPtr(ptr);
  
  if(KD_OK != ptr->init(shPtr))
    {
      shPtr.reset();
    }
  
  return shPtr;
}


// ==========================================================================

ktStatus CkppCommandPlannerPanel::init(const CkppCommandPlannerPanelWkPtr& inWeakPtr)
{
  ktStatus success = CkppPlanPathCommand::init(inWeakPtr);
  
  if(KD_OK == success)
	{
	  attWeakPtr = inWeakPtr;
	}

  return success;
}


// ==========================================================================


CkppCommandShPtr CkppCommandPlannerPanel::clone() const
{
  return CkppCommandPlannerPanel::createCopy(attWeakPtr.lock());
}


// ==========================================================================

bool CkppCommandPlannerPanel::isUndoable() const
{
  return true;
}


// ==========================================================================

unsigned int CkppCommandPlannerPanel::countParameters() const
{
  return PARAMETER_COUNT;
}


// ==========================================================================

CkppParameterConstShPtr CkppCommandPlannerPanel::parameter(unsigned int inRank) const
{
 
  CkppParameterShPtr result;
  CkppComponentShPtr nullComponent;

  switch(inRank)
    {  
/*    case MODELTREE : 
      result = CkppComponentParameter::create( "Model Tree", CkppComponentClassFilter< CkppModelTree >());
      break;
    case PATH : 
      result = CkppComponentParameter::create( "Path", CkppComponentClassFilter< CkppPathComponent >());
      break;
    case DEVICE : 
      result = CkppComponentParameter::create( "Device", CkppComponentClassFilter< CkppDeviceComponent >());
      break; */
    default:
      KIT_ASSERT( false );
    }
  
  return result;
}


// ==========================================================================

ktStatus CkppCommandPlannerPanel::doExecute()
{


//----------- Displaying Panel -----------------------------------------------------------  
  commandPlannerPanel = attCommandFactory->makeShowPanelUICommand(
							     CkppDefaultCreateFactory< CkppWindowControllerShPtr, CkppPlannerPanelController >::create(),
							     "Planner Configuration", //leave empty and the panel window title will be used (optional)
							     "Displays a configuration panel for planners", //help string (optional)
							     "" //use CkppModuleInterface::pathToResourceFile() if called from an Add-On (needed for toolbar commands only)
							     );
  
  if(commandPlannerPanel->execute() == KD_ERROR) cout<<"Error Executing Command !"<<endl; 
  //to do this, we must be sure that the current panel controller is a PlannerPanelController : currently guaranteed by the "execute" command on the previous line
  CkppPlannerPanelControllerShPtr panelController = KIT_DYNAMIC_PTR_CAST(CkppPlannerPanelController,attCommandFactory->mainWindowController()->panelController());
  CkwsDiffusingRdmBuilderShPtr RRT = KIT_DYNAMIC_PTR_CAST(CkwsDiffusingRdmBuilder,attKpp->hppPlanner()->roadmapBuilderIthProblem(0));
  panelController->getPanel()->setRdmBuilder(RRT); 
  panelController->getPanel()->setInterface(attKpp);

//------------------------------------------------------------------------------------------

  return KD_OK ;
 
}


// ==========================================================================
