// COPYRIGHT 


#ifndef KPP_COMMAND_PLANNER_PANEL_H
#define KPP_COMMAND_PLANNER_PANEL_H

/*****************************************
 INCLUDES
*******************************************/

#include <iostream>

#include "kppInterface/kppInterface.h"
#include "kppInterface/kppPlannerPanelController.h"
#include "kppInterface/kppPlannerPanel.h"
#include "kppInterface/kwsGraphicRoadmap.h"

#include "KineoGUI/kppMainWindowUICommandFactory.h"
#include "KineoModel/kppDefaultCreateFactory.h"

#include "KineoController/kppPlanPathCommand.h"

KIT_PREDEF_CLASS( CkppCommandPlannerPanel );

class CkppInterface;

/*****************************************
 CLASS
*******************************************/
/**
   \brief Command allowing users to launch the configuration panel. The planner must have initialized first with default values
*/
class CkppCommandPlannerPanel : public CkppPlanPathCommand
{

public:

  enum EParameters
    {
     /* MODELTREE = 0,
      PATH,
      DEVICE,*/
      PARAMETER_COUNT
    };

  static CkppCommandPlannerPanelShPtr   create(CkppInterface *kpp, const CkppMainWindowUICommandFactoryConstShPtr& i_commandFactory);

  static CkppCommandPlannerPanelShPtr   createCopy(const CkppCommandPlannerPanelConstShPtr& i_command);

  virtual ~CkppCommandPlannerPanel();

  /**
     \brief : What function does.
  */
  virtual ktStatus doExecute();

  virtual bool	isUndoable() const;

  virtual CkppCommandShPtr clone() const;

  virtual unsigned int	countParameters() const;

  virtual CkppParameterConstShPtr	parameter(unsigned int i_rank) const;

protected:

  CkppCommandPlannerPanel(CkppInterface *kpp,const CkppMainWindowUICommandFactoryConstShPtr& i_commandFactory_arg);

  CkppCommandPlannerPanel(const CkppCommandPlannerPanel& i_command);

  ktStatus init(const CkppCommandPlannerPanelWkPtr& i_weakPtr);

private:
  
  CkppCommandPlannerPanelWkPtr m_weakPtr;

  CkppUICommandShPtr commandPlannerPanel;

  CkppMainWindowUICommandFactoryConstShPtr i_commandFactory;

  /// pointer to the interface
  CkppInterface *attKpp;

};

#endif
