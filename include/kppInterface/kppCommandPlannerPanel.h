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
   \addtogroup Panel
   @{
   \section Command_Panel Launch Planner Configuration Panel
   Command allowing users to launch the configuration panel.
   @}
*/
class CkppCommandPlannerPanel : public CkppPlanPathCommand
{

public:

  enum EParameters
    {
      PARAMETER_COUNT
    };

  /**
     \brief Create Method
     \param kpp The kppInterface that uses the command
     \param inCommandFactory The command factory that will be used to build the UICommand (Menu item)
     \return A Shared Pointer to the newly created command
   */
  static CkppCommandPlannerPanelShPtr   create(CkppInterface *kpp, const CkppMainWindowUICommandFactoryConstShPtr& inCommandFactory);

  /**
     \brief Copy creator. Uses a previously defined PlannerPanel Command to create a new one
     \param inCommand The previous command.
     \return  A Shared Pointer to the newly created command
     
   */
  static CkppCommandPlannerPanelShPtr   createCopy(const CkppCommandPlannerPanelConstShPtr& inCommand);

  /**
     \brief Destructor
   */
  virtual ~CkppCommandPlannerPanel();

  /**
     \brief : What function does.
  */
  virtual ktStatus doExecute();

  /**
     \brief Says if the command can be undone.
     \return true if it can be  undone, false in other cases.
   */
  virtual bool	isUndoable() const;

  virtual CkppCommandShPtr clone() const;

  virtual unsigned int	countParameters() const;

  /**
     \brief Retrieves a parameter defined in EParameters (enum in the header file) from the model tree.
     \param inRank Rank in the enum structure of the parameter to retrieve.
   */
  virtual CkppParameterConstShPtr	parameter(unsigned int inRank) const;

protected:

  /**
     \brief Constructor
   */
  CkppCommandPlannerPanel(CkppInterface *kpp,const CkppMainWindowUICommandFactoryConstShPtr& inCommandFactory);

  /**
     \brief Copy Constructor
   */
  CkppCommandPlannerPanel(const CkppCommandPlannerPanel& inCommand);

  /**
     \brief Initialisation method.
   */
  ktStatus init(const CkppCommandPlannerPanelWkPtr& inWeakPtr);

private:
  
  CkppCommandPlannerPanelWkPtr attWeakPtr;

  CkppUICommandShPtr commandPlannerPanel;

  CkppMainWindowUICommandFactoryConstShPtr attCommandFactory;

  /// pointer to the interface
  CkppInterface *attKpp;

};

#endif
