//COPYRIGHT

#ifndef KPP_PLANNER_PANEL_CONTROLLER_H
#define KPP_PLANNER_PANEL_CONTROLLER_H


/**************************INCLUDES*************************/

#include "KineoGUI/kppPanel.h"
#include "KineoGUI/kppWindowController.h"
#include "KineoWorks2/kwsShooterConfigList.h"
#include "KineoWorks2/kwsDevice.h"
#include "KineoWorks2/kwsPickerBasic.h"
#include "KineoWorks2/kwsPickerSmallestTree.h"
#include "KineoWorks2/kwsShooterConfigSpace.h"
#include "KineoWorks2/kwsShooterPath.h"
#include "KineoWorks2/kwsShooterRoadmapNodes.h"
#include "KineoWorks2/kwsShooterRoadmapBox.h"
#include "KineoWorks2/kwsShooterMulti.h"
#include "KineoWorks2/kwsShooterAdaptiveMulti.h"
#include "KineoWorks2/kwsSMLinear.h"

#include "wx/sizer.h"
#include "wx/bmpcbox.h"
#include "wx/checkbox.h"
#include "wx/button.h"
#include "wx/arrstr.h"
#include "wx/window.h"
#include "wx/event.h"
#include "wx/spinctrl.h"


KIT_PREDEF_CLASS( CkppPlannerPanelController );
KIT_PREDEF_CLASS( CkppPlannerPanel );

/***********************************************************/

/*****************************CLASS*************************/

/**
   This class is where the behaviour of a PlannerPanel is implemented. It implements particularly the
   behaviour of the 2 buttons (OK & CANCEL/RESET), in the functions okClickHappens & cancelClickHappens.
   Derived from /see CkppWindowController.
   See also /see CkppPlannerPanel.
*/

class CkppPlannerPanelController : public CkppWindowController
{

 private:

  CkppPlannerPanelControllerWkPtr m_weakPtr;
  CkppPlannerPanelShPtr panel;
  int problemId;

 public:
  /** \brief Destructor 
   */
  virtual ~CkppPlannerPanelController();
  
  /**
     \name create methods
     @{
  */
  /**
     \brief Create an instance of KppInterfaceRRTShPtr
  */     
  static CkppPlannerPanelControllerShPtr create();
  /**
     @}
  */
  
  /**
     \brief returns the associated panel
   */
  CkppPlannerPanelShPtr getPanel(){return panel;}
  
  /**
     \brief sets the associated panel
   */
  void setPanel(const CkppPlannerPanelShPtr& i_panel){panel = i_panel;}

  /**
     \name inherited methods
     @{
  */
  /**
     \brief inherited from CkppWindowController.
  */
  void loadWindow();

  /**
     \brief inherited from CkppWindowController. Initialization method
  */
  ktStatus init (const CkppPlannerPanelControllerWkPtr &i_weakPtr);
  /**
     @}
  */
  
  /**
     \name event handlers associated to each control
     @{
  */
  void builderComboBoxEventHandler(wxCommandEvent& cancel);
  void shooterComboBoxEventHandler(wxCommandEvent& cancel);
  void pickerComboBoxEventHandler(wxCommandEvent& cancel);
  void steeringComboBoxEventHandler(wxCommandEvent& cancel);
  void delegateComboBoxEventHandler(wxCommandEvent& cancel);
  void optimizerComboBoxEventHandler(wxCommandEvent& cancel);
  void RoadmapCheckBoxEventHandler(wxCommandEvent& cancel);
  void SolveAllCheckBoxEventHandler(wxCommandEvent& cancel);
  void BiDiffuseCheckBoxEventHandler(wxCommandEvent& cancel);
  void MagnetCheckBoxEventHandler(wxCommandEvent& cancel);
  void CsShooterCheckBoxEventHandler(wxCommandEvent& cancel);
  void ClShooterCheckBoxEventHandler(wxCommandEvent& cancel);
  void PShooterCheckBoxEventHandler(wxCommandEvent& cancel);
  void RbShooterCheckBoxEventHandler(wxCommandEvent& cancel);
  void RnShooterCheckBoxEventHandler(wxCommandEvent& cancel);
  void ShooterCheckBoxEventHandler(wxCommandEvent& cancel);
  void StartButtonEventHandler(wxCommandEvent& cancel);
  void SaveButtonEventHandler(wxCommandEvent& cancel);
  void OpenButtonEventHandler(wxCommandEvent& cancel);
  void AddPbButtonEventHandler(wxCommandEvent& cancel);
  void ProblemSpinCtrlEventHandler(wxSpinEvent& cancel);
  /**
     @}
   */
 
  /** \brief Add existing magnet configurations in a new tab
   */
  void addConfigurations();

  /** \brief Updates the config list for a config list shooter
   */
  void updateConfigList(CkwsShooterConfigListShPtr configList);

  /** \brief Creates a list of weighted shooters for a multi shooter.
      \returns a list of weighted shooters
   */
  CkwsShooterMulti::TWeightedShooterList updateShooterList();


  
  
 protected:
  /** \brief Constructor 
   */
  CkppPlannerPanelController();

  //Declaration of a table defined in wx library to handle GUI events such as mouse events, etc...
  //Look at .cpp to see the table definition
  DECLARE_EVENT_TABLE()

};

/***********************************************************/

#endif
