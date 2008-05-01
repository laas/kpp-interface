/*
  Copyright CNRS-LAAS

  Authors: David Flavigne and Florent Lamiraux
*/

/**************************INCLUDES*************************/

#include "kppInterface/kppPlannerPanelController.h"
#include "kppInterface/kppPlannerPanel.h"

#include "KineoGUI/kppMainWindowController.h"
#include "KineoModel/kppConfigComponent.h"
#include "KineoController/kppDocument.h"
#include "KineoModel/kppModelTree.h"
#include "KineoWorks2/kwsSMSlerp.h"

#include "wx/sizer.h"
#include "wx/bmpcbox.h"
#include "wx/checkbox.h"
#include "wx/button.h"
#include "wx/arrstr.h"
#include "wx/window.h"
#include "wx/event.h"
#include "wx/spinctrl.h"
#include "wx/notebook.h"

#include <iostream>

//Definition of the event table declared in the header file
BEGIN_EVENT_TABLE(CkppPlannerPanelController,CkppWindowController)
  EVT_COMBOBOX(RDM_BUILDER_COMBO_BOX,CkppPlannerPanelController::builderComboBoxEventHandler)
  EVT_COMBOBOX(SHOOTER_COMBO_BOX,CkppPlannerPanelController::shooterComboBoxEventHandler)
  EVT_COMBOBOX(PICKER_COMBO_BOX,CkppPlannerPanelController::pickerComboBoxEventHandler)
  EVT_COMBOBOX(STEERING_COMBO_BOX,CkppPlannerPanelController::steeringComboBoxEventHandler)
  EVT_COMBOBOX(DELEGATE_COMBO_BOX,CkppPlannerPanelController::delegateComboBoxEventHandler)
  EVT_COMBOBOX(OPTIMIZER_COMBO_BOX,CkppPlannerPanelController::optimizerComboBoxEventHandler)
  EVT_CHECKBOX(RDM_CHECK_BOX,CkppPlannerPanelController::RoadmapCheckBoxEventHandler)
  EVT_CHECKBOX(SOLVE_ALL_CHECK_BOX,CkppPlannerPanelController::SolveAllCheckBoxEventHandler)
  EVT_CHECKBOX(BIDIFF_CHECK_BOX,CkppPlannerPanelController::BiDiffuseCheckBoxEventHandler)
  EVT_CHECKBOX(MAGNET_CHECK_BOX,CkppPlannerPanelController::MagnetCheckBoxEventHandler)
  EVT_CHECKBOX(CS_SHOOTER_CHECK_BOX,CkppPlannerPanelController::CsShooterCheckBoxEventHandler)
  EVT_CHECKBOX(CL_SHOOTER_CHECK_BOX,CkppPlannerPanelController::ClShooterCheckBoxEventHandler)
  EVT_CHECKBOX(P_SHOOTER_CHECK_BOX,CkppPlannerPanelController::PShooterCheckBoxEventHandler)
  EVT_CHECKBOX(RB_SHOOTER_CHECK_BOX,CkppPlannerPanelController::RbShooterCheckBoxEventHandler)
  EVT_CHECKBOX(RN_SHOOTER_CHECK_BOX,CkppPlannerPanelController::RnShooterCheckBoxEventHandler)
  EVT_BUTTON(START_BUTTON,CkppPlannerPanelController::StartButtonEventHandler)
  EVT_BUTTON(SAVE_BUTTON,CkppPlannerPanelController::SaveButtonEventHandler)
  EVT_BUTTON(OPEN_BUTTON,CkppPlannerPanelController::OpenButtonEventHandler)
  EVT_BUTTON(ADD_PB_BUTTON,CkppPlannerPanelController::AddPbButtonEventHandler)
  EVT_SPINCTRL(PROBLEM_SPIN_CTRL,CkppPlannerPanelController::ProblemSpinCtrlEventHandler)
END_EVENT_TABLE()

using namespace std;

// Select verbosity at configuration by setting CXXFLAGS="... -DDEBUG=[1 or 2]"
#if DEBUG==2
#define ODEBUG2(x) std::cout << "kppPlannerPanel:" << x << std::endl
#define ODEBUG1(x) std::cerr << "kppPlannerPanel:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "kppPlannerPanel:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

#ifndef HPP_RDMBUILDER_DEFPENETRATION
#define HPP_RDMBUILDER_DEFPENETRATION       0.02
#endif 

/***********************************************************/

/*************************METHODS***************************/
/*_________________________________________________________*/
CkppPlannerPanelController::~CkppPlannerPanelController(){

  panel.reset();
  attWeakPtr.reset();
  problemId = 0;

}

/*_________________________________________________________*/

CkppPlannerPanelController::CkppPlannerPanelController(){

  panel.reset();
  attWeakPtr.reset();
  problemId = 0;

}
/*_________________________________________________________*/

CkppPlannerPanelControllerShPtr CkppPlannerPanelController::create(){

  CkppPlannerPanelController* ptr = new CkppPlannerPanelController();
  CkppPlannerPanelControllerShPtr shPtr(ptr);

  if(KD_ERROR == ptr->init(shPtr))
    {
      shPtr.reset();
    }
  
  return shPtr;
}
/*_________________________________________________________*/

ktStatus CkppPlannerPanelController::init (const CkppPlannerPanelControllerWkPtr &inWeakPtr){

  ktStatus success = KD_ERROR;
  success = CkppWindowController::init( inWeakPtr );

  if( success == KD_OK )
    {
      attWeakPtr = inWeakPtr;
      problemId = 0;

      doesSelfDeleteOnClose(true);
      doesReleaseWindowOnClose(false);

      windowTitle("Planner Configuration Panel");

    }

  return success;
}

/*_________________________________________________________*/
void CkppPlannerPanelController::loadWindow(){

  ODEBUG2(" Creating panel ...");
  CkppPlannerPanel* myPanel;
  
  myPanel = new CkppPlannerPanel(parentWindow(), attWeakPtr.lock());
  myPanel->finalize();

  CkppPlannerPanelShPtr ptr(myPanel);

  setPanel(ptr);

  window(myPanel);
  ODEBUG2(" Done");
  
}

/*_________________________________________________________*/

void CkppPlannerPanelController::builderComboBoxEventHandler(wxCommandEvent& cancel){
//TODO : create an instance of the chosen builder
//  Add this builder to the selected problem

  ODEBUG2("builder box event triggered");
  wxComboBox* builderCombo = dynamic_cast<wxComboBox*>(cancel.GetEventObject());
  
  switch(builderCombo->GetSelection()){
  case CkppPlannerPanel::PCA : 
    ODEBUG1(" PCA builder selected: choose one of the builders in the bottom tab");
    
    if(!panel->FindWindowByName("DiffusingBuilder",panel.get())) {
      panel->addControl(panel->getNotebook()->GetPage(0),
			new wxCheckBox(panel->getNotebook()->GetPage(0), 
				       wxID_ANY,"Dif. Builder",wxDefaultPosition,wxDefaultSize,0,
				       wxDefaultValidator,"DiffusingBuilder"));
    }
    if(!panel->FindWindowByName("IPPBuilder",panel.get())) {
      panel->addControl(panel->getNotebook()->GetPage(0),
			new wxCheckBox(panel->getNotebook()->GetPage(0), 
				       wxID_ANY,"IPP Builder",wxDefaultPosition,wxDefaultSize,0,
				       wxDefaultValidator,"IPPBuilder"));
    }
    if(!panel->FindWindowByName("LTBuilder",panel.get())) {
      panel->addControl(panel->getNotebook()->GetPage(0),
			new wxCheckBox(panel->getNotebook()->GetPage(0), 
				       wxID_ANY,"LT Builder",wxDefaultPosition,wxDefaultSize,0,
				       wxDefaultValidator,"LTBuilder"));
    }
    
    break;

  case CkppPlannerPanel::LOCAL_TREES :
    ODEBUG1(" Local Trees builder selected: choose one of the builders in the bottom tab");
    
    if(!panel->FindWindowByName("DiffusingBuilder",panel.get())) {
      panel->addControl(panel->getNotebook()->GetPage(0),
			new wxCheckBox(panel->getNotebook()->GetPage(0), 
				       wxID_ANY,"Dif. Builder",wxDefaultPosition,wxDefaultSize,0,
				       wxDefaultValidator,"DiffusingBuilder"));
    }
    if(!panel->FindWindowByName("IPPBuilder",panel.get())) {
      panel->addControl(panel->getNotebook()->GetPage(0),
			new wxCheckBox(panel->getNotebook()->GetPage(0), 
				       wxID_ANY,"IPP Builder",wxDefaultPosition,wxDefaultSize,0,
				       wxDefaultValidator,"IPPBuilder"));
    }
    
    break;
    
  default:
    break;
  }

}

/*_________________________________________________________*/
void CkppPlannerPanelController::shooterComboBoxEventHandler(wxCommandEvent& cancel){
//TODO : create an instance of the chosen shooter and add it to the problem
// Verify that the builder is a diffusion one. in case of a config list shooter
// look for saved configurations in the modeltree and create checkbox for each one
// in the "magnets tab"

  ODEBUG2("shooter box event triggered");

  wxPanel* mainPage = panel->getMainPage();
  wxComboBox* shooterCombo = dynamic_cast<wxComboBox*>(cancel.GetEventObject());

  if(!mainPage){
    if(!(panel->GetSizer()->GetChildren())[0]) {
      ODEBUG1(" ERROR : The panel has no children - ");
    } 
    else {
      ODEBUG1(" ERROR : ");
    }
    ODEBUG1(" Unable to retrieve the main page of the panel");
  }
  else{
    
    switch(shooterCombo->GetSelection()){
      
    case CkppPlannerPanel::CONFIG_LIST : //selecting magnet shooter
      ODEBUG1(" Magnet Shooter has been selected : choose magnet configurations in the new tab");
      addConfigurations();

      panel->getNotebook()->GetPage(1)->Show(false);
      panel->getNotebook()->GetPage(2)->Show(true);
      break;
      
    case CkppPlannerPanel::MULTI_SHHOTER :  //selecting multi shooter
      ODEBUG1(" Multi Shooter has been selected : choose shooters in the new tab");
      panel->getNotebook()->GetPage(2)->Show(false);
      panel->getNotebook()->GetPage(1)->Show(true);
      break;
      
    case CkppPlannerPanel::ADAPTIVE_MULTI : //selecting adaptive multi shooter
      ODEBUG1(" Adaptive Multi Shooter has been selected : choose shooters in the new tab");
      panel->getNotebook()->GetPage(2)->Show(false);
      panel->getNotebook()->GetPage(1)->Show(true);
      break;

    default:
      panel->getNotebook()->GetPage(1)->Show(false);
      panel->getNotebook()->GetPage(2)->Show(false);
      break;
    }
  }

}

/*_________________________________________________________*/
void CkppPlannerPanelController::pickerComboBoxEventHandler(wxCommandEvent& cancel){
//TODO : create an instance of the chosen picker and add it to the problem
// Verify that the builder is a diffusion one
  ODEBUG2("picker box event triggered");
}

/*_________________________________________________________*/
void CkppPlannerPanelController::steeringComboBoxEventHandler(wxCommandEvent& cancel){
//TODO : create an instance of the chosen steering method and add it to the problem
// specific behaviour for reed & sheep and flic.

  ODEBUG2("steering method box event triggered");

  wxComboBox * steeringCombo = dynamic_cast<wxComboBox*>(cancel.GetEventObject());
  switch(steeringCombo->GetSelection()){
  case CkppPlannerPanel::FLIC : 
    ODEBUG1(" Flic steering method selected");
    if(!panel->FindWindowByName("IsOriented",panel.get())) {
      panel->addControl(panel->getNotebook()->GetPage(0),
			new wxCheckBox(panel->getNotebook()->GetPage(0), 
				       wxID_ANY,"Is Oriented",wxDefaultPosition,wxDefaultSize,0,
				       wxDefaultValidator,"IsOriented" ));
    }
    break;

  case CkppPlannerPanel::RS : 
    ODEBUG1(" choose Reeds & Shepp steering method");
    if(!panel->FindWindowByName("IsOriented",panel.get())) {
      panel->addControl(panel->getNotebook()->GetPage(0),
			new wxCheckBox(panel->getNotebook()->GetPage(0), 
				       wxID_ANY, "Is Oriented",wxDefaultPosition,wxDefaultSize,0,
				       wxDefaultValidator,"IsOriented" ));
    }
    if(!panel->FindWindowByName("RSRadius",panel.get())) {
      panel->addControl(panel->getNotebook()->GetPage(0),
			new wxSpinCtrl(panel->getNotebook()->GetPage(0),
				       wxID_ANY,"R & S Radius",wxDefaultPosition,wxDefaultSize,wxSP_ARROW_KEYS,
				       1,200,0,"RSRadius"));
    }
    break;
  default:
    break;
  }
  
}

/*_________________________________________________________*/
void CkppPlannerPanelController::delegateComboBoxEventHandler(wxCommandEvent& cancel){
  ODEBUG2("delegate box event triggered");

}

/*_________________________________________________________*/
void CkppPlannerPanelController::optimizerComboBoxEventHandler(wxCommandEvent& cancel){
//TODO : create an instance of the chosen path optimizer and add it to the problem
    ODEBUG2("Optimizer box event triggered");
}

/*_________________________________________________________*/
void CkppPlannerPanelController::RoadmapCheckBoxEventHandler(wxCommandEvent& cancel){
//TODO : enable roadmap display.
  ODEBUG2(" RoadmapCheckBoxEventHandler");

}

/*_________________________________________________________*/
void CkppPlannerPanelController::SolveAllCheckBoxEventHandler(wxCommandEvent& cancel){
//TODO : if checked, allow users to solve all problems contained in the vector of problems
// otherwise, only the problem with the selected number will be solved
}

/*_________________________________________________________*/
void CkppPlannerPanelController::BiDiffuseCheckBoxEventHandler(wxCommandEvent& cancel){
//TODO : only for diffusion builders. allow users to diffuse from both start & goal nodes
//  wxCheckBox * BiDiffuseCheckBox = dynamic_cast<wxCheckBox*>(cancel.GetEventObject());
//  KIT_DYNAMIC_PTR_CAST(CkwsDiffusingRdmBuilder,panel->getRdmBuilder())->diffuseFromProblemGoal(BiDiffuseCheckBox->IsChecked());
}

/*_________________________________________________________*/
void CkppPlannerPanelController::MagnetCheckBoxEventHandler(wxCommandEvent& cancel){
//TODO : fill the config list for each checked box

  

}

/*_________________________________________________________*/
void CkppPlannerPanelController::CsShooterCheckBoxEventHandler(wxCommandEvent& cancel){

  ShooterCheckBoxEventHandler(cancel);

}

/*_________________________________________________________*/
void CkppPlannerPanelController::ClShooterCheckBoxEventHandler(wxCommandEvent& cancel){

  wxCheckBox* ClShooterCheckBox = dynamic_cast<wxCheckBox*>(cancel.GetEventObject());

  if(ClShooterCheckBox->IsChecked()){
    
    ODEBUG1(" Magnet Shooter selected : choose magnet configurations in the new tab");
    addConfigurations();
    panel->getNotebook()->GetPage(2)->Show(true);

  }else{

    ODEBUG1(" Magnet Shooter has been deselected");
    panel->getNotebook()->GetPage(2)->Show(false);

  }

  ShooterCheckBoxEventHandler(cancel);
}

/*_________________________________________________________*/
void CkppPlannerPanelController::PShooterCheckBoxEventHandler(wxCommandEvent& cancel){

  ShooterCheckBoxEventHandler(cancel);

}

/*_________________________________________________________*/
void CkppPlannerPanelController::RbShooterCheckBoxEventHandler(wxCommandEvent& cancel){

  ShooterCheckBoxEventHandler(cancel);

}

/*_________________________________________________________*/
void CkppPlannerPanelController::RnShooterCheckBoxEventHandler(wxCommandEvent& cancel){

  ShooterCheckBoxEventHandler(cancel);

}
/*_________________________________________________________*/
void CkppPlannerPanelController::ShooterCheckBoxEventHandler(wxCommandEvent& cancel){
//TODO : create an instance of the chosen shooter and add it in the multishooter's vector
}

/*_________________________________________________________*/
void CkppPlannerPanelController::StartButtonEventHandler(wxCommandEvent& cancel){
  //TODO : Apply configuration to the planner. Launch algorithm. take into account the solveall checkbox ?

  double penetration = HPP_RDMBUILDER_DEFPENETRATION;
  wxCheckBox * SolveAllCheckBox = dynamic_cast<wxCheckBox*>(panel->FindWindowByName("SolveAll Chk",panel.get()));
  int FirstProblem, LastProblem;
  bool isDiffusing = false;
  CkwsRoadmapBuilderShPtr rdmBuilder;
  CkwsDiffusingRdmBuilderShPtr DiffusingRdmBuilder;
  if(SolveAllCheckBox->IsChecked()) {
    ODEBUG1(" Solve All");
    FirstProblem = 0;
    LastProblem = panel->getInterface()->hppPlanner()->getNbHppProblems();
  }
  else {
    problemId = 0;
    FirstProblem = problemId;
    LastProblem = problemId+1;
    ODEBUG1(" Solve Only Problem " << problemId);
  }

  for(int i=FirstProblem; i<LastProblem; i++){

    ODEBUG1(" Configuring Problem " << i);
    CkwsDeviceShPtr Device = panel->getInterface()->hppPlanner()->robotIthProblem(i);
    
    wxComboBox* RoadmapBuilderComboBox = dynamic_cast<wxComboBox*>(panel->FindWindowByName("Roadmap Builder",panel.get()));
    wxComboBox* ShooterComboBox  = dynamic_cast<wxComboBox*>(panel->FindWindowByName("Diffusion Shooter",panel.get()));
    wxComboBox* PickerComboBox = dynamic_cast<wxComboBox*>(panel->FindWindowByName("Diffusion Picker",panel.get()));
    wxComboBox* SteeringMethodComboBox = dynamic_cast<wxComboBox*>(panel->FindWindowByName("Steering Method",panel.get()));
#if 0
    wxComboBox* DelegateComboBox = dynamic_cast<wxComboBox*>(panel->FindWindowByName("Delegates",panel.get()));
#endif
    wxComboBox* OptimizerComboBox = dynamic_cast<wxComboBox*>(panel->FindWindowByName("Path Optimizer",panel.get()));
    wxCheckBox* biDiffuseCheckBox =  dynamic_cast<wxCheckBox*>(panel->FindWindowByName("BiDiff Chk",panel.get()));
    wxCheckBox* ShowRdmCheckBox =  dynamic_cast<wxCheckBox*>(panel->FindWindowByName("H/S Rdm Chk",panel.get()));
    wxCheckBox* IsOrientedCheckBox =  dynamic_cast<wxCheckBox*>(panel->FindWindowByName("IsOriented",panel.get()));
    wxCheckBox* DiffusingBuilderCheckBox =  dynamic_cast<wxCheckBox*>(panel->FindWindowByName("DiffusingBuilder",panel.get()));
    wxCheckBox* LTBuilderCheckBox =  dynamic_cast<wxCheckBox*>(panel->FindWindowByName("LTBuilder",panel.get()));
    wxSpinCtrl* RSRadiusSpinCtrl = dynamic_cast<wxSpinCtrl*>(panel->FindWindowByName("RSRadius",panel.get()));
    
    switch(RoadmapBuilderComboBox->GetCurrentSelection()){//to set the chosen roadmapBuilder
    case CkppPlannerPanel::BASIC_RDMBUILDER : 
      ODEBUG2(" Basic roadmap builder selected.");
      rdmBuilder = CkwsBasicRdmBuilder::create(CkwsRoadmap::create(Device), penetration);
      isDiffusing = false;
      break;

    case CkppPlannerPanel::DIFFUSING : 
      ODEBUG2(" Diffusing roadmap builder selected.");
      DiffusingRdmBuilder = CkwsDiffusingRdmBuilder::create(CkwsRoadmap::create(Device), penetration);
      rdmBuilder = DiffusingRdmBuilder;
      DiffusingRdmBuilder->diffuseFromProblemGoal(biDiffuseCheckBox->IsChecked());
      isDiffusing = true;
      break;

    case CkppPlannerPanel::IPP : 
      ODEBUG2(" IPP roadmap builder selected.");
      DiffusingRdmBuilder = CkwsIPPRdmBuilder::create(CkwsRoadmap::create(Device), penetration);
      rdmBuilder = DiffusingRdmBuilder;
      DiffusingRdmBuilder->diffuseFromProblemGoal(biDiffuseCheckBox->IsChecked());
      isDiffusing = true;
      break;

    case CkppPlannerPanel::VISIBILITY : 
      ODEBUG2(" Visibility roadmap builder selected.");
      rdmBuilder = ChppVisRdmBuilder::create(CkwsRoadmap::create(Device), penetration);
      if (!rdmBuilder) {
	ODEBUG1(" Unable to create a visibility roadmap builder");
      }
      else {
	isDiffusing = false;
      }
      break; 
    case CkppPlannerPanel::PCA : 
      ODEBUG2(" PCA roadmap builder selected.");
      isDiffusing = true;
      if(LTBuilderCheckBox->IsChecked()){
	if(DiffusingBuilderCheckBox->IsChecked()) {
	  DiffusingRdmBuilder = 
	    CkwsPlusPCARdmBuilder<CkwsPlusLTRdmBuilder<CkwsDiffusingRdmBuilder> >::create(CkwsRoadmap::create(Device), penetration);
	  rdmBuilder = DiffusingRdmBuilder;
	  
	}
	else {
	  DiffusingRdmBuilder = 
	    CkwsPlusPCARdmBuilder<CkwsPlusLTRdmBuilder<CkwsIPPRdmBuilder> >::create(CkwsRoadmap::create(Device), penetration);
	  rdmBuilder = DiffusingRdmBuilder;
	}
      } 
      else { 
	if(DiffusingBuilderCheckBox->IsChecked()) {
	  DiffusingRdmBuilder = CkwsPlusPCARdmBuilder<CkwsDiffusingRdmBuilder>::create(CkwsRoadmap::create(Device), penetration);
	  rdmBuilder = DiffusingRdmBuilder;
	}
	else {
	  DiffusingRdmBuilder = CkwsPlusPCARdmBuilder<CkwsIPPRdmBuilder>::create(CkwsRoadmap::create(Device), penetration);
	  rdmBuilder = DiffusingRdmBuilder;
	}
      }
      DiffusingRdmBuilder->diffuseFromProblemGoal(biDiffuseCheckBox->IsChecked());
      break;

    case CkppPlannerPanel::LOCAL_TREES : 
      ODEBUG2(" Local tree roadmap builder selected.");
      isDiffusing = true;
      if(DiffusingBuilderCheckBox->IsChecked()) {
	DiffusingRdmBuilder = CkwsPlusLTRdmBuilder<CkwsDiffusingRdmBuilder>::create(CkwsRoadmap::create(Device), penetration);
      }
      else {
	DiffusingRdmBuilder = CkwsPlusLTRdmBuilder<CkwsIPPRdmBuilder>::create(CkwsRoadmap::create(Device), penetration);
      }
      DiffusingRdmBuilder->diffuseFromProblemGoal(biDiffuseCheckBox->IsChecked());
      rdmBuilder = DiffusingRdmBuilder;
      break;
    default: 
      ODEBUG2(" No roadmap builder selected.");
      isDiffusing = false;
      rdmBuilder = panel->getInterface()->hppPlanner()->roadmapBuilderIthProblem(i);
      if (DiffusingRdmBuilder = KIT_DYNAMIC_PTR_CAST(CkwsDiffusingRdmBuilder, rdmBuilder)) {
	isDiffusing = true;
      }
      if (rdmBuilder) {
	ODEBUG2(" Taking existing roadmap builder.");
      }
      break;
    }

    if(isDiffusing){
      CkwsShooterConfigListShPtr configList = CkwsShooterConfigList::create();
      switch(ShooterComboBox->GetCurrentSelection()){//To set the chosen shooter
      case CkppPlannerPanel::CONFIG_PATH : 
	DiffusingRdmBuilder->diffusionShooter(CkwsShooterConfigSpace::create());
	break;
      case CkppPlannerPanel::CONFIG_LIST : 
	updateConfigList(configList); DiffusingRdmBuilder->diffusionShooter(configList);
	//TODO : open a panel, a tab or a window, to choose the number of configurations and the configurations
	break;
      case CkppPlannerPanel::PATHS : 
	DiffusingRdmBuilder->diffusionShooter(CkwsShooterPath::create());
	break;
      case CkppPlannerPanel::ROADMAP_BOX : 
	DiffusingRdmBuilder->diffusionShooter(CkwsShooterRoadmapBox::create());
	break;
      case CkppPlannerPanel::ROADMAP_NODE : 
	DiffusingRdmBuilder->diffusionShooter(CkwsShooterRoadmapNodes::create());
	break;
      case CkppPlannerPanel::MULTI_SHHOTER : 
	DiffusingRdmBuilder->diffusionShooter(CkwsShooterMulti::create(updateShooterList()));
	break;
      case CkppPlannerPanel::ADAPTIVE_MULTI : 
	DiffusingRdmBuilder->diffusionShooter(CkwsShooterAdaptiveMulti::create(updateShooterList()));
	break;
      default: 
	break;
      }

      switch(PickerComboBox->GetCurrentSelection()){//To set the chosen picker
      case CkppPlannerPanel::BASIC_PICKER : 
	DiffusingRdmBuilder->diffusionNodePicker(CkwsPickerBasic::create());
	break;
      case CkppPlannerPanel::SMALLEST_TREE : 
	DiffusingRdmBuilder->diffusionNodePicker(CkwsPickerSmallestTree::create());
	break;
      default: 
	break;
      }
    }

    switch(SteeringMethodComboBox->GetCurrentSelection()){//To set the chosen steering Method
    case CkppPlannerPanel::LINEAR : 
      Device->steeringMethod(CkwsSMLinear::create());
      break;
    case CkppPlannerPanel::FLIC : 
      Device->steeringMethod(CflicSteeringMethod::create(IsOrientedCheckBox->IsChecked()));
      break;
    case CkppPlannerPanel::RS : 
      Device->steeringMethod(CreedsSheppSteeringMethod::create(RSRadiusSpinCtrl->GetValue(),IsOrientedCheckBox->IsChecked()));
      break;
    case CkppPlannerPanel::SLERP : 
      Device->steeringMethod(CkwsSMSlerp::create());
    default: 
      break;
    }

#if 0
    switch(DelegateComboBox->GetCurrentSelection()){//To set the chosen delegate
    case CkppPlannerPanel::GRAPHIC : 
      if(!rdmBuilder) { 
	ODEBUG1(" No roadmap builder selected or existing.");
      }
      else {
	ODEBUG2("Adding a graphic delegate to roadmap builder");
	rdmBuilder->addDelegate(new CkwsGraphicRoadmapDelegate());
      }
      break;
    default: 
      break;
    }
#endif

    switch(OptimizerComboBox->GetCurrentSelection()){//To set the chosen optimizer
    case CkppPlannerPanel::CLEAR : 
      panel->getInterface()->hppPlanner()->pathOptimizerIthProblem(i,CkwsClearOptimizer::create());
      break;
    case CkppPlannerPanel::RANDOM : 
      panel->getInterface()->hppPlanner()->pathOptimizerIthProblem(i,CkwsRandomOptimizer::create());
      break;
    case CkppPlannerPanel::ADAPTIVE_SHORTCUT : 
      panel->getInterface()->hppPlanner()->pathOptimizerIthProblem(i,CkwsAdaptiveShortcutOptimizer::create());
      break;
    default :
      break;
    }

    if (rdmBuilder) {
      panel->getInterface()->hppPlanner()->roadmapBuilderIthProblem(i, rdmBuilder, 
								    ShowRdmCheckBox->IsChecked());
    }
  }
  panel->Hide();
  panel->getInterface()->hppPlanner()->solve() ;
  //panel->Destroy();
}

/*_________________________________________________________*/
void CkppPlannerPanelController::SaveButtonEventHandler(wxCommandEvent& cancel){
//TODO : (Apply changes?) and save configuration in a file ?
}

/*_________________________________________________________*/
void CkppPlannerPanelController::OpenButtonEventHandler(wxCommandEvent& cancel){
  //TODO : open a saved configuration
}

/*_________________________________________________________*/
void CkppPlannerPanelController::AddPbButtonEventHandler(wxCommandEvent& cancel){//Add a problem at the end of hppProblemVector
  //TODO : add a hppproblem in the vector of problems (how to initialize this specific problem? how to choose the device parameter needed?)

  if(panel->getInterface()->attCommandInitBase != NULL){
    if(panel->getInterface()->attCommandInitBase->execute() == KD_ERROR) {
      ODEBUG1(" Cannot add a problem (init exec error)");
    }
    else{
      wxSpinCtrl* PbNumberSpinCtrl = dynamic_cast<wxSpinCtrl*>(panel->FindWindowByName("Problem number",panel.get()));
      PbNumberSpinCtrl->SetRange(0,panel->getInterface()->hppPlanner()->getNbHppProblems()-1); 
      PbNumberSpinCtrl->Refresh();
      problemId = panel->getInterface()->hppPlanner()->getNbHppProblems()-1;
    }
  }
  else {
    ODEBUG1(" No init command");
  }
  if(panel->getInterface()->attCommandSetConfigBase != NULL) {
    if(panel->getInterface()->attCommandSetConfigBase->execute() == KD_ERROR) {
      ODEBUG1(" Cannot add a problem (set config exec error)");
    }
  }
  else {
    ODEBUG1(" Cannot add a problem (no set config command)");
  }
}

/*_________________________________________________________*/
void CkppPlannerPanelController::ProblemSpinCtrlEventHandler(wxSpinEvent& cancel){
//TODO : read each parameter of the problem in order to fill controls with their value

  wxSpinCtrl * ProblemSpinCtrl = dynamic_cast<wxSpinCtrl*>(cancel.GetEventObject());

  problemId = ProblemSpinCtrl->GetValue();

}

/*_________________________________________________________*/

void CkppPlannerPanelController::addConfigurations(){

  wxWindow* magnetTab = panel->getNotebook()->GetPage(2);
  wxWindowList children = magnetTab->GetChildren();
  unsigned int nbTotalCheckBox = children.GetCount();

  CkppDeviceComponentShPtr robot = KIT_DYNAMIC_PTR_CAST(CkppDeviceComponent, getPanel()->getInterface()->hppPlanner()->robotIthProblem(problemId));

  for(unsigned int i=nbTotalCheckBox; i<robot->countConfigComponents();i++){
    
    wxString* label = new wxString("configuration : (");
    
    for(unsigned int j=0;j<robot->configComponent(i)->kwsConfig()->size();j++){//build label string for the checkbox
      
      *label<<floor(robot->configComponent(i)->kwsConfig()->dofValue(j));
      if(i<robot->configComponent(i)->kwsConfig()->size()-1) *label<<",";
      
    }
    *label<<")";
    
    
    wxCheckBox* confCheckbox = new wxCheckBox(panel->getNotebook()->GetPage(2), wxID_ANY, *label,wxDefaultPosition,wxDefaultSize,0,wxDefaultValidator,"Path Chk" );
    panel->addControl(panel->getNotebook()->GetPage(2),confCheckbox);
  }

}

/*_________________________________________________________*/

void CkppPlannerPanelController::updateConfigList(CkwsShooterConfigListShPtr configList){

  
  //add configs to the shooter magnets configs list
  
  CkppDeviceComponentShPtr robot = KIT_DYNAMIC_PTR_CAST(CkppDeviceComponent,panel->getRdmBuilder()->roadmap()->device());
  wxWindow* magnetTab = panel->getNotebook()->GetPage(2);
  wxWindowList children = magnetTab->GetChildren();
  unsigned int nbTotalConfig = robot->countConfigComponents();
  unsigned int nbTotalCheckBox = children.GetCount();
  unsigned int nbCheckedConf = 0;

  if(nbTotalConfig == nbTotalCheckBox){

    for(unsigned int i=0; i<nbTotalCheckBox;i++){
      if(dynamic_cast<wxCheckBox*>(children.Item(i)->GetData())->IsChecked()){
	configList->add(*(robot->configComponent(i)->kwsConfig().get()),100.0,0.2);
	nbCheckedConf++;
      }
    }

    ODEBUG1(nbCheckedConf<<" configurations selected.");

  }
  else {
    ODEBUG1(" ERROR : Number of available configurations is different from number of choice in the magnet tab.");
    ODEBUG1(" ERROR : Try to reselect Magnet List as a shooter.");
  }
}

/*_________________________________________________________*/

CkwsShooterMulti::TWeightedShooterList CkppPlannerPanelController::updateShooterList(){

  CkwsShooterMulti::TWeightedShooterList shooterList;
  CkwsShooterMulti::TWeightedShooter shooterTemp;

  //add shooters to the shooters list
  
  CkppDeviceComponentShPtr robot = KIT_DYNAMIC_PTR_CAST(CkppDeviceComponent,panel->getRdmBuilder()->roadmap()->device());
  wxWindow* shooterTab = panel->getNotebook()->GetPage(1);
  wxWindowList children = shooterTab->GetChildren();
  unsigned int nbTotalShooters = children.GetCount()/2;
  unsigned int nbCheckedShooters = 0;

  CkwsShooterConfigListShPtr configList = CkwsShooterConfigList::create(); //For the magnet shooter, if it is selected for the multi shooter.

    for(unsigned int i=0; i<nbTotalShooters;i++){
      if(dynamic_cast<wxCheckBox*>(children.Item(i)->GetData())->IsChecked()){

	switch(i){

	case 0 : shooterTemp = CkwsShooterMulti::TWeightedShooter(dynamic_cast<wxCheckBox*>(children.Item(i)->GetData())->GetValue(),CkwsShooterConfigSpace::create()); 
	  break;
	case 1 : 
	  updateConfigList(configList);
	  shooterTemp = CkwsShooterMulti::TWeightedShooter(dynamic_cast<wxCheckBox*>(children.Item(i)->GetData())->GetValue(),configList); 
	  break;
	case 2 : shooterTemp = CkwsShooterMulti::TWeightedShooter(dynamic_cast<wxCheckBox*>(children.Item(i)->GetData())->GetValue(),CkwsShooterPath::create()); 
	  break;
	case 3 : shooterTemp = CkwsShooterMulti::TWeightedShooter(dynamic_cast<wxCheckBox*>(children.Item(i)->GetData())->GetValue(),CkwsShooterRoadmapBox::create()); 
	  break;
	case 4 : shooterTemp = CkwsShooterMulti::TWeightedShooter(dynamic_cast<wxCheckBox*>(children.Item(i)->GetData())->GetValue(),CkwsShooterRoadmapNodes::create()); 
	  break;
	case 5 : shooterTemp = CkwsShooterMulti::TWeightedShooter(dynamic_cast<wxCheckBox*>(children.Item(i)->GetData())->GetValue(),CkwsShooterMulti::create(shooterList)); 
	  break;
	case 6 : shooterTemp = CkwsShooterMulti::TWeightedShooter(dynamic_cast<wxCheckBox*>(children.Item(i)->GetData())->GetValue(),CkwsShooterAdaptiveMulti::create(shooterList)); 
	  break;
	default: shooterTemp = CkwsShooterMulti::TWeightedShooter(dynamic_cast<wxCheckBox*>(children.Item(i)->GetData())->GetValue(),CkwsShooterConfigSpace::create()); 
	  break;

	}

	shooterList.push_back(shooterTemp);
	nbCheckedShooters++;

      }
    }

    ODEBUG1(nbCheckedShooters<<" shooters  selected.");

  return shooterList;

}


/*_________________________________________________________*/


/***********************************************************/
