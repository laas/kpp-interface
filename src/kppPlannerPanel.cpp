/*
  Copyright CNRS-LAAS

  Authors: David Flavigne and Florent Lamiraux
*/

/**************************INCLUDES*************************/

#include "kppInterface/kppPlannerPanelController.h"
#include "kppInterface/kppPlannerPanel.h"

#include "wx/sizer.h"
#include "wx/bmpcbox.h"
#include "wx/checkbox.h"
#include "wx/button.h"
#include "wx/arrstr.h"
#include "wx/window.h"
#include "wx/event.h"
#include "wx/spinctrl.h"
#include "wx/notebook.h"
#include "wx/panel.h"

#include <iostream>

using namespace std;


/***********************************************************/

/*************************METHODS***************************/
/*_________________________________________________________*/

CkppPlannerPanel::~CkppPlannerPanel(){

  rdm.reset();
  attKpp = NULL;
  delete notebook; 
  delete pages; 

}

/*_________________________________________________________*/

void CkppPlannerPanel::build (wxBoxSizer *i_sizer){

  //these wxArrayStrings are for the comboBoxes. If you want to add choices in one of these combo boxes, you must increment the first argument.
  const wxArrayString buildersArray(6,attbuilders);
  const wxArrayString shootersArray(7,attShooters);
  const wxArrayString pickersArray(2,attPicker);
  const wxArrayString steeringsArray(4,attSteeringMethods);
  const wxArrayString delegatesArray(1,attRdmBuilderDelegates);
  const wxArrayString optimizersArray(3,attPathOptimizers);

  wxPanel * MainPage = new wxPanel(this, wxID_ANY);

  pages = MainPage;

  wxNotebook * tabs = new wxNotebook(MainPage, CHOICES_NOTEBOOK , wxDefaultPosition,wxDefaultSize,wxNB_TOP,"- Options -" );

  notebook = tabs;

  wxPanel * tab1_options = new wxPanel(tabs, wxID_ANY);
  wxPanel * tab2_shooters = new wxPanel(tabs, wxID_ANY);
  wxPanel * tab3_magnets = new wxPanel(tabs, wxID_ANY);

  wxBoxSizer * MainPageSizer = new wxBoxSizer(wxVERTICAL);
  wxBoxSizer * ProblemBoxSizer = new wxBoxSizer(wxHORIZONTAL); //will contains the spinctrl to choose the pb number and a button "add pb"
  wxBoxSizer * BuilderSizer = new wxBoxSizer(wxVERTICAL);//will contain various combo boxes to configure the planner
  wxGridSizer * OptionsSizer = new wxGridSizer(2,2,1,1);//will contain options such as hide show roadmaps, solve all pbs; bi diffuse ...
  wxBoxSizer * TabsSizer = new wxBoxSizer(wxHORIZONTAL);//will contains a page with tabs to select magnet points and/or multiple shooters
  wxBoxSizer * ButtonsSizer = new wxBoxSizer(wxHORIZONTAL);//will contain various buttons

  //specific for multishooters
  wxBoxSizer* ShootersCheckBoxes = new wxBoxSizer(wxVERTICAL); //Contains checkboxes
  wxBoxSizer* ShootersWeightsSpinCtrls = new wxBoxSizer(wxVERTICAL); //Contains spinctrls
  wxBoxSizer* ShootersCtrls = new wxBoxSizer(wxHORIZONTAL); //Contains the two previous sizers

  //specific for magnet points (configList)
  wxBoxSizer* MagnetsSizer = new wxBoxSizer(wxVERTICAL);

  /*******CONTROLS**********/
  wxComboBox * RoadmapBuilderComboBox = new wxComboBox(MainPage,RDM_BUILDER_COMBO_BOX,"Choose RdmBuilder",wxDefaultPosition,wxDefaultSize,buildersArray, (long int)(wxCB_DROPDOWN | wxCB_READONLY),wxDefaultValidator , "Roadmap Builder");
  wxComboBox * ShooterComboBox = new wxComboBox(MainPage,SHOOTER_COMBO_BOX,"Choose Shooter",wxDefaultPosition,wxDefaultSize,shootersArray, (long int)(wxCB_DROPDOWN | wxCB_READONLY),wxDefaultValidator ,"Diffusion Shooter"); //Must not appear in case of a basicRdmBuilder -> PRM
  wxComboBox * PickerComboBox = new wxComboBox(MainPage,PICKER_COMBO_BOX,"Choose Picker",wxDefaultPosition,wxDefaultSize,pickersArray, (long int)(wxCB_DROPDOWN | wxCB_READONLY),wxDefaultValidator ,"Diffusion Picker"); //Must not appear in case of a basicRdmBuilder -> PRM
  wxComboBox * SteeringMethodComboBox = new wxComboBox(MainPage,STEERING_COMBO_BOX,"Choose Steering method",wxDefaultPosition,wxDefaultSize,steeringsArray, (long int)(wxCB_DROPDOWN | wxCB_READONLY),wxDefaultValidator ,"Steering Method");
  wxComboBox * DelegateComboBox = new wxComboBox(MainPage,DELEGATE_COMBO_BOX,"Choose Delegate",wxDefaultPosition,wxDefaultSize,delegatesArray, (long int)(wxCB_DROPDOWN | wxCB_READONLY),wxDefaultValidator ,"Delegates");
  wxComboBox * OptimizerComboBox = new wxComboBox(MainPage,OPTIMIZER_COMBO_BOX,"Choose Path Optimizer",wxDefaultPosition,wxDefaultSize,optimizersArray, (long int)(wxCB_DROPDOWN | wxCB_READONLY),wxDefaultValidator ,"Path Optimizer");

  wxCheckBox * ShowRdmCheckBox = new wxCheckBox(tab1_options,RDM_CHECK_BOX,"Show Roadmap",wxDefaultPosition,wxDefaultSize,0,wxDefaultValidator,"H/S Rdm Chk");
  wxCheckBox * BidiffuseCheckBox = new wxCheckBox(tab1_options,BIDIFF_CHECK_BOX,"Bi-diffuse",wxDefaultPosition,wxDefaultSize,0,wxDefaultValidator,"BiDiff Chk");//Must not appear in case of a basicRdmBuilder -> PRM
  wxCheckBox * SolveAllCheckBox = new wxCheckBox(tab1_options,SOLVE_ALL_CHECK_BOX,"Solve All",wxDefaultPosition,wxDefaultSize,0,wxDefaultValidator,"SolveAll Chk");
  
  //specific for multishooters (following items will appear in a tab in case of a multishooter choice)
  wxCheckBox * shooterConfigSpaceCheckBox = new wxCheckBox(tab2_shooters,CS_SHOOTER_CHECK_BOX,"Basic",wxDefaultPosition,wxDefaultSize,0,wxDefaultValidator,"Basic Chk");
  wxCheckBox * shooterConfigListCheckBox = new wxCheckBox(tab2_shooters,CL_SHOOTER_CHECK_BOX,"Magnets",wxDefaultPosition,wxDefaultSize,0,wxDefaultValidator,"Magnet Chk");
  wxCheckBox * shooterPathCheckBox  = new wxCheckBox(tab2_shooters,P_SHOOTER_CHECK_BOX,"Path",wxDefaultPosition,wxDefaultSize,0,wxDefaultValidator,"Path Chk");
  wxCheckBox * shooterRoadmapBoxCheckBox = new wxCheckBox(tab2_shooters,RB_SHOOTER_CHECK_BOX,"RdmBox",wxDefaultPosition,wxDefaultSize,0,wxDefaultValidator,"RdmBox Chk");
  wxCheckBox * shooterRoadmapNodesCheckBox = new wxCheckBox(tab2_shooters,RN_SHOOTER_CHECK_BOX,"RdmNodes",wxDefaultPosition,wxDefaultSize,0,wxDefaultValidator,"RdmNodes Chk");

  //To set corresponding weights
  wxSpinCtrl* weightConfigSpaceBox = new wxSpinCtrl(tab2_shooters,CS_SPIN_CTRL,"",wxDefaultPosition,wxDefaultSize,wxSP_ARROW_KEYS,1,20,0,"Basic Wgt");
  wxSpinCtrl* weightConfigListBox = new wxSpinCtrl(tab2_shooters,CL_SPIN_CTRL,"",wxDefaultPosition,wxDefaultSize,wxSP_ARROW_KEYS,1,20,0,"Magnet Wgt");
  wxSpinCtrl* weightPathBox = new wxSpinCtrl(tab2_shooters,P_SPIN_CTRL,"",wxDefaultPosition,wxDefaultSize,wxSP_ARROW_KEYS,1,20,0,"Path Wgt");
  wxSpinCtrl* weightRdmBoxBox = new wxSpinCtrl(tab2_shooters,RB_SPIN_CTRL,"",wxDefaultPosition,wxDefaultSize,wxSP_ARROW_KEYS,1,20,0,"RdmBox Wgt");
  wxSpinCtrl* weightRdmNodesBox = new wxSpinCtrl(tab2_shooters,RN_SPIN_CTRL,"",wxDefaultPosition,wxDefaultSize,wxSP_ARROW_KEYS,1,20,0,"RdmNodes Wgt");

  wxSpinCtrl* ProblemSpinCtrl = new wxSpinCtrl(MainPage,PROBLEM_SPIN_CTRL,"",wxDefaultPosition,wxDefaultSize,wxSP_ARROW_KEYS,1,20,0,"Problem number");
  ProblemSpinCtrl->SetRange(0,0);

  wxButton * StartButton = new wxButton(MainPage, START_BUTTON,"Start");
  wxButton * SaveButton = new wxButton(MainPage, SAVE_BUTTON,"Save");
  wxButton * OpenButton = new wxButton(MainPage, OPEN_BUTTON,"Open");
  wxButton * AddProblemButton = new wxButton(MainPage, ADD_PB_BUTTON,"Add problem");

  /******** DEFINING LAYOUT ***********/
  
  //MainPage
  ProblemBoxSizer->Add(ProblemSpinCtrl);
  ProblemBoxSizer->Add(AddProblemButton);
  BuilderSizer->Add(RoadmapBuilderComboBox);
  BuilderSizer->Add(ShooterComboBox);
  BuilderSizer->Add(PickerComboBox);
  BuilderSizer->Add(SteeringMethodComboBox);
  BuilderSizer->Add(DelegateComboBox);
  BuilderSizer->Add(OptimizerComboBox);
  ButtonsSizer->Add(StartButton);
  ButtonsSizer->Add(SaveButton);
  ButtonsSizer->Add(OpenButton); 

  //tabs
  //------------tab 1 ---------------------
  OptionsSizer->Add(ShowRdmCheckBox);
  OptionsSizer->Add(BidiffuseCheckBox);
  OptionsSizer->Add(SolveAllCheckBox);
  //------------tab 2 ---------------------
  ShootersCheckBoxes->Add(shooterConfigSpaceCheckBox);
  ShootersCheckBoxes->Add(shooterConfigListCheckBox);
  ShootersCheckBoxes->Add(shooterPathCheckBox);
  ShootersCheckBoxes->Add(shooterRoadmapBoxCheckBox);
  ShootersCheckBoxes->Add(shooterRoadmapNodesCheckBox);
  ShootersWeightsSpinCtrls->Add(weightConfigSpaceBox);
  ShootersWeightsSpinCtrls->Add(weightConfigListBox);
  ShootersWeightsSpinCtrls->Add(weightPathBox);
  ShootersWeightsSpinCtrls->Add(weightRdmBoxBox);
  ShootersWeightsSpinCtrls->Add(weightRdmNodesBox);
  ShootersCtrls->Add(ShootersCheckBoxes);
  ShootersCtrls->Add(ShootersWeightsSpinCtrls);
  //------------tab 3 ---------------------
  //tab 3 contains magnets points in the case of a config list shooter
  //and is built dynamically

  tab1_options->SetSizer(OptionsSizer);
  tab2_shooters->SetSizer(ShootersCtrls);
  tab3_magnets->SetSizer(MagnetsSizer);
  ShootersCtrls->Layout();
  OptionsSizer->Layout();
  MagnetsSizer->Layout();

  tabs->AddPage(tab1_options,"Options");
  tabs->AddPage(tab2_shooters,"Shooters Selection");
  tabs->AddPage(tab3_magnets,"Configurations Selection");
  TabsSizer->Add(tabs);

  tab1_options->Show(true);
  tab2_shooters->Show(false);
  tab3_magnets->Show(false);

  MainPageSizer->Layout();
  MainPageSizer->Add(ProblemBoxSizer);
  MainPageSizer->Add(BuilderSizer);
  MainPageSizer->Add(TabsSizer);
  MainPageSizer->Add(ButtonsSizer);
  MainPage->SetSizer(MainPageSizer);

  i_sizer->Layout();
  i_sizer->Add(MainPage);
  i_sizer->Show(true);

}
/*_________________________________________________________*/

CkppPlannerPanel::CkppPlannerPanel(wxWindow *i_parent, const CkppPlannerPanelControllerShPtr &i_controller):CkppPanel(i_parent,i_controller){

  isRdmSet = false;
  isInterfaceSet = false;

  attPicker[CkppPlannerPanel::BASIC_PICKER] = wxString("Basic Picker");
  attPicker[CkppPlannerPanel::SMALLEST_TREE] = wxString("Smallest Tree");

  attShooters[CkppPlannerPanel::CONFIG_PATH] = wxString("Config Space");
  attShooters[CkppPlannerPanel::CONFIG_LIST] = wxString("Config List");
  attShooters[CkppPlannerPanel::PATHS] = wxString("Paths");
  attShooters[CkppPlannerPanel::ROADMAP_BOX] = wxString("Roadmap Box");
  attShooters[CkppPlannerPanel::ROADMAP_NODE] = wxString("Roadmap Nodes");
  attShooters[CkppPlannerPanel::MULTI_SHHOTER] = wxString("Multi Shooter");
  attShooters[CkppPlannerPanel::ADAPTIVE_MULTI] = wxString("Adaptative Multi");

  attbuilders[CkppPlannerPanel::BASIC_RDMBUILDER] = wxString("Basic Rdm Builder");
  attbuilders[CkppPlannerPanel::DIFFUSING] = wxString("Diffusing Rdm builder");
  attbuilders[CkppPlannerPanel::IPP] = wxString("IPP Rdm Builder");
  attbuilders[CkppPlannerPanel::VISIBILITY] = wxString("Visibility Rdm Builder");
  attbuilders[CkppPlannerPanel::PCA] = wxString("PCA Rdm Builder");
  attbuilders[CkppPlannerPanel::LOCAL_TREES] = wxString("LocalTrees Rdm Builder");

  attSteeringMethods[CkppPlannerPanel::LINEAR] = wxString("Linear");
  attSteeringMethods[CkppPlannerPanel::FLIC] = wxString("Flic");
  attSteeringMethods[CkppPlannerPanel::RS] = wxString("Reeds & Shepp");
  attSteeringMethods[CkppPlannerPanel::SLERP] = wxString("SLERP");

  attRdmBuilderDelegates[CkppPlannerPanel::GRAPHIC] = wxString("Graphic Roadmap Delegate");

  attPathOptimizers[CkppPlannerPanel::CLEAR] = wxString("Clear Optimizer");
  attPathOptimizers[CkppPlannerPanel::RANDOM] = wxString("Random Optimizer");
  attPathOptimizers[CkppPlannerPanel::ADAPTIVE_SHORTCUT] = wxString("Adaptive Shortcut Optimizer");

}
/*_________________________________________________________*/
CkppPlannerPanelControllerShPtr CkppPlannerPanel::commandPanelController() const{
	return KIT_DYNAMIC_PTR_CAST( CkppPlannerPanelController, controller());
}
/*_________________________________________________________*/

void CkppPlannerPanel::addControl(wxWindow* window, wxObject* object){

  window->GetSizer()->Add((wxWindow*)(object));
  window->GetSizer()->Layout();

}

/*_________________________________________________________*/
/***********************************************************/
