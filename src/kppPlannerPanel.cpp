//
// Copyright (c) 2005, 2006, 2007, 2008, 2009, 2010, 2011, 2012, 2013 CNRS
// Authors: David Flavigne, Thomas Moulard and Florent Lamiraux
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

  wxNotebook * tabs = new wxNotebook (MainPage, CHOICES_NOTEBOOK ,
				      wxDefaultPosition, wxDefaultSize,
				      wxNB_TOP,
				      wxString::FromAscii ("- Options -"));

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
  wxComboBox * RoadmapBuilderComboBox = 
    new wxComboBox(MainPage, RDM_BUILDER_COMBO_BOX,
		   wxString::FromAscii ("Choose RdmBuilder"), wxDefaultPosition,
		   wxDefaultSize, buildersArray, (long int)(wxCB_DROPDOWN |
							    wxCB_READONLY),
		   wxDefaultValidator ,
		   wxString::FromAscii ("Roadmap Builder"));
  wxComboBox * ShooterComboBox = 
    new wxComboBox (MainPage, SHOOTER_COMBO_BOX,
		   wxString::FromAscii ("Choose Shooter"), wxDefaultPosition,
		   wxDefaultSize, shootersArray, (long int)(wxCB_DROPDOWN |
							    wxCB_READONLY),
		   wxDefaultValidator,
		   wxString::FromAscii ("Diffusion Shooter"));

  wxComboBox * PickerComboBox =
    new wxComboBox (MainPage, PICKER_COMBO_BOX,
		    wxString::FromAscii ("Choose Picker"), wxDefaultPosition,
		    wxDefaultSize,pickersArray, (long int)(wxCB_DROPDOWN |
							   wxCB_READONLY),
		    wxDefaultValidator,
		    wxString::FromAscii ("Diffusion Picker"));

  wxComboBox * SteeringMethodComboBox =
    new wxComboBox (MainPage, STEERING_COMBO_BOX,
		    wxString::FromAscii ("Choose Steering method"),
		    wxDefaultPosition, wxDefaultSize, steeringsArray,
		    (long int)(wxCB_DROPDOWN | wxCB_READONLY),
		    wxDefaultValidator,
		    wxString::FromAscii ("Steering Method"));

  wxComboBox * DelegateComboBox =
    new wxComboBox (MainPage, DELEGATE_COMBO_BOX,
		    wxString::FromAscii ("Choose Delegate"), wxDefaultPosition,
		    wxDefaultSize, delegatesArray, (long int)(wxCB_DROPDOWN |
							      wxCB_READONLY),
		    wxDefaultValidator,wxString::FromAscii ("Delegates"));

  wxComboBox * OptimizerComboBox =
    new wxComboBox (MainPage, OPTIMIZER_COMBO_BOX,
		    wxString::FromAscii ("Choose Path Optimizer"),
		    wxDefaultPosition, wxDefaultSize, optimizersArray,
		    (long int)(wxCB_DROPDOWN | wxCB_READONLY),
		    wxDefaultValidator,
		    wxString::FromAscii ("Path Optimizer"));

  wxCheckBox * ShowRdmCheckBox =
    new wxCheckBox (tab1_options, RDM_CHECK_BOX,
		    wxString::FromAscii ("Show Roadmap"), wxDefaultPosition,
		    wxDefaultSize, 0, wxDefaultValidator,
		    wxString::FromAscii ("H/S Rdm Chk"));

  wxCheckBox * BidiffuseCheckBox =
    new wxCheckBox (tab1_options, BIDIFF_CHECK_BOX,
		    wxString::FromAscii ("Bi-diffuse"), wxDefaultPosition,
		    wxDefaultSize, 0, wxDefaultValidator,
		    wxString::FromAscii ("BiDiff Chk"));

  wxCheckBox * SolveAllCheckBox =
    new wxCheckBox (tab1_options, SOLVE_ALL_CHECK_BOX,
		    wxString::FromAscii ("Solve All"), wxDefaultPosition,
		    wxDefaultSize, 0, wxDefaultValidator,
		    wxString::FromAscii ("SolveAll Chk"));
  
  //specific for multishooters (following items will appear in a tab in case of a multishooter choice)
  wxCheckBox * shooterConfigSpaceCheckBox =
    new wxCheckBox (tab2_shooters, CS_SHOOTER_CHECK_BOX,
		    wxString::FromAscii ("Basic"), wxDefaultPosition,
		    wxDefaultSize, 0, wxDefaultValidator,
		    wxString::FromAscii ("Basic Chk"));

  wxCheckBox * shooterConfigListCheckBox =
    new wxCheckBox (tab2_shooters, CL_SHOOTER_CHECK_BOX,
		    wxString::FromAscii ("Magnets"), wxDefaultPosition,
		    wxDefaultSize, 0, wxDefaultValidator,
		    wxString::FromAscii ("Magnet Chk"));

  wxCheckBox * shooterPathCheckBox  =
    new wxCheckBox (tab2_shooters, P_SHOOTER_CHECK_BOX,
		    wxString::FromAscii ("Path"), wxDefaultPosition,
		    wxDefaultSize, 0, wxDefaultValidator,
		    wxString::FromAscii ("Path Chk"));

  wxCheckBox * shooterRoadmapBoxCheckBox =
    new wxCheckBox(tab2_shooters, RB_SHOOTER_CHECK_BOX,
		   wxString::FromAscii ("RdmBox"), wxDefaultPosition,
		   wxDefaultSize, 0, wxDefaultValidator,
		   wxString::FromAscii ("RdmBox Chk"));

  wxCheckBox * shooterRoadmapNodesCheckBox =
    new wxCheckBox (tab2_shooters, RN_SHOOTER_CHECK_BOX,
		    wxString::FromAscii ("RdmNodes"), wxDefaultPosition,
		    wxDefaultSize, 0, wxDefaultValidator,
		    wxString::FromAscii ("RdmNodes Chk"));

  //To set corresponding weights
  wxSpinCtrl* weightConfigSpaceBox =
    new wxSpinCtrl (tab2_shooters, CS_SPIN_CTRL, wxString::FromAscii (""),
		    wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 20, 0,
		    wxString::FromAscii ("Basic Wgt"));

  wxSpinCtrl* weightConfigListBox =
    new wxSpinCtrl (tab2_shooters, CL_SPIN_CTRL, wxString::FromAscii (""),
		    wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 20, 0,
		    wxString::FromAscii ("Magnet Wgt"));

  wxSpinCtrl* weightPathBox =
    new wxSpinCtrl (tab2_shooters, P_SPIN_CTRL, wxString::FromAscii (""),
		    wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 20, 0,
		    wxString::FromAscii ("Path Wgt"));

  wxSpinCtrl* weightRdmBoxBox =
    new wxSpinCtrl (tab2_shooters, RB_SPIN_CTRL, wxString::FromAscii (""),
		    wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 20, 0,
		    wxString::FromAscii ("RdmBox Wgt"));

  wxSpinCtrl* weightRdmNodesBox =
    new wxSpinCtrl (tab2_shooters, RN_SPIN_CTRL, wxString::FromAscii (""),
		    wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 20, 0,
		    wxString::FromAscii ("RdmNodes Wgt"));

  wxSpinCtrl* ProblemSpinCtrl =
    new wxSpinCtrl (MainPage, PROBLEM_SPIN_CTRL, wxString::FromAscii (""),
		    wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 20, 0,
		    wxString::FromAscii ("Problem number"));

  ProblemSpinCtrl->SetRange(0,0);

  wxButton * StartButton = new wxButton (MainPage, START_BUTTON,
					 wxString::FromAscii ("Start"));
  wxButton * SaveButton = new wxButton (MainPage, SAVE_BUTTON,
					wxString::FromAscii ("Save"));
  wxButton * OpenButton = new wxButton (MainPage, OPEN_BUTTON,
					wxString::FromAscii ("Open"));
  wxButton * AddProblemButton = new wxButton (MainPage, ADD_PB_BUTTON,
					      wxString::FromAscii
					      ("Add problem"));

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

  tabs->AddPage(tab1_options, wxString::FromAscii ("Options"));
  tabs->AddPage(tab2_shooters, wxString::FromAscii ("Shooters Selection"));
  tabs->AddPage(tab3_magnets, wxString::FromAscii ("Configurations Selection"));
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

  attPicker[CkppPlannerPanel::BASIC_PICKER] =
    wxString::FromAscii("Basic Picker");
  attPicker[CkppPlannerPanel::SMALLEST_TREE] =
    wxString::FromAscii("Smallest Tree");

  attShooters[CkppPlannerPanel::CONFIG_PATH] =
    wxString::FromAscii("Config Space");
  attShooters[CkppPlannerPanel::CONFIG_LIST] =
    wxString::FromAscii("Config List");
  attShooters[CkppPlannerPanel::PATHS] =
    wxString::FromAscii("Paths");
  attShooters[CkppPlannerPanel::ROADMAP_BOX] =
    wxString::FromAscii("Roadmap Box");
  attShooters[CkppPlannerPanel::ROADMAP_NODE] =
    wxString::FromAscii("Roadmap Nodes");
  attShooters[CkppPlannerPanel::MULTI_SHHOTER] =
    wxString::FromAscii("Multi Shooter");
  attShooters[CkppPlannerPanel::ADAPTIVE_MULTI] =
    wxString::FromAscii("Adaptative Multi");

  attbuilders[CkppPlannerPanel::BASIC_RDMBUILDER] =
    wxString::FromAscii("Basic Rdm Builder");
  attbuilders[CkppPlannerPanel::DIFFUSING] =
    wxString::FromAscii("Diffusing Rdm builder");
  attbuilders[CkppPlannerPanel::IPP] =
    wxString::FromAscii("IPP Rdm Builder");
  attbuilders[CkppPlannerPanel::VISIBILITY] =
    wxString::FromAscii("Visibility Rdm Builder");
  attbuilders[CkppPlannerPanel::PCA] =
    wxString::FromAscii("PCA Rdm Builder");
  attbuilders[CkppPlannerPanel::LOCAL_TREES] =
    wxString::FromAscii("LocalTrees Rdm Builder");

  attSteeringMethods[CkppPlannerPanel::LINEAR] = wxString::FromAscii("Linear");
  attSteeringMethods[CkppPlannerPanel::FLIC] = wxString::FromAscii("Flic");
  attSteeringMethods[CkppPlannerPanel::RS] =
    wxString::FromAscii("Reeds & Shepp");
  attSteeringMethods[CkppPlannerPanel::SLERP] = wxString::FromAscii("SLERP");

  attRdmBuilderDelegates[CkppPlannerPanel::GRAPHIC] =
    wxString::FromAscii("Graphic Roadmap Delegate");

  attPathOptimizers[CkppPlannerPanel::CLEAR] =
    wxString::FromAscii("Clear Optimizer");
  attPathOptimizers[CkppPlannerPanel::RANDOM] =
    wxString::FromAscii("Random Optimizer");
  attPathOptimizers[CkppPlannerPanel::ADAPTIVE_SHORTCUT] =
    wxString::FromAscii("Adaptive Shortcut Optimizer");

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
