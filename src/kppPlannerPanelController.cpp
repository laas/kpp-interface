
/**************************INCLUDES*************************/

#include "kppInterface/kppPlannerPanelController.h"
#include "kppInterface/kppPlannerPanel.h"

#include "KineoGUI/kppMainWindowController.h"
#include "KineoModel/kppConfigComponent.h"
#include "KineoController/kppDocument.h"
#include "KineoModel/kppModelTree.h"

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

/***********************************************************/

/*************************METHODS***************************/
/*_________________________________________________________*/
CkppPlannerPanelController::~CkppPlannerPanelController(){

}

/*_________________________________________________________*/

CkppPlannerPanelController::CkppPlannerPanelController(){
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

ktStatus CkppPlannerPanelController::init (const CkppPlannerPanelControllerWkPtr &i_weakPtr){

  ktStatus success = KD_ERROR;
  success = CkppWindowController::init( i_weakPtr );

  if( success == KD_OK )
    {
      m_weakPtr = i_weakPtr;
      problemId = 0;

      doesSelfDeleteOnClose(true);
      doesReleaseWindowOnClose(false);

      windowTitle("Planner Configuration Panel");

    }

  return success;
}

/*_________________________________________________________*/
void CkppPlannerPanelController::loadWindow(){

  cout<<"Creating panel ...";
  CkppPlannerPanel* myPanel;
  
  myPanel = new CkppPlannerPanel(parentWindow(), m_weakPtr.lock());
  myPanel->finalize();

  CkppPlannerPanelShPtr ptr(myPanel);

  setPanel(ptr);

  window(myPanel);
  cout<<"Done"<<endl;
  
}

/*_________________________________________________________*/

void CkppPlannerPanelController::builderComboBoxEventHandler(wxCommandEvent& cancel){
//TODO : create an instance of the chosen builder (using the vector defined in kppPlannerPanel.h?)
//  Add this builder to the selected problem

}

/*_________________________________________________________*/
void CkppPlannerPanelController::shooterComboBoxEventHandler(wxCommandEvent& cancel){
//TODO : create an instance of the chosen shooter and add it to the problem
// Verify that the builder is a diffusion one. in case of a config list shooter
// look for saved configurations in the modeltree and create checkbox for each one
// in the "magnets tab"

  wxPanel* mainPage = panel->getMainPage();
  wxComboBox* shooterCombo = dynamic_cast<wxComboBox*>(cancel.GetEventObject());

  if(!mainPage){
    if(!(panel->GetSizer()->GetChildren())[0]) cout<<"ERROR : The panel has no children - "; else cout<<"ERROR : ";
    cout<<"unable to retrieve the main page of the panel"<<endl;
  }
  else{
    
    switch(shooterCombo->GetSelection()){
      
    case 1 : //selecting magnet shooter
      cout<<"Magnet Shooter has been selected : choose magnet configurations in the new tab"<<endl;

      addConfigurations();

      panel->getNotebook()->GetPage(1)->Show(false);
      panel->getNotebook()->GetPage(2)->Show(true);
      break;
      
    case 5 :  //selecting multi shooter
      cout<<"Multi Shooter has been selected : choose shooters in the new tab"<<endl;
      panel->getNotebook()->GetPage(2)->Show(false);
      panel->getNotebook()->GetPage(1)->Show(true);
      break;
      
    case 6 : //selecting adaptive multi shooter
      cout<<"Adaptive Multi Shooter has been selected : choose shooters in the new tab"<<endl;
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
}

/*_________________________________________________________*/
void CkppPlannerPanelController::steeringComboBoxEventHandler(wxCommandEvent& cancel){
//TODO : create an instance of the chosen steering method and add it to the problem
// specific behaviour for reed & sheep (open a new tab) -> ask mathieu + make this as an example
}

/*_________________________________________________________*/
void CkppPlannerPanelController::delegateComboBoxEventHandler(wxCommandEvent& cancel){
//TODO : create an instance of the chosen steering method and add it to the problem
// specific behaviour for reed & sheep (open a new tab) -> ask mathieu + make this as an example
}

/*_________________________________________________________*/
void CkppPlannerPanelController::optimizerComboBoxEventHandler(wxCommandEvent& cancel){
//TODO : create an instance of the chosen path optimizer and add it to the problem
}

/*_________________________________________________________*/
void CkppPlannerPanelController::RoadmapCheckBoxEventHandler(wxCommandEvent& cancel){
//TODO : enable roadmap display.
 /*  if(panel->interfaceIsSet() && panel->getInterface()){

      if(cancel.IsChecked()){
	panel->getInterface()->showRoadmap(problemId);
	cout<<"Showing Roadmap"<<endl;
      }
      else panel->getInterface()->hideRoadmap(problemId);
    }*/
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
    
    cout<<"Magnet Shooter has been selected : choose magnet configurations in the new tab"<<endl;
    addConfigurations();
    panel->getNotebook()->GetPage(2)->Show(true);

  }else{

    cout<<"Magnet Shooter has been deselected"<<endl;
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

  wxCheckBox * SolveAllCheckBox = dynamic_cast<wxCheckBox*>(panel->FindWindowByName("SolveAll Chk",panel.get()));
  int FirstProblem, LastProblem;
  if(SolveAllCheckBox->IsChecked()){
  cout<<"Solve All"<<problemId<<endl;
    FirstProblem = 0;
    LastProblem = panel->getInterface()->hppPlanner()->getNbHppProblems();
  }else{
  cout<<"Solve Only Problem "<<problemId<<endl;
    FirstProblem = problemId;
    LastProblem = problemId+1;
  }

  for(int i=FirstProblem; i<LastProblem; i++){

    cout<<"Configuring Problem "<<problemId<<endl;
    CkwsRoadmapBuilderShPtr RdmBuilder;
    CkwsDiffusingRdmBuilderShPtr DiffusingRdmBuilder;
    bool isDiffusing = true;
    CkwsDeviceShPtr Device = panel->getInterface()->hppPlanner()->robotIthProblem(i);
    
    wxComboBox* RoadmapBuilderComboBox = dynamic_cast<wxComboBox*>(panel->FindWindowByName("Roadmap Builder",panel.get()));
    wxComboBox* ShooterComboBox  = dynamic_cast<wxComboBox*>(panel->FindWindowByName("Diffusion Shooter",panel.get()));
    wxComboBox* PickerComboBox = dynamic_cast<wxComboBox*>(panel->FindWindowByName("Diffusion Picker",panel.get()));
    wxComboBox* SteeringMethodComboBox = dynamic_cast<wxComboBox*>(panel->FindWindowByName("Steering Method",panel.get()));
    wxComboBox* DelegateComboBox = dynamic_cast<wxComboBox*>(panel->FindWindowByName("Delegates",panel.get()));
    wxComboBox* OptimizerComboBox = dynamic_cast<wxComboBox*>(panel->FindWindowByName("Path Optimizer",panel.get()));
    wxCheckBox* biDiffuseCheckBox =  dynamic_cast<wxCheckBox*>(panel->FindWindowByName("BiDiff Chk",panel.get()));
    wxCheckBox* ShowRdmCheckBox =  dynamic_cast<wxCheckBox*>(panel->FindWindowByName("H/S Rdm Chk",panel.get()));
    
    switch(RoadmapBuilderComboBox->GetCurrentSelection()){//to set the chosen roadmapBuilder
    case 0 : RdmBuilder = CkwsBasicRdmBuilder::create(CkwsRoadmap::create(Device),1.0 );
    panel->getInterface()->hppPlanner()->roadmapBuilderIthProblem(i,RdmBuilder);
    isDiffusing = false;
      break;
    case 1 : DiffusingRdmBuilder = CkwsDiffusingRdmBuilder::create(CkwsRoadmap::create(Device),1.0 );
    DiffusingRdmBuilder->diffuseFromProblemGoal(biDiffuseCheckBox->IsChecked());
    panel->getInterface()->hppPlanner()->roadmapBuilderIthProblem(i,DiffusingRdmBuilder);
      break;
    case 2 : DiffusingRdmBuilder = CkwsIPPRdmBuilder::create(CkwsRoadmap::create(Device),1.0 );
    DiffusingRdmBuilder->diffuseFromProblemGoal(biDiffuseCheckBox->IsChecked());
    panel->getInterface()->hppPlanner()->roadmapBuilderIthProblem(i,DiffusingRdmBuilder);
      break;
    default : 
      break;
    }

    if(isDiffusing){
      CkwsShooterConfigListShPtr configList = CkwsShooterConfigList::create();
      switch(ShooterComboBox->GetCurrentSelection()){//To set the chosen shooter
      case 0 : DiffusingRdmBuilder->diffusionShooter(CkwsShooterConfigSpace::create());
	break;
      case 1 : updateConfigList(configList); DiffusingRdmBuilder->diffusionShooter(configList);
	//TODO : open a panel, a tab or a window, to choose the number of configurations and the configurations
	break;
      case 2 : DiffusingRdmBuilder->diffusionShooter(CkwsShooterPath::create());
	break;
      case 3 : DiffusingRdmBuilder->diffusionShooter(CkwsShooterRoadmapBox::create());
	break;
      case 4 : DiffusingRdmBuilder->diffusionShooter(CkwsShooterRoadmapNodes::create());
	break;
      case 5 : DiffusingRdmBuilder->diffusionShooter(CkwsShooterMulti::create(updateShooterList()));
	break;
      case 6 : DiffusingRdmBuilder->diffusionShooter(CkwsShooterAdaptiveMulti::create(updateShooterList()));
	break;
      default: 
	break;
      }


      switch(PickerComboBox->GetCurrentSelection()){//To set the chosen picker
      case 0 : DiffusingRdmBuilder->diffusionNodePicker(CkwsPickerBasic::create());
	break;
      case 1 : DiffusingRdmBuilder->diffusionNodePicker(CkwsPickerSmallestTree::create());
	break;
      default: 
	break;
      }
    }

    switch(SteeringMethodComboBox->GetCurrentSelection()){//To set the chosen steering Method
    case 0 : Device->steeringMethod(CkwsSMLinear::create());
      break;
    default: 
      break;
    }

    switch(DelegateComboBox->GetCurrentSelection()){//To set the chosen delegate
    case 0 : if(isDiffusing) DiffusingRdmBuilder->addDelegate(new CkwsGraphicRoadmapDelegate("Default Delegate")) ; else RdmBuilder->addDelegate(new CkwsGraphicRoadmapDelegate("Default Delegate"));
      break;
    default: 
      break;
    }

    switch(OptimizerComboBox->GetCurrentSelection()){//To set the chosen optimizer
    case 0 : panel->getInterface()->hppPlanner()->pathOptimizerIthProblem(i,CkwsClearOptimizer::create());
      break;
    case 1 : panel->getInterface()->hppPlanner()->pathOptimizerIthProblem(i,CkwsAdaptiveShortcutOptimizer::create());
      break;
    case 2 : panel->getInterface()->hppPlanner()->pathOptimizerIthProblem(i,CkwsRandomOptimizer::create());
      break;
    default :
      break;
    }

    if(ShowRdmCheckBox->IsChecked()){
      CkwsGraphicRoadmapShPtr kwsGraphicRoadmap = CkwsGraphicRoadmap::create(panel->getInterface()->hppPlanner()->roadmapBuilderIthProblem(0),"default graphic roadmap") ;
      panel->getInterface()->addGraphicRoadmap(kwsGraphicRoadmap,true);
      cout<<"Showing Roadmap"<<endl;
    }

  }
  cout<<"RRT Configuration - Done"<<endl;
  panel->Hide();
  panel->getInterface()->hppPlanner()->solve() ;

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
}

/*_________________________________________________________*/
void CkppPlannerPanelController::ProblemSpinCtrlEventHandler(wxSpinEvent& cancel){
//TODO : read each parameter of the problem in order to fill controls with their value

  wxSpinCtrl * ProblemSpinCtrl = dynamic_cast<wxSpinCtrl*>(cancel.GetEventObject());

  ProblemSpinCtrl->SetRange(0,panel->getInterface()->hppPlanner()->getNbHppProblems());

  problemId = ProblemSpinCtrl->GetValue();

}

/*_________________________________________________________*/

void CkppPlannerPanelController::addConfigurations(){

  //finding existing configurations
  CkppModelTreeShPtr modelTree = CkppMainWindowController::getInstance()->document()->modelTree();

  wxWindow* magnetTab = panel->getNotebook()->GetPage(2);
  wxWindowList children = magnetTab->GetChildren();
  unsigned int nbTotalCheckBox = children.GetCount();

  CkppDeviceComponentShPtr robot = KIT_DYNAMIC_PTR_CAST(CkppDeviceComponent, getPanel()->getInterface()->hppPlanner()->robotIthProblem(problemId));

  for(int i=nbTotalCheckBox; i<robot->countConfigComponents();i++){
    
    wxString* label = new wxString("configuration : (");
    
    for(int j=0;j<robot->configComponent(i)->kwsConfig()->size();j++){//build label string for the checkbox
      
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

    for(int i=0; i<nbTotalCheckBox;i++){
      if(dynamic_cast<wxCheckBox*>(children.Item(i)->GetData())->IsChecked()){
	configList->add(*(robot->configComponent(i)->kwsConfig().get()),100.0,0.2);
	nbCheckedConf++;
      }
    }

    cout<<nbCheckedConf<<" configurations selected."<<endl;

  }
  else cout<<"ERROR : Number of available configurations is different from number of choice in the magnet tab."<<endl<<"        Try to reselect Magnet List as a shooter."<<endl;
  

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

    for(int i=0; i<nbTotalShooters;i++){
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

    cout<<nbCheckedShooters<<" shooters  selected."<<endl;

  return shooterList;

}


/*_________________________________________________________*/


/***********************************************************/
