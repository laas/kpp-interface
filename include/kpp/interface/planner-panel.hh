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

#ifndef KPP_PLANNER_PANEL_H
#define KPP_PLANNER_PANEL_H


/**************************INCLUDES*************************/

#include "KineoGUI/kppPanel.h"
#include "KineoGUI/kppWindowController.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoWorks2/kwsBasicRdmBuilder.h"
#include "KineoWorks2/kwsDiffusingRdmBuilder.h"
#include "KineoWorks2/kwsIPPRdmBuilder.h"
#include "KineoWorks2/kwsClearOptimizer.h"
#include "KineoWorks2/kwsAdaptiveShortcutOptimizer.h"
#include "KineoWorks2/kwsRandomOptimizer.h"

#include "wx/sizer.h"
#include "wx/bmpcbox.h"
#include "wx/checkbox.h"
#include "wx/button.h"
#include "wx/arrstr.h"
#include "wx/window.h"
#include "wx/notebook.h"
#include "wx/event.h"

class CkppInterface; //references croisees

#define PROBLEM_SPIN_CTRL wxID_LOWEST-1
#define CS_SPIN_CTRL wxID_LOWEST-2
#define CL_SPIN_CTRL wxID_LOWEST-3
#define P_SPIN_CTRL wxID_LOWEST-4
#define RB_SPIN_CTRL wxID_LOWEST-5
#define RN_SPIN_CTRL wxID_LOWEST-6
#define RDM_BUILDER_COMBO_BOX wxID_LOWEST-7
#define SHOOTER_COMBO_BOX wxID_LOWEST-8
#define PICKER_COMBO_BOX wxID_LOWEST-9
#define STEERING_COMBO_BOX wxID_LOWEST-10
#define OPTIMIZER_COMBO_BOX wxID_LOWEST-11
#define DELEGATE_COMBO_BOX wxID_LOWEST-12
#define RDM_CHECK_BOX wxID_LOWEST-13
#define SOLVE_ALL_CHECK_BOX wxID_LOWEST-14
#define BIDIFF_CHECK_BOX wxID_LOWEST-15
#define MAGNET_CHECK_BOX wxID_LOWEST-16
#define CS_SHOOTER_CHECK_BOX wxID_LOWEST-17
#define CL_SHOOTER_CHECK_BOX wxID_LOWEST-18
#define P_SHOOTER_CHECK_BOX wxID_LOWEST-19
#define RB_SHOOTER_CHECK_BOX wxID_LOWEST-20
#define RN_SHOOTER_CHECK_BOX wxID_LOWEST-21
#define START_BUTTON wxID_LOWEST-22
#define SAVE_BUTTON wxID_LOWEST-23
#define OPEN_BUTTON wxID_LOWEST-24
#define ADD_PB_BUTTON wxID_LOWEST-25
#define CHOICES_NOTEBOOK wxID_LOWEST-26


KIT_PREDEF_CLASS( CkppPlannerPanel );
KIT_PREDEF_CLASS( CkppPlannerPanelController );

/***********************************************************/

/*****************************CLASS*************************/

/** 
   \addtogroup Panel
   @{
   \Section Panel_P Planner Configuration Panel
   This class is aimed to create graphical elements in the kpp interface, for displaying a Panel at the right of the window.
   If you want to add elements, first get a look to the wxWidgets homepage at http://wiki.wxwidgets.org/docbrowse.html 
    

    To define behavior of your elements, please refer to the \link #CkppPlannerPanelController CkppPlannerPanelController\endlink class. If you only want
    to add choices in the combo boxes there are lot of things to do : 
    - For each combo box, there is a String Array and a "StringArray" to modify:
    - Increment size of the String Array in the header file.
    - Same for the StringArray in the "build()" function in the source file.
    - Add Corresponding Lines in the constructor function (see source file).

    - In the CkppPlannerPanelController class :
    - you must create the corresponding class in the "StartButtonEventHandler", in the appropriate case in the desired "switch" structure (There is one "switch" for each combo box).
    - if needed, you can define action to perform when your choice is selected in the corresponding function (The ones that are suffixed "ComboBoxEventHandler")
    @}
*/

class CkppPlannerPanel : public CkppPanel
{
public:
  /**
     \brief enumerator of Roadmap Builders
  */
  typedef enum {
    BASIC_RDMBUILDER=0,
    DIFFUSING,
    IPP,
    VISIBILITY,
    PCA,
    LOCAL_TREES,
    NB_RDMBUILDER
  } Erdmbuilder;

  /**
     \brief enumerator of Shooters
  */
  typedef enum {
    CONFIG_PATH=0,
    CONFIG_LIST,
    PATHS,
    ROADMAP_BOX,
    ROADMAP_NODE,
    MULTI_SHHOTER,
    ADAPTIVE_MULTI,
    NB_SHOOTER
  } Eshooter;

  /**
     \brief enumerator of Pickers
  */
  typedef enum {
    BASIC_PICKER=0,
    SMALLEST_TREE,
    NB_PICKERS
  } Epicker ;

  /**
     \brief enumerator of SteeringMethods
  */
  typedef enum {
    LINEAR=0,
    FLIC,
    RS,
    SLERP,
    NB_STEERINGMETHODS
  } Esteeringmethod;

  /**
     \brief enumerator of RdmBuilderDelegates
  */
  typedef enum {
    GRAPHIC=0,
    NB_RDMBUILDERDELEGATE
  } Erdmbuilderdelegate;

  /**
     \brief enumerator of PathOptimizers
  */
  typedef enum {
    CLEAR=0,
    RANDOM,
    ADAPTIVE_SHORTCUT,
    NB_PATHOPTIMIZER
  } Epathoptimizer;

private:
  
  //associated rdmBuilder
  CkwsRoadmapBuilderShPtr rdm;
  ///Interface associated with the panel
  CkppInterface * attKpp;
  bool isRdmSet;
  bool isInterfaceSet;
  
  wxPanel* pages;
  wxNotebook* notebook;

  /*for comboBoxes -> initialized in the constructor*/
  wxString attPicker[NB_PICKERS];
  wxString attbuilders[NB_RDMBUILDER];
  wxString attShooters[NB_SHOOTER];
  wxString attSteeringMethods[NB_STEERINGMETHODS];
  wxString attRdmBuilderDelegates[NB_RDMBUILDERDELEGATE];
  wxString attPathOptimizers[NB_PATHOPTIMIZER];

  const wxString wxDefaultString;///Empty String
  
 public:
  /** \brief Destructor 
   */
  virtual ~CkppPlannerPanel();

  /** \brief Constructor 
   */
  CkppPlannerPanel(wxWindow *i_parent, const CkppPlannerPanelControllerShPtr &i_controller);

  /**
     \brief returns the associated rdmBuilder
   */
  CkwsRoadmapBuilderShPtr getRdmBuilder(){return rdm;}
  
  /**
     \brief sets the associated rdm
   */
  void setRdmBuilder(const CkwsRoadmapBuilderShPtr& i_rdm){rdm = i_rdm; isRdmSet = true;}

  /**
     \brief returns the associated interface
   */
  CkppInterface* getInterface(){return attKpp;}
  
  /**
     \brief sets the associated RRT
   */
  void setInterface(CkppInterface* i_interface){attKpp = i_interface; isInterfaceSet = true;}

  /**
     \brief Tells whether the rrt attribute has been set or not.
   */
  bool rdmIsSet(){return isRdmSet;}

  /**
     \brief Tells whether the rrt attribute has been set or not.
   */
  bool interfaceIsSet(){return isInterfaceSet;}

  /**
     \brief returns the current controller
   */
  CkppPlannerPanelControllerShPtr commandPanelController() const;
    
  /**
     \brief returns the main panel
   */
  wxPanel* getMainPage(){return pages;}

  /**
     \brief returns the main panel's notebook
   */
  wxNotebook* getNotebook(){return notebook;}

  /**
     \brief allow users to dynamically add controls
   */
  void addControl(wxWindow* window, wxObject* object);

  /**
     \name inherited methods
     @{
  */
  /**
     \brief inherited from CkppPanel. Adding controls and subwindows in the panel.
  */
 virtual void	build (wxBoxSizer *i_sizer);
  /**
     @}
  */

};

/***********************************************************/

#endif
