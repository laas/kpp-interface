/*
  Copyright 2008 CNRS-LAAS

  Authors: David Flavigne and Florent Lamiraux

*/

#ifndef KWS_GRAPHIC_ROADMAP_H_
#define KWS_GRAPHIC_ROADMAP_H_

/*------------------------------------*/
/*              INCLUDES              */
/*------------------------------------*/

#include "KineoView/kppViewGraphic.h"
#include "KineoView/kppViewGeneral.h"
#include "KineoView/kppViewWindow.h"
#include "KineoView/kppViewCanvas.h"
#include "KineoGUI/kppMainWindowController.h"
#include "KineoGUI/kppGraphicWindowController.h"
#include "KineoWorks2/kwsRoadmap.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoWorks2/kwsEdge.h"
#include "KineoWorks2/kwsDevice.h"
#include "KineoUtility/kitNotification.h"
#include "KineoUtility/kitNotificator.h"
#include "KineoController/kppPlanPathCommand.h"
#include "KineoController/kppDocument.h"
#include "KineoModel/kppModelTree.h"
#include "KineoModel/kppDeviceComponent.h"
#include "KineoModel/kppDeviceNode.h"
#include "KineoModel/kppPathComponent.h"
#include "KineoModel/kppPathNode.h"

#include "kppInterface/kwsGraphicRoadmapDelegate.h"

KIT_PREDEF_CLASS(CkwsGraphicRoadmap);
KIT_PREDEF_CLASS(CkppInterface);

/** 
    \addtogroup kppInterface_graphic_Roadmap

    @{
*/

/**

This class allows users to display their roadmaps \c CkwsRoadmap. It displays 
all the joints with displayPath property set to true.
    In order to do so, you just have to create an instance of CkwsGraphicRoadmap
    giving a \c CkwsRoadmapBuilder, then add it to your kppInterface object:

    \code

    bool is_Graphic_Roadmap_Updated_At_RunTime = false;
    CkwsGraphicRoadmapShPtr myGraphicRoadmap = CkwsGraphicRoadmap::create(anyCkwsRoadmapBuilder);
    myKppInterface->addGraphicRoadmap(myGraphicRoadmap, is_Graphic_Roadmap_Updated_At_RunTime);

    \endcode

    Then you can call methods \c showRoadmap(rank) and \c hideRoadmap(rank) of \c CkppInterface to show/hide your roadmap.
    The method \c removeGraphicRoadmap(rank) allows to erase definitively a roadmap, and if rank is -1, it erase all
    the roadmaps of the instance of \c CkppInterface.

    However, if you want your roadmap to be updated at run-time, you need to verify three things :
    - A \c CkppProgressDelegate has been set for the used roadmap builder (see class CkitProgressDelegate in KineoWorks documentation for more details). This class is already provided in kppInterface.
    It allows the graphic window to be refreshed at run-time
    - The CkppPlanPathCommand::DID_ADD_EDGE_TO_ROADMAP notification has been sent with a CkwsRoadmapBuilder object.
    You should send it by yourself when needed, since default behaviour is to send the notification with a CkppCommand object. See code below for an example.
    - The graphic roadmap has to be created and added before solving the associated problem.

    \code

    void CmyRoadmapBuilderDelegate::didAddEdge (const CkwsRoadmapBuilderConstShPtr &inRdmBuilder, 
                                                const CkwsEdgeConstShPtr &inEdge)
    {

         CkitNotificationShPtr notification = 
           CkitNotification::createWithShPtr<CkwsRoadmapBuilder>(CkppPlanPathCommand::DID_ADD_EDGE_TO_ROADMAP, 
	   m_builder);
         CkitNotificator::defaultNotificator()->notify(notification);

         CkwsRdmBuilderDelegate::didAddEdge (inRdmBuilder, inEdge);

    }

    \endcode

    Here, the class \c CmyRoadmapBuilderDelegate inherits from both CkwsRoadmapBuilderDelegate and 
    CkppProgressDelegate.

    Graphical roadmaps can also be set visible or not in the \link #CkppPlannerPanel Planner Configuration 
    Panel \endlink , and a delegate can be added as well.
*/

class CkwsGraphicRoadmap : public CkppViewGraphic {

 public:
  /**
     \brief Get name
  */
  inline const std::string& name() { return attName;};
  /**
     \brief Rendering method for the roadmap : this function draws segments from the roadmap vertices. It is automatically called by KPP.
   */
  virtual void 	render();

  /**
     \brief Create method.
     \param inRoadmapBuilder the roadmap builder owning the roadmap to display.
     \param inName Name of the graphic roadmap.
     \param inKppInterface kppInterface owning this object
   */
  static CkwsGraphicRoadmapShPtr create(const CkwsRoadmapBuilderShPtr & inRoadmapBuilder,
					const CkppInterfaceWkPtr& inKppInterface,
					const std::string &inName = "");

  /**
     \brief Destructor
  */
  virtual ~CkwsGraphicRoadmap();

  /**
     \brief Allows user to tell if the roadmap will be updated at run time or displayed at end of
     building.
     \param rtu set it true if you want the roadmap to be real time updated
   */
  void SetRealTimeUpdate(bool rtu);

  /**
     \brief Says if the roadmap is updated at run-time or displayed at end of building
   */
  bool GetRealTimeUpdate();

  /**
     \brief returns the kwsRoadmap
   */
  CkwsRoadmapShPtr kwsRoadmap() {
    return attKwsRoadmap.lock();
  }

  /**
     \brief Change the graphic roadmap status (displayed or not)
     \param disp Set this to true if you want the roadmap to be displayed
   */
  void isDisplayed(bool disp) {
    attIsDisplayed = disp;
  }
  /**
     \brief Returns the graphic roadmap status (displayed or not)
   */
  bool isDisplayed() {
    return attIsDisplayed;
  }

  /**
     \brief draws the entire roadmap when the end of building is notified
   */
  void drawNotifRoadmap(const CkitNotificationConstShPtr& inNotification);

 protected:
  /**
     \brief Constructor
  */
  CkwsGraphicRoadmap(const std::string& inName);

  /**
     \brief initialization method

     \param inWkPtr Weak pointer to itself
     \param inKppInterface kppInterface owning this object
     \param inRoadmapBuilder the roadmap builder owning the roadmap to display.
   */
  ktStatus init(const CkwsGraphicRoadmapWkPtr& inWkPtr,
		const CkppInterfaceWkPtr& inKppInterface,
		const CkwsRoadmapBuilderShPtr &inRoadmapBuilder);

  /**
     \brief draws the entire roadmap
   */
  void drawRoadmap();

 private:

  /**
     \brief Weak pointer to itself
  */
  CkwsGraphicRoadmapWkPtr attWeakPtr;
  
  /**
     \brief Whether the display is updated at run time.
  */
  bool isRealTimeUpdated;

  /**
     \brief Whether the roadmap builder has finished the roadmap construction.
  */
  bool attFinished;
  
  /**
     \brief Whether the roadmap shoudl be displayed
  */
  bool attIsDisplayed;

  /**
     \brief The roadmap that is displayed
  */
  CkwsRoadmapWkPtr attKwsRoadmap;

  /**
     \brief name
  */
  std::string attName;

  /**
     \brief Weak pointer to kppInterface owning this object
  */
  CkppInterfaceWkPtr attKppInterface;

  /**
     \if 0
     \brief Counter of objects of this class
  */
  static unsigned int nbObjects;

  /**
     \endif
  */
};

/**
   @}
*/
#endif /*KWS_GRAPHIC_ROADMAP_H*/
