/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavign\'e

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

/** This class allows users to display their roadmaps \c CkwsRoadmap. It displays all the joint configurations, for joints that are their displayPath property set to true.
    In order to do so, you just have to create an instance of CkwsGraphicRoadmap
    giving a \c CkwsRoadmap, then add it to your kppInterface-derived class :

    \code

    bool is_Graphic_Roadmap_Updated_At_RunTime = false;
    CkwsGraphicRoadmapShPtr myGraphicRoadmap = CkwsGraphicRoadmap::create(anyCkwsRoadmap);
    myKppInterface->addGraphicRoadmap(myGraphicRoadmap, is_Graphic_Roadmap_Updated_At_RunTime);

    \endcode

    Then you can call methods \c showRoadmap(rank) and \c hideRoadmap(rank) of \c CkppInterface to show/hide your roadmap.
    The method \c removeGraphicRoadmap(rank) allows to erase definitively a roadmap, and if rank is -1, it erase all
    the roadmaps of the instance of \c CkppInterface.

    However, if you want your roadmap to be updated at run-time, you need to verify three things :
    - A \c CkppProgressDelegate has been set for the used roadmap builder (see class \link #CkitProgressDelegate CkitProgressDelegate\endlink for more details). This class is already provided in kppInterface.
    It allows the graphic window to be refreshed at run-time
    - The \link #CkppPlanPathCommand CkppPlanPathCommand\endlink ::DID_ADD_EDGE_TO_ROADMAP notification has been sent with a \link #CkwsRoadmapBuilder CkwsRoadmapBuilder\endlink object.
    You should send it by yourself when needed, since default behaviour is to send the notification with a CkppCommand object. See code below for an example.
    - The graphic roadmap has to be created and added before solving the associated problem.

    \code

    void CmyRoadmapBuilderDelegate::didAddEdge (const CkwsRoadmapBuilderConstShPtr &i_builder, const CkwsEdgeConstShPtr &i_edge){

         CkitNotificationShPtr notification = CkitNotification::createWithShPtr<CkwsRoadmapBuilder>(CkppPlanPathCommand::DID_ADD_EDGE_TO_ROADMAP, m_builder);
         CkitNotificator::defaultNotificator()->notify(notification);

         CkwsRdmBuilderDelegate::didAddEdge (i_builder,i_edge);

    }

    \endcode

    Here, the class \c CmyRoadmapBuilderDelegate inherits from both CkwsRoadmapBuilderDelegate and CkppProgressDelegate.
    

*/

class CkwsGraphicRoadmap : public CkppViewGraphic {

 public:

  /**
     \brief Rendering method for the roadmap : this function draws segments from the roadmap vertices. It is automatically called by KPP.
   */
  virtual void 	render();

  /**
     \brief Create method.
   */
  static CkwsGraphicRoadmapShPtr create(const CkwsRoadmapBuilderShPtr & i_roadmap,const std::string &inName = "");

  /**
     \brief Destructor
  */
  virtual ~CkwsGraphicRoadmap();

  /**
     \brief Allows user to tell if the roadmap will be updated at run time or displayed at end of
     building.
   */
  void SetRealTimeUpdate(bool rtu);

  /**
     \brief Says if the roadmap is updated at run-time or displayed at end of building
   */
  bool GetRealTimeUpdate();

  /**
     \brief returns the kwsRoadmap
   */
  CkwsRoadmapShPtr kwsRoadmap(){return m_kwsRoadmap;}

  void isDisplayed(bool disp){m_isDisplayed = disp;}
  bool isDisplayed(){return m_isDisplayed;}

  void isJointDisplayed(bool disp){m_isJointDisplayed = disp;}
  bool isJointDisplayed(){return m_isJointDisplayed;}

  /**
     \brief draws the entire roadmap when the end of building is notified
   */
  void drawNotifRoadmap(const CkitNotificationConstShPtr& i_notification);

 protected:
  /**
     \brief Constructor
  */
  CkwsGraphicRoadmap();

  /**
     \brief initialization method
   */
  ktStatus init(const CkwsGraphicRoadmapWkPtr& i_ptr,const CkwsRoadmapBuilderShPtr &i_roadmap);

  /**
     \brief draws the entire roadmap
   */
  void drawRoadmap();

 private:

  CkwsGraphicRoadmapWkPtr m_weakPtr;
  bool isRealTimeUpdated;
  bool m_isJointDisplayed;
  bool finished;
  bool m_isDisplayed;
  CkwsRoadmapShPtr m_kwsRoadmap;


};

#endif /*KWS_GRAPHIC_ROADMAP_H*/
