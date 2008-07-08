// COPYRIGHT 

/*
 *
 *
 *
 *   EXEMPLE FILE TO CREATE COMMAND BASE : DO NOT WRITE IN
 *
 *   
 *
 *
 */

#ifndef KPP_COMMAND_INIT_H
#define KPP_COMMAND_INIT_H

/*****************************************
 INCLUDES
*******************************************/

#include <iostream>

#include "kppInterface/kppInterface.h"

#include "KineoController/kppCommand.h"

KIT_PREDEF_CLASS( CkppCommandInit );


/*****************************************
 CLASS
*******************************************/
/**
   \addtogroup Panel
   @{
   \section Command_Init Initialize Problem
   This class is a default implementation for initializing a hpp problem. Actually, it only retrieves a device component from the model tree and give it for initialization to the planner
   @}
*/
class CkppCommandInit : public CkppCommand
{

public:

  enum EParameters
    {
      DEVICE = 0,
      PARAMETER_COUNT
    };

  static CkppCommandInitShPtr   create(CkppInterface *kpp);

  static CkppCommandInitShPtr   createCopy(const CkppCommandInitConstShPtr& inCommand);

  virtual ~CkppCommandInit();

  /**
     \brief : What function does.
  */
  virtual ktStatus doExecute();

  virtual bool	isUndoable() const;

  virtual CkppCommandShPtr clone() const;

  virtual unsigned int	countParameters() const;

  virtual CkppParameterConstShPtr	parameter(unsigned int inRank) const;

protected:

  CkppCommandInit(CkppInterface *kpp);

  CkppCommandInit(const CkppCommandInit& inCommand);

  ktStatus init(const CkppCommandInitWkPtr& inWeakPtr);

private:

  CkppCommandInitWkPtr attWeakPtr;

  /// pointer to the interface
  CkppInterface *attKpp;

};

#endif
