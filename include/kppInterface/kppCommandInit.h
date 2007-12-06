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

class CkppCommandInit : public CkppCommand
{

public:

  enum EParameters
    {
      MODELTREE = 0,
      PATH,
      DEVICE,
      PARAMETER_COUNT
    };

  static CkppCommandInitShPtr   create(CkppInterface *kpp);

  static CkppCommandInitShPtr   createCopy(const CkppCommandInitConstShPtr& i_command);

  virtual ~CkppCommandInit();

  /**
     \brief : What function does.
  */
  virtual ktStatus doExecute();

  virtual bool	isUndoable() const;

  virtual CkppCommandShPtr clone() const;

  virtual unsigned int	countParameters() const;

  virtual CkppParameterConstShPtr	parameter(unsigned int i_rank) const;

protected:

  CkppCommandInit(CkppInterface *kpp);

  CkppCommandInit(const CkppCommandInit& i_command);

  ktStatus init(const CkppCommandInitWkPtr& i_weakPtr);

private:

  CkppCommandInitWkPtr m_weakPtr;

  /// pointer to the interface
  CkppInterface *attKpp;

};

#endif
