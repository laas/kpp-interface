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
