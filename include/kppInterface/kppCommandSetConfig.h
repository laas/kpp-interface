// COPYRIGHT 



#ifndef KPP_COMMAND_SET_CONFIG_H
#define KPP_COMMAND_SET_CONFIG_H

/*****************************************
 INCLUDES
*******************************************/

#include <iostream>

#include "kppInterface/kppInterface.h"

#include "KineoController/kppCommand.h"

KIT_PREDEF_CLASS( CkppCommandSetConfig );


/*****************************************
 CLASS
*******************************************/
/**
   \addtogroup Panel
   @{
   \section Command_Set Set Configurations
   This class is a default implementation for initializing a path for a given problem. It takes start and goal configurations from a selected path, and give them to the planner.
   @}
*/
class CkppCommandSetConfig : public CkppCommand
{

public:

	enum EParameters
	{ 
	  PATH = 0,
	  PARAMETER_COUNT
	};
	  

	static CkppCommandSetConfigShPtr   create(CkppInterface *kpp);

	static CkppCommandSetConfigShPtr   createCopy(const CkppCommandSetConfigConstShPtr& inCommand);

	virtual ~CkppCommandSetConfig();

	virtual ktStatus doExecute();

	virtual bool	isUndoable() const;

	virtual CkppCommandShPtr clone() const;

	virtual unsigned int	countParameters() const;

	virtual CkppParameterConstShPtr	parameter(unsigned int inRank) const;

protected:

	CkppCommandSetConfig(CkppInterface *kpp);

	CkppCommandSetConfig(const CkppCommandSetConfig& inCommand);

	ktStatus init(const CkppCommandSetConfigWkPtr& inWeakPtr);

private:

	CkppCommandSetConfigWkPtr attWeakPtr;

	CkppInterface *attKpp;

};

#endif
