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

class CkppCommandSetConfig : public CkppCommand
{

public:

	enum EParameters
	{ 
	  PATH = 0,
	  PARAMETER_COUNT
	};
	  

	static CkppCommandSetConfigShPtr   create(CkppInterface *kpp);

	static CkppCommandSetConfigShPtr   createCopy(const CkppCommandSetConfigConstShPtr& i_command);

	virtual ~CkppCommandSetConfig();

	virtual ktStatus doExecute();

	virtual bool	isUndoable() const;

	virtual CkppCommandShPtr clone() const;

	virtual unsigned int	countParameters() const;

	virtual CkppParameterConstShPtr	parameter(unsigned int i_rank) const;

protected:

	CkppCommandSetConfig(CkppInterface *kpp);

	CkppCommandSetConfig(const CkppCommandSetConfig& i_command);

	ktStatus init(const CkppCommandSetConfigWkPtr& i_weakPtr);

private:

	CkppCommandSetConfigWkPtr m_weakPtr;

	CkppInterface *attKpp;

};

#endif