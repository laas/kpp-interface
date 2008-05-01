// COPYRIGHT 



#ifndef KPP_COMMAND_SOLVE_PLANNER_H
#define KPP_COMMAND_SOLVE_PLANENR_H

/*****************************************
 INCLUDES
*******************************************/

#include <iostream>

#include "kppInterface/kppInterface.h"

#include "KineoController/kppCommand.h"

KIT_PREDEF_CLASS( CkppCommandSolvePlanner );


/*****************************************
 CLASS
*******************************************/

class CkppCommandSolvePlanner : public CkppCommand
{

public:

	enum EParameters
	{
		PARAMETER_COUNT
	};

	static CkppCommandSolvePlannerShPtr   create(CkppInterface *kpp);

	static CkppCommandSolvePlannerShPtr   createCopy(const CkppCommandSolvePlannerConstShPtr& inCommand);

	virtual ~CkppCommandSolvePlanner();

	virtual ktStatus doExecute();

	virtual bool	isUndoable() const;

	virtual CkppCommandShPtr clone() const;

	virtual unsigned int	countParameters() const;

	virtual CkppParameterConstShPtr	parameter(unsigned int inRank) const;

protected:

	CkppCommandSolvePlanner(CkppInterface *kpp);

	CkppCommandSolvePlanner(const CkppCommandSolvePlanner& inCommand);

	ktStatus init(const CkppCommandSolvePlannerWkPtr& inWeakPtr);

private:

	CkppCommandSolvePlannerWkPtr attWeakPtr;

	CkppInterface *attKpp;

};

#endif
