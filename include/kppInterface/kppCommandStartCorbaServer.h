// COPYRIGHT KINEOCAM 2007
#ifndef KPP_CORBASERVERCOMMAND_H
#define KPP_CORBASERVERCOMMAND_H

#include "KineoController/kppCommand.h"

KIT_PREDEF_CLASS( CkppCommandStartCorbaServer );
KIT_PREDEF_CLASS( ChppDevice );
KIT_PREDEF_CLASS( ChppBox );
KIT_PREDEF_CLASS( CkppInterface );

class ChppPlanner;

KIT_PREDEF_CLASS( CkitNotification);

class CkppCommandStartCorbaServer : public CkppCommand
{

public:

	enum EParameters
	{
		MODEL_TREE = 0,
		PARAMETER_COUNT
	};

	static CkppCommandStartCorbaServerShPtr create(CkppInterface *kpp);

	static CkppCommandStartCorbaServerShPtr createCopy(const CkppCommandStartCorbaServerConstShPtr& i_command);

	virtual ~CkppCommandStartCorbaServer();

	virtual ktStatus											doExecute();

	virtual bool													isUndoable() const;

	virtual CkppCommandShPtr							clone() const;

	virtual unsigned int									countParameters() const;

	virtual CkppParameterConstShPtr				parameter(unsigned int i_rank) const;

	  std::string commandStr() {return attCommandStr;};
	  void commandStr(const std::string &i_str) {attCommandStr = i_str;};


protected:

	// think about taking hppPlanner as argument
	CkppCommandStartCorbaServer(CkppInterface *kpp);

	CkppCommandStartCorbaServer(const CkppCommandStartCorbaServer& i_command);

	ktStatus init(const CkppCommandStartCorbaServerWkPtr& i_weakPtr);


private:

	CkppCommandStartCorbaServerWkPtr		m_weakPtr;

	CkppInterface *attKpp;
	std::string attCommandStr;

};

#endif //HPPKPP_CORBASERVERCOMMAND_H
