// COPYRIGHT KINEOCAM 2007
#ifndef KPP_CORBASERVERCOMMAND_H
#define KPP_CORBASERVERCOMMAND_H

#include <KineoController/kppCommand.h>


KIT_PREDEF_CLASS( CkppCommandStartCorbaServer );
KIT_PREDEF_CLASS( ChppDevice );
KIT_PREDEF_CLASS( ChppBox );
KIT_PREDEF_CLASS( CkppInterface );

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

	static CkppCommandStartCorbaServerShPtr createCopy(const CkppCommandStartCorbaServerConstShPtr& inCommand);

	virtual ~CkppCommandStartCorbaServer();

	virtual ktStatus											doExecute();

	virtual bool													isUndoable() const;

	virtual CkppCommandShPtr							clone() const;

	virtual unsigned int									countParameters() const;

	virtual CkppParameterConstShPtr				parameter(unsigned int inRank) const;

	  std::string commandStr() {return attCommandStr;};
	  void commandStr(const std::string &i_str) {attCommandStr = i_str;};


protected:

	// think about taking hppPlanner as argument
	CkppCommandStartCorbaServer(CkppInterface *kpp);

	CkppCommandStartCorbaServer(const CkppCommandStartCorbaServer& inCommand);

	ktStatus init(const CkppCommandStartCorbaServerWkPtr& inWeakPtr);


private:

	CkppCommandStartCorbaServerWkPtr		attWeakPtr;

	CkppInterface *attKpp;
	std::string attCommandStr;

};

#endif //HPPKPP_CORBASERVERCOMMAND_H
