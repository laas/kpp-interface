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

#ifndef KPP_CORBASERVERCOMMAND_H
# define KPP_CORBASERVERCOMMAND_H

# include "KineoController/kppCommand.h"

# include <hpp/util/kitelab.hh>

HPP_KIT_PREDEF_CLASS( CkppCommandStartCorbaServer );
HPP_KIT_PREDEF_CLASS( ChppDevice );
HPP_KIT_PREDEF_CLASS( ChppBox );
HPP_KIT_PREDEF_CLASS( CkppInterface );
HPP_KIT_PREDEF_CLASS( CkitNotification);

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
