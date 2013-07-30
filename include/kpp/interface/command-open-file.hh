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

#ifndef kppCommandOpenFile_H
#define kppCommandOpenFile_H

/**
  \brief Main class of builder kppCommandOpenFile
 */

#include "kpp/interface/interface.hh"

KIT_PREDEF_CLASS(CkppCommandOpenFile);
class CkppCommandOpenFile : public CkppCommand{
 public:
   enum EParameters 
     { 
       FILE_PATH = 0,       
       PARAMETER_COUNT 
     }; 
  /**
     \brief Create Method
     \return A Shared Pointer to the newly created command
   */
  static CkppCommandOpenFileShPtr   create(CkppInterface * inInterface);
  /**
     \brief Copy creator. Uses a previously defined CreateBuilderIPP Command to create a new one
     \param inCommand The previous command.
     \return  A Shared Pointer to the newly created command
   */
  static CkppCommandOpenFileShPtr   createCopy(const CkppCommandOpenFileConstShPtr& inCommand);
  /**
     \brief Destructor
   */
  virtual ~CkppCommandOpenFile();
  /**
     \brief : What function does.
  */
  virtual ktStatus doExecute();
   /**
      \brief Says if the command can be undone. 
      \return true if it can be  undone, false in other cases. 
    */ 
   virtual bool	isUndoable() const; 
   virtual CkppCommandShPtr clone() const; 
   virtual unsigned int	countParameters() const; 
   /** 
      \brief Retrieves a parameter defined in EParameters (enum in the header file) from the model tree.
      \param inRank Rank in the enum structure of the parameter to retrieve. 
    */ 
   virtual CkppParameterConstShPtr	parameter(unsigned int inRank) const; 
protected:
  /**
     \brief Constructor
   */
  CkppCommandOpenFile(CkppInterface * inInterface);
  /**
     \brief Copy Constructor
     \param inCommand The previous command.
   */
  CkppCommandOpenFile(const CkppCommandOpenFile& inCommand);
  /**
     \brief Initialisation method.
     \param inWeakPtr Pointer on the object itself.
   */
  ktStatus init(const CkppCommandOpenFileWkPtr& inWeakPtr);
private:
  CkppCommandOpenFileWkPtr attWeakPtr;
  CkppInterface * attInterface;
};

#endif
