/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

/*

   Exception class
   Copyright (C) 2001-2005 Jussi Laako

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 */


#include <exception>
#include <string>
#include <cstring>
#include <cstdio>
#include <cstddef>


#ifndef EXCEPTION_HH
#define EXCEPTION_HH


class clException : public std::exception
{
	int iCode;
	std::string strMsg;
	void AddErrorStr () throw()
	{
		/* in SuSE 9.2 glibc strerror_r() is somehow broken */
		/*char cpErrorStr[256];
		  cpErrorStr[0] = '\0';
		  strerror_r(iCode, cpErrorStr, 256);
		  strMsg = strMsg + std::string(": ") +
		  std::string(cpErrorStr);*/
		strMsg = strMsg + std::string(": ") +
			std::string(strerror(iCode));
	}
	public:
	clException () throw()
	{ }
	clException (int iErrorCode) throw()
	{
		char cpConvBuf[79];  /* Enough for 256-bit integers */
		iCode = iErrorCode;
		sprintf(cpConvBuf, "%i", iErrorCode);
		strMsg = std::string(cpConvBuf);
		AddErrorStr();
	}
	clException (const std::string &strErrorMsg) throw()
	{
		iCode = 0;
		strMsg = strErrorMsg;
	}
	clException (const std::string &strErrorMsg, int iErrorCode)
		throw()
		{
			iCode = iErrorCode;
			strMsg = strErrorMsg;
			AddErrorStr();
		}
	clException (const char *cpErrorMsg) throw()
	{
		iCode = 0;
		strMsg = std::string(cpErrorMsg);
	}
	clException (const char *cpErrorMsg, int iErrorCode) throw()
	{
		iCode = iErrorCode;
		strMsg = std::string(cpErrorMsg);
		AddErrorStr();
	}
	virtual ~clException () throw()
	{ }
	operator int () const throw()
	{
		return iCode;
	}
	operator std::string () const throw()
	{
		return strMsg;
	}
	virtual const char * what () const throw()
	{
		return strMsg.c_str();
	}
};


#endif
