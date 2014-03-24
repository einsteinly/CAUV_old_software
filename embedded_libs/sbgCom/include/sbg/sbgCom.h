/*!
 *	\file		sbgCom.h
 *  \author		SBG-Systems (Raphael Siryani)
 *	\date		18/07/07
 *
 *	\brief		Main header file for sbgCom library.<br>
 *				Copyright 2007-2008 SBG Systems. All rights reserved.
 */

/*!
 *	\mainpage sbgCom library documentation
 *	Welcome to the sbgCom library documentation.<br>
 *	This documentation describes all functions implemented in the sbgCom library.
 */

#ifndef __SBG_COM_H__
#define __SBG_COM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <sbg/sbgCommon.h>
#include <sbg/time/sbgTime.h>
#include <sbg/protocol/protocol.h>
#include <sbg/protocol/protocolOutputMode.h>
#include <sbg/protocol/protocolOutput.h>
#include <sbg/protocol/commands.h>
#include <sbg/protocol/commandsCalib.h>
#include <sbg/protocol/commandsFilter.h>
#include <sbg/protocol/commandsOrientation.h>
#include <sbg/protocol/commandsOutput.h>
#include <sbg/protocol/commandsNav.h>
#include <sbg/protocol/commandsGps.h>
#include <sbg/protocol/commandsExt.h>
#include <sbg/protocol/commandsSync.h>
#include <sbg/protocol/commandsOdo.h>
#include <sbg/protocol/extDevices/extIg.h>
#include <sbg/protocol/extDevices/extNmea.h>

//----------------------------------------------------------------------//
//- sbgCom library main operations                                     -//
//----------------------------------------------------------------------//

/*!
 *	Initialize our sbgCom library.<br>
 *	Open our COM port and try to get the default output mask and output mode from the device.<br>
 *	\param[in]	deviceName						Communication port to open ("COM1" on Windows, "/dev/ttysX" on UNIX platforms).
 *	\param[in]	baudRate						Baud rate used to communicate with the device.<br>
 *												Possible values are:<br>
 *												- 9600<br>
 *												- 19200<br>
 *												- 38400<br>
 *												- 57600<br>
 *												- 115200<br>
 *												- 230400<br>
 *												- 460800<br>
  *												- 921600<br>
 *	\param[out]	pHandle							Used to returns the created SbgProtocolHandle struct.
 *	\return										SBG_NO_ERROR if a connection with the device has been established.
 */
SbgErrorCode sbgComInit(const char *deviceName, uint32 baudRate, SbgProtocolHandle *pHandle);

/*!
 *	Close our COM port and release memory for our sbgCom library.
 *	\param[in]	handle							Our sbgCom library handle to release.
 *	\return										SBG_NO_ERROR if we have released our handle.
 */
SbgErrorCode sbgComClose(SbgProtocolHandle handle);

/*!
 *	Returns an integer representing the version of our sbgCom library.
 *	\return										An integer representing the version of our sbgCom library.<br>
 *												Use #SBG_VERSION_GET_MAJOR, #SBG_VERSION_GET_MINOR, #SBG_VERSION_GET_REV and #SBG_VERSION_GET_BUILD.
 */
uint32 sbgComGetVersion(void);

/*!
 *	Retreive our sbgCom library version as a string (1.0.0.0).
 *	\param[out]	version							String buffer used to hold our sbgCom library version.
 */
void sbgComGetVersionAsString(char version[32]);

/*!
 *	Convert an error code into a human readable string.
 *	\param[in]	errorCode						Our errorCode to convert into a string.
 *	\param[out]	errorMsg						String buffer used to hold our error string.
 */
void sbgComErrorToString(SbgErrorCode errorCode, char errorMsg[256]);

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif	// __SBG_COM_H__
