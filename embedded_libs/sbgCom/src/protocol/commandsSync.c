#include <sbg/protocol/protocolOutputMode.h>
#include <sbg/protocol/commandsSync.h>
#include <sbg/protocol/commands.h>

//----------------------------------------------------------------------//
// Logic inputs operations                                              //
//----------------------------------------------------------------------//

/*!
 *	Set a logic input channel setting
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	permanent			Store this setting permanently (TRUE) or temporary (FALSE)
 *	\param[in]	channel				Channel to configure. May Be 0 or 1
 *	\param[in]	inputType			Type of the logic input, may be <br>
 * 										- SBG_IN_DISABLED
 *										- SBG_IN_EVENT
 * 										- SBG_IN_MAIN_LOOP_START
 *										- SBG_IN_TIME_PULSE
 * 										- SBG_IN_ODOMETER
 * \param[in]	sensitivity			Sensitivity of the trigger. It may be:<br>
 * 										- SBG_IN_FALLING_EDGE
 * 										- SBG_IN_RISING_EDGE
 * 										- SBG_IN_LEVEL_CHANGE
 * \param[in]	location			Physical location of the input pin for the syncIN channel 0 on IG-500E. <br>
 *										- SBG_IN_STD_LOCATION (default value, leave to this value if not used)
 *										- SBG_IN_EXT_LOCATION
 * \param[in]	nsDelay				Delay to be added before the actual trigger is taken into account (in nano seconds) delays up to 2seconds are allowed:<br>
 *									This delay is only used when the input is set to:
 * 										- SBG_IN_EVENT
 * 										- SBG_IN_MAIN_LOOP_START
 *										- SBG_IN_TIME_PULSE <br>
 * 									When used with the time pulse event, this delay can be negative, in order to simulate the time pulse propagation time
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgSetLogicInChannel(SbgProtocolHandle handle, bool permanent, uint8 channel, SbgLogicInType inputType, SbgLogicInSensitivity sensitivity, SbgLogicInLocation location, int32 nsDelay)
{
	uint8 buffer[4*sizeof(uint8)+sizeof(int32)];
	SbgErrorCode error;

	//
	// Build our params buffer
	//
	buffer[0] = permanent;
	buffer[1] = channel;
	buffer[2] = (uint8)inputType;
	buffer[3] = (uint8)sensitivity + (uint8)location;
	*(int32*)(buffer+4*sizeof(uint8)) = sbgHostToTarget32(handle->targetOutputMode, nsDelay);

	//
	// Send the command used to set the sync IN channel options
	//
	error = sbgProtocolSend(handle, SBG_SET_SYNC_IN_CONF, buffer, 4*sizeof(uint8)+sizeof(int32));

	if (error == SBG_NO_ERROR)
	{
		//
		// Wait for an answer
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
	}
	
	return error;
}

/*!
 *	Get a logic input channel setting
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	channel				Channel to configure. May Be 0 or 1
 *	\param[out]	pInputType			Type of the logic input, may be <br>
 * 										- SBG_IN_DISABLED
 *										- SBG_IN_EVENT
 * 										- SBG_IN_MAIN_LOOP_START
 *										- SBG_IN_TIME_PULSE
 * 										- SBG_IN_ODOMETER	
 * \param[out]	pSensitivity		Sensitivity of the trigger. It may be:<br>
 * 										- SBG_IN_FALLING_EDGE
 * 										- SBG_IN_RISING_EDGE
 * 										- SBG_IN_LEVEL_CHANGE
 * \param[out]	pLocation			Physical location of the input pin for the syncIN channel 0 on IG-500E. <br>
 *										- SBG_IN_STD_LOCATION
 *										- SBG_IN_EXT_LOCATION
 * \param[out]	pNsDelay			Delay added before the actual trigger is taken into account (in nano seconds) delays up to +2seconds are possible:<:<br>
 *									This delay is only valid when the input is set to:
 * 										- SBG_IN_EVENT
 * 										- SBG_IN_MAIN_LOOP_START
 *										- SBG_IN_TIME_PULSE <br>
 * 									When used with the time pulse event, this delay can be negative, in order to simulate the time pulse propagation time
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgGetLogicInChannel(SbgProtocolHandle handle, uint8 channel, SbgLogicInType *pInputType, SbgLogicInSensitivity *pSensitivity, SbgLogicInLocation *pLocation, int32 *pNsDelay)
{
	uint8 cmd;
	uint8 buffer[3*sizeof(uint8)+sizeof(int32)];
	uint16 size;
	SbgErrorCode error;

	//
	// Send the command used to get our sync IN channel options
	//
	error = sbgProtocolSend(handle, SBG_GET_SYNC_IN_CONF, &channel, sizeof(uint8));

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read our answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, buffer, &size, 3*sizeof(uint8)+sizeof(int32));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_SYNC_IN_CONF) && (size == 3*sizeof(uint8)+sizeof(uint32)))
			{
				//
				// We have the right answer, so copy results to user variables
				//
				if (pInputType)
				{
					*pInputType = (SbgLogicInType)(buffer[1]);
				}
				if (pSensitivity)
				{
					*pSensitivity = (SbgLogicInSensitivity)(buffer[2]&SBG_IN_SENSE_MASK);
				}
				if (pLocation)
				{
					*pLocation = (SbgLogicInLocation)(buffer[2]&SBG_IN_EXT_LOCATION);
				}
				if (pNsDelay)
				{
					*pNsDelay = sbgTargetToHost32(handle->targetOutputMode, *(int32*)(buffer+3*sizeof(uint8)));
				}
			}
			else if ((cmd == SBG_ACK) && (size == sizeof(uint8)))
			{
				//
				// Get the error code contained in the ACK frame
				//
				error = (SbgErrorCode)(buffer[0]);

				//
				// We should receive an NACK so check if it's the case and return the error
				//
				if (error == SBG_NO_ERROR)
				{
					//
					// We have received an invalid frame, maybe a question/answer desync!
					//
					error = SBG_INVALID_FRAME;
				}
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
	}

	return error;
}

/*!
 *	Set a logic output channel setting
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	permanent			Store this setting permanently (TRUE) or temporary (FALSE)
 *	\param[in]	channel				Channel to configure. Set always 0
 *	\param[in]	outputType			Type of the logic output, may be <br>
 * 										- SBG_OUT_DISABLED
 *										- SBG_OUT_MAIN_LOOP_START
 *										- SBG_OUT_MAIN_LOOP_DIVIDER
 *										- SBG_OUT_TIME_PULSE_COPY
 *										- SBG_OUT_EVENT_COPY
 * \param[in]	polarity			Polarity of the out pulse. It may be:
 * 										- SBG_OUT_FALLING_EDGE
 * 										- SBG_OUT_RISING_EDGE
 * 										- SBG_OUT_TOGGLE
 *  \param[in]	duration			When the polarity is set to SBG_OUT_FALLING_EDGE or SBG_OUT_RISING_EDGE, this is the pulse duration in ms. <br>
 *									Leave to 0 if not used.
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgSetLogicOutChannel(SbgProtocolHandle handle, bool permanent, uint8 channel, SbgLogicOutType outputType, SbgLogicOutPolarity polarity, uint8 duration)
{
	uint8 buffer[5*sizeof(uint8)];
	SbgErrorCode error;

	//
	// Build our params buffer
	//
	buffer[0] = permanent;
	buffer[1] = channel;
	buffer[2] = (uint8)outputType;
	buffer[3] = (uint8)polarity;
	buffer[4] = duration;

	//
	// Send the command used to set the sync OUT options
	//
	error = sbgProtocolSend(handle, SBG_SET_SYNC_OUT_CONF, buffer, 5*sizeof(uint8));

	if (error == SBG_NO_ERROR)
	{
		//
		// Wait for an answer
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
	}
	
	return error;
}

/*!
 *	Get a logic output channel setting
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	channel				Channel to configure. Leave always to 0
 *	\param[out]	pOutputType			Type of the logic output, may be <br>
 * 										- SBG_OUT_DISABLED
 *										- SBG_OUT_MAIN_LOOP_START
 *										- SBG_OUT_MAIN_LOOP_DIVIDER
 *										- SBG_OUT_TIME_PULSE_COPY
 *										- SBG_OUT_EVENT_COPY
 * \param[out]	pPolarity			Polarity of the out pulse. It may be:
 * 										- SBG_OUT_FALLING_EDGE
 * 										- SBG_OUT_RISING_EDGE
 * 										- SBG_OUT_TOGGLE
 *  \param[out]	pDuration			When the polarity is set to SBG_OUT_FALLING_EDGE or SBG_OUT_RISING_EDGE, this is the pulse duration in ms. <br>
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgGetLogicOutChannel(SbgProtocolHandle handle, uint8 channel, SbgLogicOutType *pOutputType, SbgLogicOutPolarity *pPolarity, uint8 *pDuration)
{
	uint8 cmd;
	uint8 buffer[4];
	uint16 size;
	SbgErrorCode error;

	//
	// Send the command used to get our sync OUT channel options
	//
	error = sbgProtocolSend(handle, SBG_GET_SYNC_OUT_CONF, &channel, sizeof(uint8));

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read our answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, buffer, &size, 4*sizeof(uint8));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_SYNC_OUT_CONF) && (size == 4*sizeof(uint8)))
			{
				//
				// We have the right answer, so copy results to user variables
				//
				if (pOutputType)
				{
					*pOutputType = (SbgLogicOutType)(buffer[1]);
				}
				if (pPolarity)
				{
					*pPolarity = (SbgLogicOutPolarity)(buffer[2]);
				}
				if (pDuration)
				{
					*pDuration = buffer[3];
				}
			}
			else if ((cmd == SBG_ACK) && (size == sizeof(uint8)))
			{
				//
				// Get the error code contained in the ACK frame
				//
				error = (SbgErrorCode)(buffer[0]);

				//
				// We should receive an NACK so check if it's the case and return the error
				//
				if (error == SBG_NO_ERROR)
				{
					//
					// We have received an invalid frame, maybe a question/answer desync!
					//
					error = SBG_INVALID_FRAME;
				}
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
	}

	return error;
}