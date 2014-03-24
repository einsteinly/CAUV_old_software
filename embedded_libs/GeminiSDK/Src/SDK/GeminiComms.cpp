// GeminiComms.cpp : Defines the entry point for the DLL application.
//

#ifdef _WIN32
	#include "stdafx.h"
	#include "SerialPortEnum.h"
#else
	#include <cstdio>
	#include <string.h>
#endif
#include "GeminiNetwork.h"
#include "GeminiStructures.h"
#include "GeminiDefaultsLimits.h"

#define BUILDING_GEMINI_COMMS
//#define BUILDING_SDK
#include "GeminiComms.h"

#ifdef _MANAGED
#pragma managed(push, off)
#endif
//----------------------------------------------------------------------------

// Define a version constant for the DLL
// Version number corresponds to the current revision number of the document
// Interface Specification for Gemini Interface DLL - Jxxxx

const char cVerNum[8]   = "1.05.10";

//----------------------------------------------------------------------------
#ifdef _WIN32
BOOL APIENTRY DllMain(HMODULE hModule,
                      DWORD  ul_reason_for_call,
                      LPVOID lpReserved)
{
    return TRUE;
}
#endif

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// Global data

//----------------------------------------------------------------------------

CGeminiNetwork    geminiNetwork;
CGemPingConfig    geminiPingConfig;
CGemStatusRequest geminiStatusRequest;

int               geminiModFrequency              = 90422;

BOOL              geminiVDSLRestartRA             = FALSE;
BOOL              geminiVDSLInhibitRecommendedRA  = FALSE;
BOOL              geminiVDSLEnableRA              = FALSE;
BOOL              geminiEnableRAMonitoring        = FALSE;

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// Configuration commands

//----------------------------------------------------------------------------
void GEM_StartGeminiNetwork(unsigned short sonarID)
{
  geminiNetwork.m_pingStartBeam = geminiPingStartBeamDefault;
  geminiNetwork.m_pingEndBeam   = geminiPingEndBeamDefault;

	geminiNetwork.Init();
	geminiNetwork.SetDLLSonarID(sonarID);
}

//----------------------------------------------------------------------------
// Returns 1 if successfully started, 0 otherwise
int GEM_StartGeminiNetworkWithResult(unsigned short sonarID)
{
  bool success  = false;
  int  retValue = 0;

  geminiNetwork.m_pingStartBeam = geminiPingStartBeamDefault;
  geminiNetwork.m_pingEndBeam   = geminiPingEndBeamDefault;

	success = geminiNetwork.Init();
	geminiNetwork.SetDLLSonarID(sonarID);
	
	if (success)
	  retValue = 1;
	  
	return retValue;
}

//----------------------------------------------------------------------------
void GEM_SetGeminiSoftwareMode(char *softwareMode)
{
  if (strcmp(softwareMode, "Evo") == 0)
  {
    geminiNetwork.m_geminiCommsMode = geminiCommsModeEvo;
  }
  else
  if (strcmp(softwareMode, "SeaNet") == 0)
  {
    geminiNetwork.m_geminiCommsMode = geminiCommsModeSeaNet;
  }
  else
  if (strcmp(softwareMode, "SeaNetC") == 0)
  {
    geminiNetwork.m_geminiCommsMode = geminiCommsModeSeaNetComp;
  }
  else
  if (strcmp(softwareMode, "Prod") == 0)
  {
    geminiNetwork.m_geminiCommsMode = geminiCommsModeProd;
  }
  else
  if (strcmp(softwareMode, "EvoC") == 0)
  {
    geminiNetwork.m_geminiCommsMode = geminiCommsModeEvoComp;
  }
  else
  {
    geminiNetwork.m_geminiCommsMode = geminiCommsModeDefault;  
  }
}

//----------------------------------------------------------------------------
void GEM_SetHandlerFunction(void (cdecl *FnPtr)(int eType, int len, char *dataBlock))
{
  geminiNetwork.SetHandlerFunction(FnPtr);
}

//----------------------------------------------------------------------------
void GEM_SetProgressFunction(void (cdecl *FnPtr)(int sType, int retryCount, int packet, 
                                                 int block, int totalBlocks, int otherValue))
{
  geminiNetwork.SetProgressFunction(FnPtr);
}

//----------------------------------------------------------------------------
void GEM_ResetInternalCounters(void)
{

}

//----------------------------------------------------------------------------
void GEM_StopGeminiNetwork(void)
{
  geminiNetwork.Stop();
}

//----------------------------------------------------------------------------
unsigned short GEM_GetSonarID(void)
{
  return geminiNetwork.GetSonarID();
}

//----------------------------------------------------------------------------
void GEM_SetDLLSonarID(unsigned short sonarID)
{
  geminiNetwork.SetDLLSonarID(sonarID);
}

//----------------------------------------------------------------------------
void GEM_ProgramSonarID(unsigned short sonarID, int useHighFreq)
{
  if (geminiNetwork.m_geminiCommsMode == geminiCommsModeProd)
    geminiNetwork.ProgramSonarID(sonarID, useHighFreq);
}

//----------------------------------------------------------------------------
void GEM_ProgramVelocimeterCoeffs(unsigned int CCoeff, unsigned int MCoeff)
{
  if (geminiNetwork.m_geminiCommsMode == geminiCommsModeProd)
    geminiNetwork.ProgramVelocimeterCoeffs(CCoeff, MCoeff);
}

//----------------------------------------------------------------------------
void GEM_GetAltSonarIPAddress(unsigned char *a1, unsigned char *a2, unsigned char *a3, unsigned char *a4,
                              unsigned char *s1, unsigned char *s2, unsigned char *s3, unsigned char *s4)
{
  geminiNetwork.GetAltSonarIPAddress(a1, a2, a3, a4, s1, s2, s3, s4);
}

//----------------------------------------------------------------------------
void GEM_SetAltSonarIPAddress(unsigned char a1, unsigned char a2, unsigned char a3, unsigned char a4,
                              unsigned char s1, unsigned char s2, unsigned char s3, unsigned char s4)
{
  geminiNetwork.SetAltSonarIPAddress(a1, a2, a3, a4, s1, s2, s3, s4);
}

//----------------------------------------------------------------------------
void GEM_UseAltSonarIPAddress(unsigned char a1, unsigned char a2, unsigned char a3, unsigned char a4,
                              unsigned char s1, unsigned char s2, unsigned char s3, unsigned char s4)
{
  geminiNetwork.UseAltSonarIPAddress(a1, a2, a3, a4, s1, s2, s3, s4);
}

//----------------------------------------------------------------------------
void GEM_TxToAltIPAddress(int useAltIPAddress)
{
  if (useAltIPAddress)
    geminiNetwork.TxToAltIPAddress(true);
  else
    geminiNetwork.TxToAltIPAddress(false);
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// Ping configuration commands

//----------------------------------------------------------------------------

void GEM_SetGeminiModFrequency(int modFrequency)
{
  geminiModFrequency = modFrequency;
}

//----------------------------------------------------------------------------
void GEM_AutoPingConfig(float range, unsigned short spreadGain, float sos)
{
  // This code assumes that the Ping structure has been set to its default state
  // prior to this code being called, as it only calculate and sets some of the values
  // leaving all the others untouched.

  // Limit any incoming values
  if (range > geminiMaximumRange)
    range = geminiMaximumRange;

  // Set fixed items
  GEM_SetShadeBank(1);        // Shading                    15 dB
  GEM_SetExtModeTDBFlag(0);   // Time delay beamforming     ON
  GEM_SetExtModeFocusMode(0); // Focus mode                 2 WAY
  GEM_SetExtModeDataType(0);  // Data type                  BEAMFORMED MAGNITUDE
  GEM_SetExtModeDataSize(1);  // Data size                  8 BIT
  GEM_SetStartRange(0);       // Start range                0
  GEM_SetAbsorbGain(0);       // Absorbtion gain            0
  
  // Set and calculate variable members of the ping structure dependant on range, gain, and sos

  // Tx length
  if (range < 10.0)
  {
    GEM_SetTXLength(4);
  }
  else
  if ((range >= 10.0) && (range < 50.0))
  {
    GEM_SetTXLength(4 + (unsigned short)((28.0 * ((range - 10.0) / 40.0))));
  }
  else
  if ((range >= 50.0) && (range < 100.0))
  {
    GEM_SetTXLength(32 + (unsigned short)((96.0 * ((range - 50.0) / 50.0))));
  }
  else
  if (range >= 100.0)
  {
    GEM_SetTXLength(128);
  }
  
  // Filter // SPG:ToDo: Filter v range specification currently estimated
  if (range < 10.0)
  {
    GEM_SetFilterBank(7);
  }
  else
  if ((range >= 10.0) && (range < 15.0))
  {
    GEM_SetFilterBank(6);
  }
  else
  if ((range >= 15.0) && (range < 20.0))
  {
    GEM_SetFilterBank(5);
  }
  else
  if ((range >= 20.0) && (range < 25.0))
  {
    GEM_SetFilterBank(4);
  }
  else
  if ((range >= 25.0) && (range < 30.0))
  {
    GEM_SetFilterBank(3);
  }
  else
  if ((range >= 30.0) && (range < 40.0))
  {
    GEM_SetFilterBank(2);
  }
  else
  if ((range >= 40.0) && (range < 50.0))
  {
    GEM_SetFilterBank(1);
  }
  else
  if (range >= 50.0)
  {
    GEM_SetFilterBank(0);
  }
  
  // Speed of sound
  GEM_SetSpeedOfSound((unsigned short)(sos * 10.0));

  // Calculate end range in lines from the range, based on speed of sound
  double oneLineM        = (((double)sos / 2.0) / (double)geminiModFrequency);
  unsigned short endRngL = (unsigned short)floor((range / oneLineM) + 0.5);

  GEM_SetEndRange(endRngL);
  
  // Must do something with the gain as well
  // Set the spreading gain (range of 0 to 500 seems ideal)

#if 1
  // Use exponential gain curve    
  double calcGain = 100.0 * exp(((double)spreadGain - 100.0) / 25.0);
  GEM_SetSpreadGain((unsigned short)(calcGain * 5.0));
#else
  // Use linear gain curve
  GEM_SetSpreadGain((unsigned short)((double)spreadGain * 5.0));
#endif
  
  // Main gain (var)  is set to 1000
  // Main gain (base) is set to 11.0dB for the first five metres, 16.5dB for the rest of the range
  
  if (range < 5.0)
    GEM_SetMainGain(2, 1000);
  else
    GEM_SetMainGain(3, 1000);

  // Software gain is fixed
  
  GEM_SetSoftGain(0x00001e00);

  // Internal software gain ramp of Gemini sonar is set to 18 (chosen by Neil)
 
  GEM_SetSoftwareGainRamp(18);
  
  // Disable OOW
  
  GEM_SetExtModeOutOfWaterOverride(1);
}

//----------------------------------------------------------------------------
void GEM_SetGeminiEvoQuality(unsigned char evoQualitySetting)
{
  if (evoQualitySetting <= geminiEvoRangeCompressionMax)
    geminiNetwork.m_geminiEvoRangeCompression = evoQualitySetting;
}

//----------------------------------------------------------------------------
unsigned short GEM_GetRequestedCompressionFactor(void)
{
  return geminiNetwork.m_geminiCompressionFactorUsed;
}

//----------------------------------------------------------------------------
unsigned short GEM_GetRXAddress1(void)
{
  return geminiPingConfig.m_rxIdent1;
}

//----------------------------------------------------------------------------
void GEM_SetRXAddress1(unsigned short rxAddress1)
{
  geminiPingConfig.m_rxIdent1 = rxAddress1;
}

//----------------------------------------------------------------------------
unsigned short GEM_GetRXAddress2(void)
{
  return geminiPingConfig.m_rxIdent2;
}

//----------------------------------------------------------------------------
void GEM_SetRXAddress2(unsigned short rxAddress2)
{
  geminiPingConfig.m_rxIdent2 = rxAddress2;
}

//----------------------------------------------------------------------------
unsigned short GEM_GetTXAddress1(void)
{
  return geminiPingConfig.m_txIdent1;
}

//----------------------------------------------------------------------------
void GEM_SetTXAddress1(unsigned short txAddress1)
{
  geminiPingConfig.m_txIdent1 = txAddress1;
}

//----------------------------------------------------------------------------
unsigned short GEM_GetTXAddress2(void)
{
  return geminiPingConfig.m_txIdent2;
}

//----------------------------------------------------------------------------
void GEM_SetTXAddress2(unsigned short txAddress2)
{
  geminiPingConfig.m_txIdent2 = txAddress2;
}

//----------------------------------------------------------------------------
void GEM_SetRunMode(unsigned short triggerEdge, unsigned short pingMethod)
{
  unsigned short localRunMode = pingMethod & 0x0003;

  if (triggerEdge)
    localRunMode |= 0x8000;
  
  geminiPingConfig.m_runMode = localRunMode;     
}

//----------------------------------------------------------------------------
void GEM_SetPingMode(unsigned short pingMethod)
{
  geminiPingConfig.m_runMode = pingMethod & 0x0001;     
}

//----------------------------------------------------------------------------
void GEM_SetExtModeDataType(unsigned short dataType)
{
  unsigned short localExtMode = geminiPingConfig.m_extModeFlags;

  localExtMode &= 0xfffc;
  localExtMode |= (dataType & 0x0003);
  
  geminiPingConfig.m_extModeFlags = localExtMode;
}

//----------------------------------------------------------------------------
void GEM_SetExtModeDataSize(unsigned short dataSize)
{
  unsigned short localExtMode = geminiPingConfig.m_extModeFlags;
  
  localExtMode &= 0xfffb;

  if (dataSize)
    localExtMode |= 0x0004;
  
  geminiPingConfig.m_extModeFlags = localExtMode;
}

//----------------------------------------------------------------------------
void GEM_SetExtModeFocusMode(unsigned short focusMode)
{
  unsigned short localExtMode = geminiPingConfig.m_extModeFlags;
  
  localExtMode &= 0xffe7;

  switch (focusMode)
  {
    case 0:   // Deliberate drop through
    default:
    
      // Do nothing
      
      break;
      
    case 1:
    
      localExtMode |= 0x0008;   

      break;

    case 2:
    
      localExtMode |= 0x0010;   

      break;

    case 3:
    
      localExtMode |= 0x0018;   

      break;
  }
  
  geminiPingConfig.m_extModeFlags = localExtMode;

}

//----------------------------------------------------------------------------
void GEM_SetExtModeVelDataEnable(unsigned short velDataEnable)
{
  unsigned short localExtMode = geminiPingConfig.m_extModeFlags;
  
  localExtMode &= 0xfeff;

  if (velDataEnable)
    localExtMode |= 0x0100;
  
  geminiPingConfig.m_extModeFlags = localExtMode;
}

//----------------------------------------------------------------------------
void GEM_SetExtModeOutOfWaterOverride(unsigned short outOfWaterOverride)
{
  unsigned short localExtMode = geminiPingConfig.m_extModeFlags;
  
  localExtMode &= 0xfdff;

  if (outOfWaterOverride)
    localExtMode |= 0x0200;
  
  geminiPingConfig.m_extModeFlags = localExtMode;
}

//----------------------------------------------------------------------------
void GEM_SetExtModeScopeEnable(unsigned short scopeEnable)
{
  unsigned short localExtMode = geminiPingConfig.m_extModeFlags;
  
  localExtMode &= 0xefff;

  if (scopeEnable)
    localExtMode |= 0x1000;
  
  geminiPingConfig.m_extModeFlags = localExtMode;
}

//----------------------------------------------------------------------------
void GEM_SetExtModeTDBFlag(unsigned short tdbFlag)
{
  unsigned short localExtMode = geminiPingConfig.m_extModeFlags;
  
  localExtMode &= 0xdfff;

  if (tdbFlag)
    localExtMode |= 0x2000;
  
  geminiPingConfig.m_extModeFlags = localExtMode;
}

//----------------------------------------------------------------------------
void GEM_SetExtModeSimCQIFlag(unsigned short simCQIFlag)
{
  unsigned short localExtMode = geminiPingConfig.m_extModeFlags;
  
  localExtMode &= 0xbfff;

  if (simCQIFlag)
    localExtMode |= 0x4000;
  
  geminiPingConfig.m_extModeFlags = localExtMode;
}

//----------------------------------------------------------------------------
void GEM_SetExtModeSimADCFlag(unsigned short simADCFlag)
{
  unsigned short localExtMode = geminiPingConfig.m_extModeFlags;
  
  localExtMode &= 0x7fff;

  if (simADCFlag)
    localExtMode |= 0x8000;
  
  geminiPingConfig.m_extModeFlags = localExtMode;
}

//----------------------------------------------------------------------------
void GEM_SetExtModeVelDataEnableOnce(unsigned short velDataEnableOnce)
{
  unsigned short localExtMode = geminiPingConfig.m_extModeFlags;
  
  localExtMode &= 0xfbff;

  if (velDataEnableOnce)
    localExtMode |= 0x0400;
  
  geminiPingConfig.m_extModeFlags = localExtMode;
}

//----------------------------------------------------------------------------
void GEM_SetStartRange(unsigned short rangeInLines)
{
  geminiPingConfig.m_startRange = rangeInLines;
}

//----------------------------------------------------------------------------
void GEM_SetEndRange(unsigned short rangeInLines)
{
  // The head reacts badly if less than 32 range lines are requested, so never
  // send an end range of less than 32

  if (rangeInLines < 32)
    geminiPingConfig.m_endRange = 32;
  else
    geminiPingConfig.m_endRange = rangeInLines;
  
  // Are we using any of the compressed modes?
  if (geminiNetwork.m_geminiCommsMode == geminiCommsModeSeaNetComp)
  {
    // We are running in Seanet compressed mode - which means that
    // we want to return no more than 1500 lines to Seanet. Use the
    // range compression built into the head to achieve this.
    
    // Cast to a short as this statement will always be true because of the
    // datatype. It's not a big issue but removes some g++ warnings.
    if (((short)rangeInLines >= 0) && (rangeInLines < 1500))
    {
      // Set the head to no range compression, using average compression
      GEM_SetRangeCompression(0, 0);
    }
    else
    if ((rangeInLines >= 1500) && (rangeInLines < 3000))
    {
      // Set the head to 2 * range compression, using average compression
      GEM_SetRangeCompression(1, 0);
    }
    else
    if ((rangeInLines >= 3000) && (rangeInLines < 6000))
    {
      // Set the head to 4 * range compression, using average compression
      GEM_SetRangeCompression(2, 0);
    }
    else
    if ((rangeInLines >= 6000) && (rangeInLines < 12000))
    {
      // Set the head to 8 * range compression, using average compression
      GEM_SetRangeCompression(3, 0);
    }
    else
    if ((rangeInLines >= 12000) && (rangeInLines < 24000))
    {
      // Set the head to 16 * range compression, using average compression
      GEM_SetRangeCompression(4, 0);
    }
    else
    if (rangeInLines >= 24000)
    {
      // Set the head to 16 * range compression, using average compression
      // and limit the number of range lines to 24000
      GEM_SetRangeCompression(4, 0);
      geminiPingConfig.m_endRange = 24000;
    }
  }
  else
  if (geminiNetwork.m_geminiCommsMode == geminiCommsModeEvoComp)
  {
    // We are running in Evo compressed mode - which means that
    // we want to return no more than the number of lines defined by
    // the quality of the display Evo is using. Use the
    // range compression built into the head to achieve this.
    
    if (((short)rangeInLines >= 0) && 
        (rangeInLines < geminiNetwork.m_geminiCompressionLines[geminiNetwork.m_geminiEvoRangeCompression]))
    {
      // Set the head to no range compression, using average compression
      GEM_SetRangeCompression(0, 0);
    }
    else
    if ((rangeInLines >=  geminiNetwork.m_geminiCompressionLines[geminiNetwork.m_geminiEvoRangeCompression]) && 
        (rangeInLines <  (geminiNetwork.m_geminiCompressionLines[geminiNetwork.m_geminiEvoRangeCompression] * 2)))
    {
      // Set the head to 2 * range compression, using average compression
      GEM_SetRangeCompression(1, 0);
    }
    else
    if ((rangeInLines >= (geminiNetwork.m_geminiCompressionLines[geminiNetwork.m_geminiEvoRangeCompression] * 2)) && 
        (rangeInLines <  (geminiNetwork.m_geminiCompressionLines[geminiNetwork.m_geminiEvoRangeCompression] * 4)))
    {
      // Set the head to 4 * range compression, using average compression
      GEM_SetRangeCompression(2, 0);
    }
    else
    if (rangeInLines >= (geminiNetwork.m_geminiCompressionLines[geminiNetwork.m_geminiEvoRangeCompression] * 4)) 
    {
      // Set the head to 8 * range compression, using average compression
      GEM_SetRangeCompression(3, 0);
    }

#if 0
    if ((rangeInLines >= (geminiNetwork.m_geminiCompressionLines[geminiNetwork.m_geminiEvoRangeCompression] * 4)) && 
        (rangeInLines <  (geminiNetwork.m_geminiCompressionLines[geminiNetwork.m_geminiEvoRangeCompression] * 8)))
    {
      // Set the head to 8 * range compression, using average compression
      GEM_SetRangeCompression(3, 0);
    }
    else
    if (rangeInLines >= (geminiNetwork.m_geminiCompressionLines[geminiNetwork.m_geminiEvoRangeCompression] * 8)) 
    {
      // Set the head to 16 * range compression, using average compression
      GEM_SetRangeCompression(4, 0);
    }
#endif
  }

  // Final checks, look at the mode which has already been set and limit the number of lines to the maximum
  // that the buffers in the Gemini hardware can cope with
  
  #define GEMINI_HW_MAX_LINES_MODE0 20000    // geminiMaximumRangeInLines
  #define GEMINI_HW_MAX_LINES_MODE1 5000    // geminiMaximumRangeInLines
  #define GEMINI_HW_MAX_LINES_MODE2 5000    // geminiMaximumRangeInLines
  #define GEMINI_HW_MAX_LINES_MODE3 8000    // geminiMaximumRangeInLines
  
  if ((geminiPingConfig.m_extModeFlags & 0x0003) == 0x0000)
  {
    // Beamformed magnitude
    if ((geminiPingConfig.m_extModeFlags & 0x0004) == 0x0004)
    {
      // 8 bit data
      if (rangeInLines > GEMINI_HW_MAX_LINES_MODE0)
        geminiPingConfig.m_endRange = GEMINI_HW_MAX_LINES_MODE0;
    }
    else
    {
      // 16 bit data
      if (rangeInLines > GEMINI_HW_MAX_LINES_MODE0 / 2)
        geminiPingConfig.m_endRange = GEMINI_HW_MAX_LINES_MODE0 /2;
    }
  }
  else
  if ((geminiPingConfig.m_extModeFlags & 0x0003) == 0x0001)
  {
    // Beamformed QI
    if (rangeInLines > GEMINI_HW_MAX_LINES_MODE1)
      geminiPingConfig.m_endRange = GEMINI_HW_MAX_LINES_MODE1;
  }
  else
  if ((geminiPingConfig.m_extModeFlags & 0x0003) == 0x0002)
  {
    // Channel QI
    if (rangeInLines > GEMINI_HW_MAX_LINES_MODE2)
      geminiPingConfig.m_endRange = GEMINI_HW_MAX_LINES_MODE2;
  }
  else
  if ((geminiPingConfig.m_extModeFlags & 0x0003) == 0x0003)
  {
    // Channel samples
    if (rangeInLines > GEMINI_HW_MAX_LINES_MODE3)
      geminiPingConfig.m_endRange = GEMINI_HW_MAX_LINES_MODE3;
  }
}

//----------------------------------------------------------------------------
void GEM_SetInterPingPeriod(unsigned int periodInMicroSeconds)
{
  geminiPingConfig.m_interping = periodInMicroSeconds;
}

//----------------------------------------------------------------------------
void GEM_SetTXLength(unsigned short txLength)
{
  unsigned short localTXLength = txLength & 0x00ff;

  // Special case code - if TXLength is set to zero also set TXMark to zero
  // so that the transmitter will not transmit at all when the TXLength is
  // set to zero.

  if (localTXLength == 0)
  {
    geminiPingConfig.m_txMark              = 0;
    geminiPingConfig.m_velocimeterTXPeriod = 0;
	  geminiPingConfig.m_velocimeterTXMark   = 0;
  }
  else
  {
    geminiPingConfig.m_txMark              = geminiTXMarkDefault;
    geminiPingConfig.m_velocimeterTXPeriod = geminiVelocimeterTXPeriodDefault;
	  geminiPingConfig.m_velocimeterTXMark   = geminiVelocimeterTXMarkDefault;
  }
  
  geminiPingConfig.m_txLength = localTXLength;
}

//----------------------------------------------------------------------------
void GEM_SetTXAngle(unsigned short txAngle)
{
  geminiPingConfig.m_txAngle = txAngle;
}

//----------------------------------------------------------------------------
void GEM_SetMainGain(unsigned short baseGain, unsigned short variableGain)
{
  unsigned short localGain = variableGain & 0x3fff;
  
  switch (baseGain)
  {
    case 0:   // Deliberate drop through
    default:
    
      // Do nothing
      
      break;
      
    case 1:
    
      localGain |= 0x4000;   

      break;

    case 2:
    
      localGain |= 0x8000;   

      break;

    case 3:
    
      localGain |= 0xc000;   

      break;
  }
  
  geminiPingConfig.m_mainGain = localGain;
}

//----------------------------------------------------------------------------
void GEM_SetMainGainPerCent(unsigned short perCentGain)
{
  // GEM_SetMainGainUnits takes a value between 0 and 665, so lets scale our
  // percerntage value from 0 to 100 to 0 to 665 and use GEM_SetMainGainUnits
  
  unsigned int localGain = ((unsigned int)perCentGain * 665) / 100;
  
  GEM_SetMainGainUnits((unsigned short)localGain);
}

//----------------------------------------------------------------------------
void GEM_SetMainGainUnits(unsigned short dBGain)
{
  // dbGain is the gain required, expressed in dB * 10, between 0dB and 66.5dB (dbGain = 665)

  // GEM_SetMainGain takes two values, base gain and variable gain
  // Base gain is either 0, 5.5, 11 or 16.5dB, expressed as 0, 1, 2, or 3.
  // Variable gain is between 0 and 50dB, expressed as number between 0 and 4095
  
  // Using the value of dBGain, and wanting to minimise the base gain,
  // lets calculate the settings for these two values
  
  unsigned short baseGain       = 0;
  unsigned short variableGain   = 0;
  unsigned int   variableGaindB = 0;
  
  if (dBGain < 80)
  {
    // Minimum gain applied by the amplifiers is 8dB
    // We don't need to adjust base gain value
    variableGaindB = 80;
  }
  else
  if ((dBGain >= 80) && (dBGain < 500))
  {
    // We don't need to adjust base gain value
    variableGaindB = dBGain;
  }
  else
  if ((dBGain >= 500) && (dBGain < 555))
  {
    baseGain = 1;
    variableGaindB = dBGain - 55;
  }
  else
  if ((dBGain >= 555) && (dBGain < 610))
  {
    baseGain = 2;
    variableGaindB = dBGain - 110;
  }
  else
  if ((dBGain >= 610) && (dBGain <= 665))
  {
    baseGain = 3;
    variableGaindB = dBGain - 165;
  }
  else
  {
    // Value is out of range
  }
  
  // Need to adjust variableGain from dB to a value between 0 and 4095
  // Actually a second look at the graphs for amplifier chips shows that
  // the gain increases by 42dB for a range of 2700 steps (roughly)
  
  variableGain = (unsigned short)((((variableGaindB - 80) * 2700) / 420) + 1000);

  // And use the values we have just calculated
  
  GEM_SetMainGain(baseGain, variableGain);
}

//----------------------------------------------------------------------------
void GEM_SetAbsorbGain(unsigned short absorbtionGain)
{
  // Absorbtion gain is limited to 12 bits
  geminiPingConfig.m_absorption = absorbtionGain & 0x0fff;
}

//----------------------------------------------------------------------------
void GEM_SetSoftGain(unsigned int softwareGain)
{
  geminiPingConfig.m_softwareGain = softwareGain;
}

//----------------------------------------------------------------------------
void GEM_SetSpreadGain(unsigned short spreadingGain)
{
  // Spreading gain is limited to 10 bits
  geminiPingConfig.m_spreading = spreadingGain & 0x03ff;
}

//----------------------------------------------------------------------------
void GEM_SetAutoGain(unsigned short autoGainFlag, unsigned short targetGain)
{
  unsigned short localAutoGain = targetGain & 0x0007;
  
  if (autoGainFlag)
    localAutoGain |= 0x0008;
    
  geminiPingConfig.m_autoGain = localAutoGain;
}

//----------------------------------------------------------------------------
void GEM_SetSoftwareGainRamp(unsigned short softwareGainRamp)
{
  unsigned char localGainRamp = softwareGainRamp & 0xff;
  
  geminiPingConfig.m_softwareGainRamp = localGainRamp;
}

//----------------------------------------------------------------------------
void GEM_SetFilterBank(unsigned short channelFIRFilterBandwidth)
{
  unsigned short localChannelFIRFilterBandwidth = channelFIRFilterBandwidth & 0x0007;

  geminiPingConfig.m_filterBank = localChannelFIRFilterBandwidth;
}

//----------------------------------------------------------------------------
void GEM_SetShadeBank(unsigned short beamformerShadingLevel)
{
  unsigned short localBeamformerShadingLevel = beamformerShadingLevel & 0x0007;

  geminiPingConfig.m_shadeBank = localBeamformerShadingLevel;
}

//----------------------------------------------------------------------------
void GEM_SetSpeedOfSound(unsigned short speedOfSound)
{
  geminiPingConfig.m_spdSnd = speedOfSound;
}

//----------------------------------------------------------------------------
void GEM_SetVelocimeterMode(unsigned short gainMode, unsigned short outputMode)
{
  unsigned short localVelocimeterMode = 0;
  
  if (outputMode)
    localVelocimeterMode = 1;

  if (gainMode)
    localVelocimeterMode |= 0x8000;

  geminiPingConfig.m_velMode = localVelocimeterMode;
}

//----------------------------------------------------------------------------
void GEM_SetRangeCompression(unsigned short compressionLevel, unsigned short compressionType)
{
  unsigned short localRangeCompression = 0;
  
  if (compressionType)
    localRangeCompression = 0x100;
    
  switch (compressionLevel)
  {
    case 0: // Deliberate drop through
    default:
    
      // Do nothing
      geminiNetwork.m_geminiCompressionFactorUsed = 1;
      
      break;
    
    case 1:
    
      localRangeCompression |= 0x01;
      geminiNetwork.m_geminiCompressionFactorUsed = 2;
      
      break;
      
    case 2:
    
      localRangeCompression |= 0x02;
      geminiNetwork.m_geminiCompressionFactorUsed = 4;
      
      break;
      
    case 3:
    
      localRangeCompression |= 0x03;
      geminiNetwork.m_geminiCompressionFactorUsed = 8;
      
      break;
      
    case 4:
    
      localRangeCompression |= 0x04;
      geminiNetwork.m_geminiCompressionFactorUsed = 16;
      
      break;
  }

  geminiPingConfig.m_rngComp = localRangeCompression;
}

//----------------------------------------------------------------------------
void GEM_SetRLEThreshold(unsigned short threshold)
{
  unsigned short localThreshold = threshold & 0x00ff;

  geminiPingConfig.m_rleThreshold = localThreshold;
}

//----------------------------------------------------------------------------
void GEM_SetChannelSample(unsigned short channelSample)
{
  unsigned short localChannelSample = channelSample & 0x00ff;

  geminiPingConfig.m_chanSamp = localChannelSample;
}

//----------------------------------------------------------------------------
void GEM_SetVelocimeterGain(unsigned short gain)
{
  geminiPingConfig.m_velGain = gain;
}

//----------------------------------------------------------------------------
void GEM_SetVelocimeterTXLength(unsigned char txLength)
{
  geminiPingConfig.m_velTxLength = txLength;
}

//----------------------------------------------------------------------------
void GEM_SetVelocimeterTXPeriod(unsigned short velocimeterTXPeriod)
{
  unsigned char localvelocimeterTXPeriod = velocimeterTXPeriod & 0xff;

  geminiPingConfig.m_velocimeterTXPeriod = localvelocimeterTXPeriod;
}

//----------------------------------------------------------------------------
void GEM_SetVelocimeterTXMark(unsigned short velocimeterTXMark)
{
  unsigned char localvelocimeterTXMark = velocimeterTXMark & 0xff;

  geminiPingConfig.m_velocimeterTXMark = localvelocimeterTXMark;
}

//----------------------------------------------------------------------------
void GEM_SetStartBeam(unsigned short startBeam)
{
  geminiNetwork.m_pingStartBeam = startBeam;
}

//----------------------------------------------------------------------------
void GEM_SetEndBeam(unsigned short endBeam)
{
  geminiNetwork.m_pingEndBeam = endBeam;
}

//----------------------------------------------------------------------------
void GEM_SetPingToDefaults(void)
{
  geminiPingConfig.InitPingConfig();
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// Transmit to sonar commands

//----------------------------------------------------------------------------
void GEM_SendGeminiStayAlive(void)
{
	geminiNetwork.SendMsg(&geminiStatusRequest, sizeof(geminiStatusRequest));
}

//----------------------------------------------------------------------------
void GEM_SendGeminiPingConfig(void)
{
	geminiNetwork.SendMsg(&geminiPingConfig, sizeof(geminiPingConfig));
}

//----------------------------------------------------------------------------
void GEM_RebootSonar(void)
{
  geminiNetwork.RebootSonar();
}

//----------------------------------------------------------------------------
void GEM_InhibitSonarReboot(void)
{
  if (geminiNetwork.m_geminiCommsMode == geminiCommsModeProd)
    geminiNetwork.InhibitSonarReboot();
}

//----------------------------------------------------------------------------
void GEM_SendRS232(int len, char *data)
{
  geminiNetwork.SendRS232Data(len, data);
}

//----------------------------------------------------------------------------
void GEM_CommandPOST(void)
{
  geminiNetwork.CommandPOST();
}

//----------------------------------------------------------------------------
void GEM_CommandProductionTest(bool runProdTest, unsigned long long seedValue)
{
  if (geminiNetwork.m_geminiCommsMode == geminiCommsModeProd)
    geminiNetwork.CommandProductionTest(runProdTest, seedValue);
}

//----------------------------------------------------------------------------
void GEM_AbortProductionTest(void)
{
  geminiNetwork.AbortProductionTest();
}

//----------------------------------------------------------------------------
void GEM_RequestTestResult(void)
{
  geminiNetwork.RequestTestResult();
}

//----------------------------------------------------------------------------
void GEM_RequestRetry(unsigned short type, unsigned short lineNo, unsigned short noOfLines)
{
  if (((short)type >= GEM_RETRY_REQUEST_HEAD) &&
      (type <= GEM_RETRY_REQUEST_TAIL))
  {
    geminiNetwork.RequestRetry(type, lineNo, noOfLines);
  }
}

//----------------------------------------------------------------------------
void GEM_SendGeminiNetworkConfig(void)
{
  // Build retriggerRateAdaptionMember variable from constituent elements
  geminiNetwork.m_geminiNetworkConfig.m_retriggerRateAdaption = 0;

  // Trigger VDSL rate adaption restart
  if (geminiVDSLRestartRA == TRUE)
    geminiNetwork.m_geminiNetworkConfig.m_retriggerRateAdaption |= 0x01;

  // Inhibit recommended rate adaptions
  if (geminiVDSLInhibitRecommendedRA == TRUE)
    geminiNetwork.m_geminiNetworkConfig.m_retriggerRateAdaption |= 0x02;

  // Enable any rate adaptions
  if (geminiVDSLEnableRA == TRUE)
    geminiNetwork.m_geminiNetworkConfig.m_retriggerRateAdaption |= 0x04;

  // Enable rate adaption monitoring by VDSL chip
  if (geminiEnableRAMonitoring == TRUE)
    geminiNetwork.m_geminiNetworkConfig.m_retriggerRateAdaption |= 0x08;

  // Send the network config
  geminiNetwork.SendNetworkConfig();

  // Reset the rate adaption restart trigger so we don't do it if sent again
  geminiVDSLRestartRA = FALSE;
}

//----------------------------------------------------------------------------
void GEM_SetNetworkIFGs(unsigned char ethernetIFG, unsigned char VDSLIFG)
{
  geminiNetwork.m_geminiNetworkConfig.m_ethernetIFG = ethernetIFG;
  geminiNetwork.m_geminiNetworkConfig.m_VDSLIFG     = VDSLIFG;
}

//----------------------------------------------------------------------------
void GEM_GetNetworkIFGs(unsigned char *ethernetIFG, unsigned char *VDSLIFG)
{
  *ethernetIFG = geminiNetwork.m_geminiNetworkConfig.m_ethernetIFG;
  *VDSLIFG     = geminiNetwork.m_geminiNetworkConfig.m_VDSLIFG;
}

//----------------------------------------------------------------------------
void GEM_SetNetworkAdvertiseGigaBit(unsigned short advertiseGigaBit)
{
  if (advertiseGigaBit)
    geminiNetwork.m_geminiNetworkConfig.m_advertiseGigaBit = (unsigned char)advertiseGigaBit;
}

//----------------------------------------------------------------------------
void GEM_SetNetworkRetriggerVDSLRateAdaption(unsigned short retriggerVDSLadaption)
{
  if (retriggerVDSLadaption)
    geminiVDSLRestartRA = TRUE;
  else
    geminiVDSLRestartRA = FALSE;    
}

//----------------------------------------------------------------------------
void GEM_SetNetworkInhibitRecommendedRateAdaption(unsigned short inhibitRecommendedRA)
{
  if (inhibitRecommendedRA)
    geminiVDSLInhibitRecommendedRA = TRUE;
  else
    geminiVDSLInhibitRecommendedRA = FALSE;    
}

//----------------------------------------------------------------------------
void GEM_SetNetworkEnableAllRateAdaption(unsigned short enableAllRA)
{
  if (enableAllRA)
    geminiVDSLEnableRA = TRUE;
  else
    geminiVDSLEnableRA = FALSE;    
}

//----------------------------------------------------------------------------
void GEM_SetNetworkEnableRAMonitoring(unsigned short enableRAMonitoring)
{
  if (enableRAMonitoring)
    geminiEnableRAMonitoring = TRUE;
  else
    geminiEnableRAMonitoring = FALSE;
}

//----------------------------------------------------------------------------
void GEM_SetNetworkRS232BaudRate(unsigned int baudRate)
{
  switch (baudRate)
  {
    case 2400:
      geminiNetwork.m_geminiNetworkConfig.m_baudRate        = geminiBaud2400;
      geminiNetwork.m_geminiNetworkConfig.m_RS232PollPeriod = geminiPollPeriodFor2400Baud;
      break;

    case 4800:
      geminiNetwork.m_geminiNetworkConfig.m_baudRate        = geminiBaud4800;
      geminiNetwork.m_geminiNetworkConfig.m_RS232PollPeriod = geminiPollPeriodFor4800Baud;
      break;

    case 9600:
      geminiNetwork.m_geminiNetworkConfig.m_baudRate        = geminiBaud9600;
      geminiNetwork.m_geminiNetworkConfig.m_RS232PollPeriod = geminiPollPeriodFor9600Baud;
      break;
      
    case 19200:
      geminiNetwork.m_geminiNetworkConfig.m_baudRate        = geminiBaud19200;
      geminiNetwork.m_geminiNetworkConfig.m_RS232PollPeriod = geminiPollPeriodFor19200Baud;
      break;
      
    case 38400:
      geminiNetwork.m_geminiNetworkConfig.m_baudRate        = geminiBaud38400;
      geminiNetwork.m_geminiNetworkConfig.m_RS232PollPeriod = geminiPollPeriodFor38400Baud;
      break;
      
    case 57600:
      geminiNetwork.m_geminiNetworkConfig.m_baudRate        = geminiBaud57600;
      geminiNetwork.m_geminiNetworkConfig.m_RS232PollPeriod = geminiPollPeriodFor57600Baud;
      break;
      
    case 115200:
      geminiNetwork.m_geminiNetworkConfig.m_baudRate        = geminiBaud115200;
      geminiNetwork.m_geminiNetworkConfig.m_RS232PollPeriod = geminiPollPeriodFor115200Baud;
      break;
      
    default:
      geminiNetwork.m_geminiNetworkConfig.m_baudRate        = geminiBaud9600;
      geminiNetwork.m_geminiNetworkConfig.m_RS232PollPeriod = geminiPollPeriodFor9600Baud;
      break;
  }
}
//----------------------------------------------------------------------------
void GEM_SetNetworkConfigToDefaults(void)
{
  geminiVDSLRestartRA             = FALSE;
  geminiVDSLInhibitRecommendedRA  = FALSE;
  geminiVDSLEnableRA              = FALSE;
  geminiEnableRAMonitoring        = FALSE;

  geminiNetwork.m_geminiNetworkConfig.InitNetworkConfig();
}

//----------------------------------------------------------------------------
void GEM_MDIOInit(void)
{
  geminiNetwork.m_geminiMDIOPacket.Clear();
}

//----------------------------------------------------------------------------
void GEM_MDIOAddVDSLReg(unsigned short regAddr, unsigned short value)
{
  geminiNetwork.m_geminiMDIOPacket.AddVDSLRegWriteCommand(regAddr, value);
}

//----------------------------------------------------------------------------
void GEM_MDIOAddSwitchToEthernet(void)
{
  geminiNetwork.m_geminiMDIOPacket.AddSwitchEthernetCommand();
}

//----------------------------------------------------------------------------
void GEM_MDIOAddSwitchToVDSL(void)
{
  geminiNetwork.m_geminiMDIOPacket.AddSwitchVDSLCommand();
}

//----------------------------------------------------------------------------
void GEM_MDIOSend(void)
{
  geminiNetwork.SendMDIOPacket();
}

//----------------------------------------------------------------------------
void GEM_SetVDSLSetting(unsigned short level)
{
  if (level <= 2)
  {
    geminiNetwork.m_VDSLSettingsToUse = level;
    
    geminiNetwork.ConfigureVDSLToCurrentDefault();
  }
}

//----------------------------------------------------------------------------
void GEM_SetNetworkInterPacketGap(unsigned short interPacketGap)
{
  geminiNetwork.m_geminiNetworkConfig.m_interPacketGap = interPacketGap;
}

//----------------------------------------------------------------------------
void GEM_SetNetworkRS232Routing(unsigned short RS232Routing)
{
  unsigned short localRS232Routing = RS232Routing & 0x0003;
  
  geminiNetwork.m_geminiNetworkConfig.m_RS232Routing = localRS232Routing;
}

//----------------------------------------------------------------------------
unsigned short GEM_GetNetworkInterPacketGap(void)
{
  return geminiNetwork.m_geminiNetworkConfig.m_interPacketGap;
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// Modifiers and Flash Programming commands

//----------------------------------------------------------------------------
void GEM_ProgramFPGA(char *fileName, char *destination)
{
  if (geminiNetwork.m_geminiCommsMode == geminiCommsModeProd)
    geminiNetwork.ProgramFPGA(fileName, destination);
}

//----------------------------------------------------------------------------
void GEM_AbortFPGAProgramming(void)
{
  if (geminiNetwork.m_geminiCommsMode == geminiCommsModeProd)
    geminiNetwork.AbortFPGAProgramming();
}

//----------------------------------------------------------------------------
void GEM_InitCalibrationValues(bool useFiles, char *txFile, char *rxFile)
{
  geminiNetwork.InitCalibrationValues(useFiles, txFile, rxFile);
}

//----------------------------------------------------------------------------
int GEM_CalculateModifiers(void)
{
  if (geminiNetwork.CalculateModifiers())
    return 1;
  else
    return 0;
}

//----------------------------------------------------------------------------
void GEM_ProgramModifiers(void)
{
  if (geminiNetwork.m_geminiCommsMode == geminiCommsModeProd)
    geminiNetwork.ProgramModifiers();
}

//----------------------------------------------------------------------------
void GEM_AbortModifierProgramming(void)
{
  if (geminiNetwork.m_geminiCommsMode == geminiCommsModeProd)
    geminiNetwork.AbortModifierProgramming();
}

//----------------------------------------------------------------------------
int GEM_WriteModifiersToFile(char *fileName)
{
  return geminiNetwork.WriteModifiersToFile(fileName);
}

//----------------------------------------------------------------------------
int GEM_ReadModifiersFromFile(char *fileName)
{
  return geminiNetwork.ReadModifiersFromFile(fileName);
}

//----------------------------------------------------------------------------
void GEM_ReadFromFlash(unsigned int addressToRead)
{
  geminiNetwork.ReadFromFlash(addressToRead);
}

//----------------------------------------------------------------------------
void GEM_WriteToFlash(unsigned int addressToWrite, unsigned short readbackFlag, unsigned char *data)
{
  geminiNetwork.WriteToFlash(addressToWrite, readbackFlag, data);
}

//----------------------------------------------------------------------------
void GEM_ReadFlashInfoBlock(unsigned short blockToRead)
{
  geminiNetwork.ReadFlashInfoBlock(blockToRead);
}

//----------------------------------------------------------------------------
void GEM_SendGPIOReadCommand(void)
{
  geminiNetwork.SendGPIOReadCommand();
}

//----------------------------------------------------------------------------
void GEM_RequestDiagnosticInfo(void)
{
  geminiNetwork.RequestDiagnosticInfo();
}

//----------------------------------------------------------------------------
void GEM_SendTxDriveGPICommand(unsigned short command)
{
  geminiNetwork.SendTxDriveGPICommand(command);
}

//----------------------------------------------------------------------------
void GEM_SendGPOWriteCommand(unsigned short command)
{
  geminiNetwork.SendGPOWriteCommand(command);
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// DLL version info

//----------------------------------------------------------------------------
int GEM_GetVStringLen(void)
{
  char ver[50];
  
  #ifdef _WIN32
	  sprintf_s(ver, 48, "Gemini Comms V%s SRDUK Ltd.", cVerNum);
  #else
	  snprintf(ver, 48, "Gemini Comms V%s SRDUK Ltd.", cVerNum);
  #endif

  return (int)(strlen(ver));
}

//----------------------------------------------------------------------------
void GEM_GetVString(char *data, int len)
{
  int i;
  char ver[50];
  
  if (data)
  {
    if (len < 48)
    {
      #ifdef _WIN32
      	sprintf_s(ver, 48, "Gemini Comms V%s SRDUK Ltd.", cVerNum);
      #else
      	snprintf(ver, 48, "Gemini Comms V%s SRDUK Ltd.", cVerNum);
      #endif
    
      for (i = 0; i < (int)strlen(ver); i++)
      {
        data[i] = ver[i];
      }
      
      data[len] = 0;
    }
    else
    {
      data[0] = 0;
    }
  }
}

//----------------------------------------------------------------------------
int GEM_GetVNumStringLen(void)
{
  return (int)(strlen(cVerNum));
}

//----------------------------------------------------------------------------
void GEM_GetVNumString(char *data, int len)
{
  int i;

  if (data)
  {
    if (len >= GEM_GetVNumStringLen())
    {
      for (i = 0; i < GEM_GetVNumStringLen(); i++)
      {
        data[i] = cVerNum[i];
      }
      
      data[len] = 0;
    }
    else
    {
      data[0] = 0;
    }
  }
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// Serial port related commands

//----------------------------------------------------------------------------
int GEM_InitComPort(int comPortNo, unsigned short echoData, 
                                   unsigned short sendToCalling)
{
  bool retValue = geminiNetwork.InitComPort(comPortNo, echoData, sendToCalling);

  if (retValue)
    return 1;
  else
    return 0;
}

//----------------------------------------------------------------------------
int GEM_WriteToComPort(char *data, unsigned short len)
{
  return geminiNetwork.WriteToComPort(data, len);
}

//----------------------------------------------------------------------------
#ifdef _MANAGED
#pragma managed(pop)
#endif
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

