#ifndef __GEMINICOMMSPUBLIC_H__
#define __GEMINICOMMSPUBLIC_H__

#include "DataTypes.h"
#include "GeminiStructures.h"

#ifdef _WIN32
	#define DllImport extern "C" __declspec( dllimport )
	#define DllExport extern "C" __declspec( dllexport )

	#ifdef BUILDING_GEMINI_COMMS
	  #define PUB_GEMINI_COMMS DllExport
	#else
	  #define PUB_GEMINI_COMMS DllImport
	#endif
#else
	#define PUB_GEMINI_COMMS
#endif

// Configuration commands

PUB_GEMINI_COMMS int  GEM_StartGeminiNetworkWithResult(unsigned short sonarID);
PUB_GEMINI_COMMS void GEM_SetGeminiSoftwareMode(char *softwareMode);
PUB_GEMINI_COMMS void GEM_SetHandlerFunction(void (cdecl *FnPtr)(int eType, int len, char *dataBlock));
PUB_GEMINI_COMMS void GEM_ResetInternalCounters(void);
PUB_GEMINI_COMMS void GEM_StopGeminiNetwork(void);

PUB_GEMINI_COMMS unsigned short GEM_GetSonarID(void);
PUB_GEMINI_COMMS void GEM_SetDLLSonarID(unsigned short sonarID);

PUB_GEMINI_COMMS void GEM_GetAltSonarIPAddress(unsigned char *a1, unsigned char *a2, unsigned char *a3, unsigned char *a4,
                                               unsigned char *s1, unsigned char *s2, unsigned char *s3, unsigned char *s4);
PUB_GEMINI_COMMS void GEM_SetAltSonarIPAddress(unsigned char a1, unsigned char a2, unsigned char a3, unsigned char a4,
                                               unsigned char s1, unsigned char s2, unsigned char s3, unsigned char s4);
PUB_GEMINI_COMMS void GEM_UseAltSonarIPAddress(unsigned char a1, unsigned char a2, unsigned char a3, unsigned char a4,
                                               unsigned char s1, unsigned char s2, unsigned char s3, unsigned char s4);
PUB_GEMINI_COMMS void GEM_TxToAltIPAddress(int useAltIPAddress);

// Ping configuration commands
PUB_GEMINI_COMMS void GEM_AutoPingConfig(float range, unsigned short spreadGain, float sos);
PUB_GEMINI_COMMS void GEM_SetGeminiEvoQuality(unsigned char evoQualitySetting);
PUB_GEMINI_COMMS unsigned short GEM_GetRequestedCompressionFactor(void);

PUB_GEMINI_COMMS void GEM_SetPingMode(unsigned short pingMethod);

PUB_GEMINI_COMMS void GEM_SetExtModeOutOfWaterOverride(unsigned short outOfWaterOverride);
PUB_GEMINI_COMMS void GEM_SetExtModeTDBFlag(unsigned short tdbFlag);

PUB_GEMINI_COMMS void GEM_SetEndRange(unsigned short rangeInLines);

PUB_GEMINI_COMMS void GEM_SetInterPingPeriod(unsigned int periodInMicroSeconds);

PUB_GEMINI_COMMS void GEM_SetTXLength(unsigned short txLength);

PUB_GEMINI_COMMS void GEM_SetMainGainPerCent(unsigned short perCentGain);
PUB_GEMINI_COMMS void GEM_SetMainGainUnits(unsigned short dBGain);
PUB_GEMINI_COMMS void GEM_SetAbsorbGain(unsigned short absorbtionGain);
PUB_GEMINI_COMMS void GEM_SetSoftGain(unsigned int softwareGain);
PUB_GEMINI_COMMS void GEM_SetSpreadGain(unsigned short spreadingGain);
PUB_GEMINI_COMMS void GEM_SetSoftwareGainRamp(unsigned short softwareGainRamp);

PUB_GEMINI_COMMS void GEM_SetSpeedOfSound(unsigned short speedOfSound);
PUB_GEMINI_COMMS void GEM_SetVelocimeterMode(unsigned short gainMode, unsigned short outputMode);
PUB_GEMINI_COMMS void GEM_SetRangeCompression(unsigned short compressionLevel, unsigned short compressionType);
PUB_GEMINI_COMMS void GEM_SetRLEThreshold(unsigned short threshold);
PUB_GEMINI_COMMS void GEM_SetPingToDefaults(void);
PUB_GEMINI_COMMS void GEM_SetStartBeam(unsigned short startBeam);
PUB_GEMINI_COMMS void GEM_SetEndBeam(unsigned short endBeam);

// Transmit to sonar commands

PUB_GEMINI_COMMS void GEM_SendGeminiStayAlive(void);
PUB_GEMINI_COMMS void GEM_SendGeminiPingConfig(void);
PUB_GEMINI_COMMS void GEM_RebootSonar(void);
PUB_GEMINI_COMMS void GEM_RequestRetry(unsigned short type, unsigned short lineNo, unsigned short noOfLines);

PUB_GEMINI_COMMS void GEM_SetVDSLSetting(unsigned short level);

// DLL version info

PUB_GEMINI_COMMS int  GEM_GetVStringLen(void);
PUB_GEMINI_COMMS void GEM_GetVString(char *data, int len);
PUB_GEMINI_COMMS int  GEM_GetVNumStringLen(void);
PUB_GEMINI_COMMS void GEM_GetVNumString(char *data, int len);

#endif //__GEMINICOMMSPUBLIC_H__

