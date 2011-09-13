#ifndef __GEMINICOMMS_H__
#define __GEMINICOMMS_H__

#include "DataTypes.h"
#include "GeminiStructures.h"

#ifdef _WIN32
	#define DllImport extern "C" __declspec( dllimport )
	#define DllExport extern "C" __declspec( dllexport )
	#ifdef BUILDING_GEMINI_COMMS
	  #ifdef BUILDING_SDK
		 #define EXT_GEMINI_COMMS 
	  #else
		 #define EXT_GEMINI_COMMS DllExport
	  #endif
	#else
	  #define EXT_GEMINI_COMMS DllImport
	#endif
#else
	#define EXT_GEMINI_COMMS
#endif

#include "GeminiCommsPublic.h"

// Configuration commands

EXT_GEMINI_COMMS void GEM_StartGeminiNetwork(unsigned short sonarID);
EXT_GEMINI_COMMS void GEM_SetProgressFunction(void (cdecl *FnPtr)(int sType, int retryCount, int packet, int block, int totalBlocks, int otherValue));

EXT_GEMINI_COMMS void GEM_ProgramSonarID(unsigned short sonarID, int useHighFreq);
EXT_GEMINI_COMMS void GEM_ProgramVelocimeterCoeffs(unsigned int CCoeff, unsigned int MCoeff);

// Ping configuration commands
EXT_GEMINI_COMMS void GEM_SetGeminiModFrequency(int modFrequency);

EXT_GEMINI_COMMS unsigned short GEM_GetRXAddress1(void);
EXT_GEMINI_COMMS void GEM_SetRXAddress1(unsigned short rxAddress1);
EXT_GEMINI_COMMS unsigned short GEM_GetRXAddress2(void);
EXT_GEMINI_COMMS void GEM_SetRXAddress2(unsigned short rxAddress2);
EXT_GEMINI_COMMS unsigned short GEM_GetTXAddress1(void);
EXT_GEMINI_COMMS void GEM_SetTXAddress1(unsigned short txAddress1);
EXT_GEMINI_COMMS unsigned short GEM_GetTXAddress2(void);
EXT_GEMINI_COMMS void GEM_SetTXAddress2(unsigned short txAddress2);

EXT_GEMINI_COMMS void GEM_SetRunMode(unsigned short triggerEdge, unsigned short pingMethod);

EXT_GEMINI_COMMS void GEM_SetExtModeDataType(unsigned short dataType);
EXT_GEMINI_COMMS void GEM_SetExtModeDataSize(unsigned short dataSize);
EXT_GEMINI_COMMS void GEM_SetExtModeFocusMode(unsigned short focusMode);
EXT_GEMINI_COMMS void GEM_SetExtModeVelDataEnable(unsigned short velDataEnable);
EXT_GEMINI_COMMS void GEM_SetExtModeScopeEnable(unsigned short scopeEnable);
EXT_GEMINI_COMMS void GEM_SetExtModeSimCQIFlag(unsigned short simCQIFlag);
EXT_GEMINI_COMMS void GEM_SetExtModeSimADCFlag(unsigned short simADCFlag);
EXT_GEMINI_COMMS void GEM_SetExtModeVelDataEnableOnce(unsigned short velDataEnableOnce);

EXT_GEMINI_COMMS void GEM_SetStartRange(unsigned short rangeInLines);

EXT_GEMINI_COMMS void GEM_SetTXAngle(unsigned short txAngle);

EXT_GEMINI_COMMS void GEM_SetMainGain(unsigned short baseGain, unsigned short variableGain);
EXT_GEMINI_COMMS void GEM_SetAutoGain(unsigned short autoGainFlag, unsigned short targetGain);
EXT_GEMINI_COMMS void GEM_SetVelocimeterTXPeriod(unsigned short velocimeterTXPeriod);
EXT_GEMINI_COMMS void GEM_SetVelocimeterTXMark(unsigned short velocimeterTXMark);

EXT_GEMINI_COMMS void GEM_SetFilterBank(unsigned short channelFIRFilterBandwidth);
EXT_GEMINI_COMMS void GEM_SetShadeBank(unsigned short beamformerShadingLevel);
EXT_GEMINI_COMMS void GEM_SetChannelSample(unsigned short channelSample);
EXT_GEMINI_COMMS void GEM_SetVelocimeterGain(unsigned short gain);
EXT_GEMINI_COMMS void GEM_SetVelocimeterTXLength(unsigned char txLength);

// Transmit to sonar commands

EXT_GEMINI_COMMS void GEM_InhibitSonarReboot(void);
EXT_GEMINI_COMMS void GEM_SendRS232(int len, char *data);
EXT_GEMINI_COMMS void GEM_CommandPOST(void);
EXT_GEMINI_COMMS void GEM_CommandProductionTest(bool runProdTest, unsigned long long seedValue);
EXT_GEMINI_COMMS void GEM_AbortProductionTest(void);
EXT_GEMINI_COMMS void GEM_RequestTestResult(void);
EXT_GEMINI_COMMS void GEM_SendGeminiNetworkConfig(void);
EXT_GEMINI_COMMS void GEM_SetNetworkIFGs(unsigned char ethernetIFG, unsigned char VDSLIFG);
EXT_GEMINI_COMMS void GEM_GetNetworkIFGs(unsigned char *ethernetIFG, unsigned char *VDSLIFG);
EXT_GEMINI_COMMS void GEM_SetNetworkAdvertiseGigaBit(unsigned short advertiseGigaBit);
EXT_GEMINI_COMMS void GEM_SetNetworkRetriggerVDSLRateAdaption(unsigned short retriggerVDSLadaption);
EXT_GEMINI_COMMS void GEM_SetNetworkInhibitRecommendedRateAdaption(unsigned short inhibitRecommendedRA);
EXT_GEMINI_COMMS void GEM_SetNetworkEnableAllRateAdaption(unsigned short enableAllRA);
EXT_GEMINI_COMMS void GEM_SetNetworkEnableRAMonitoring(unsigned short enableRAMonitoring);
EXT_GEMINI_COMMS void GEM_SetNetworkRS232BaudRate(unsigned short baudRate);
EXT_GEMINI_COMMS void GEM_SetNetworkConfigToDefaults(void);
EXT_GEMINI_COMMS void GEM_SetNetworkInterPacketGap(unsigned short interPacketGap);
EXT_GEMINI_COMMS void GEM_SetNetworkRS232Routing(unsigned short RS232Routing);
EXT_GEMINI_COMMS unsigned short GEM_GetNetworkInterPacketGap(void);

EXT_GEMINI_COMMS void GEM_MDIOInit(void);
EXT_GEMINI_COMMS void GEM_MDIOAddVDSLReg(unsigned short regAddr, unsigned short value);
EXT_GEMINI_COMMS void GEM_MDIOAddSwitchToEthernet(void);
EXT_GEMINI_COMMS void GEM_MDIOAddSwitchToVDSL(void);
EXT_GEMINI_COMMS void GEM_MDIOSend(void);

// Modifiers and Flash Programming commands

EXT_GEMINI_COMMS void GEM_ProgramFPGA(char *fileName, char *destination);
EXT_GEMINI_COMMS void GEM_AbortFPGAProgramming(void);
EXT_GEMINI_COMMS void GEM_InitCalibrationValues(bool useFiles, char *txFile, char *rxFile);
EXT_GEMINI_COMMS int  GEM_CalculateModifiers(void);
EXT_GEMINI_COMMS void GEM_ProgramModifiers(void);
EXT_GEMINI_COMMS void GEM_AbortModifierProgramming(void);
EXT_GEMINI_COMMS int  GEM_WriteModifiersToFile(char *fileName);
EXT_GEMINI_COMMS int  GEM_ReadModifiersFromFile(char *fileName);
EXT_GEMINI_COMMS void GEM_ReadFromFlash(unsigned int addressToRead);
EXT_GEMINI_COMMS void GEM_WriteToFlash(unsigned int addressToWrite, unsigned short readbackFlag, unsigned char *data);
EXT_GEMINI_COMMS void GEM_ReadFlashInfoBlock(unsigned short blockToRead);
EXT_GEMINI_COMMS void GEM_SendTxDriveGPICommand(unsigned short command);
EXT_GEMINI_COMMS void GEM_SendGPOWriteCommand(unsigned short command);
EXT_GEMINI_COMMS void GEM_SendGPIOReadCommand(void);
EXT_GEMINI_COMMS void GEM_RequestDiagnosticInfo(void);

// Serial port related commands

EXT_GEMINI_COMMS int  GEM_InitComPort(int comPortNo, unsigned short echoData, 
                                                     unsigned short sendToCalling);
EXT_GEMINI_COMMS int  GEM_WriteToComPort(char *data, unsigned short len);
#endif //__GEMINICOMMS_H__

