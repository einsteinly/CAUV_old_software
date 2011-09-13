#ifndef __GEMININETWORK_H__
#define __GEMININETWORK_H__

#include "DataTypes.h"
#include <iostream>
#ifdef _WIN32
	#include "winsock2.h"
#else
	#include <pthread.h>
	#include <sys/types.h>
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <netdb.h>
	#include <errno.h>
#endif
#include "GeminiStructures.h"
#include "GeminiDefaultsLimits.h"
#include "GeminiModifiers.h"

#define BUF_SIZE 30

// create linux alternatives for CreateMutex, ReleaseMutex CreateEvent and 
// WaitForSingleObject.
// NOTE: There are no calls to CloseHandle which I would have expected - maybe
// there is a reason so I'll not add in the linux alternative yet (which would be
// pthread_mutex_destroy()).
#ifndef _WIN32
	HANDLE CreateMutex(HANDLE & hMutex, BOOL bInitialOwner, LPCTSTR lpName);
	
	BOOL ReleaseMutex(HANDLE & hMutex);
	
	// NOTE: This only works for MUTEX handles!
	DWORD WaitForSingleObject(HANDLE & hHandle, DWORD dwMilliseconds);
/*	
	// left the windows prototype incase we move to semaphores - they use this stuff.
	HANDLE CreateEvent(HANDLE hEvent, BOOL bManualReset, BOOL bInitialState, LPCTSTR lpName)
	{
		int nResult = pthread_cond_init((pthread_cond_t*)hEvent, NULL);
		if(nResult==0)
		{
			return hEvent;
		}
		else
		{
			return NULL;
		}
	}
	
	BOOL SetEvent(HANDLE hEvent)
	{
		// return pthread_cond_broadcast((pthread_cond_t*)hEvent);
		return pthread_cond_signal((pthread_cond_t*)hEvent);
	}
*/
#endif

class CStatusMutex
{
public:
	CGemStatusPacket m_statusPacket;
	HANDLE           m_mutex;
	BOOL             m_bLocked;
	
	#ifndef _WIN32
		pthread_mutex_t m_hPTHRMutex;
	#endif

	CStatusMutex()
	{
	#ifdef _WIN32
   	m_mutex = CreateMutex(NULL,  // default security attributes
       	                   FALSE, // initially not owned
       	                   NULL); // unnamed mutex
   #else
   	m_mutex = &m_hPTHRMutex;
   	CreateMutex(m_mutex, FALSE, NULL);
   #endif
		m_bLocked = false;
	}
};

class CTgtImgMutex
{
public:	
//	CTgtImg m_tgtImg;
	HANDLE  m_mutex;
	BOOL    m_bLocked;
	
	#ifndef _WIN32
		pthread_mutex_t m_hPTHRMutex;
	#endif

  unsigned char   m_RLEThresholdUsed;
  unsigned char   m_rangeCompressionUsed;
  unsigned char   m_outputLineMultiplier;
  unsigned short  m_extModeUsed;
  unsigned short  m_numBeamsUsed;
  unsigned short  m_numRangesUsed;
  unsigned short *m_dataPtr;
  signed short   *m_retryQueuePtr;
  int             m_lastRxLineNo;
  bool            m_pingStarted;
  bool            m_queuedTailRetry;
  unsigned short  m_firstPassRetries;
  unsigned short  m_secondPassRetries;
  unsigned short  m_tailRetries;
  
	CTgtImgMutex()
	{
    #ifdef _WIN32
   	m_mutex = CreateMutex(NULL,  // default security attributes
       	                   FALSE, // initially not owned
       	                   NULL); // unnamed mutex
   #else
   	m_mutex = &m_hPTHRMutex;
   	CreateMutex(m_mutex, FALSE, NULL);
   #endif

		m_bLocked = false;
		
		m_dataPtr       = NULL;
		m_retryQueuePtr = NULL;
		m_pingStarted   = false;
	}
	
	~CTgtImgMutex()
	{
	  if (m_dataPtr != NULL)
	  {
	    free(m_dataPtr);
	    m_dataPtr = NULL;
    }
    
	  if (m_retryQueuePtr != NULL)
	  {
	    free(m_retryQueuePtr);
	    m_retryQueuePtr = NULL;
	  }
  }	
};

class CFlashResultMutex
{
public:	
  CGemFlashResult   m_gemFlashResult;
	HANDLE            m_mutex;
	BOOL              m_bLocked;
	
	#ifndef _WIN32
		pthread_mutex_t m_hPTHRMutex;
	#endif

	CFlashResultMutex()
	{
    #ifdef _WIN32
   	m_mutex = CreateMutex(NULL,  // default security attributes
       	                   FALSE, // initially not owned
       	                   NULL); // unnamed mutex
   #else
   	m_mutex = &m_hPTHRMutex;
   	CreateMutex(m_mutex, FALSE, NULL);
   #endif

		m_bLocked = false;
	}
};

class CFlashReadbackMutex
{
public:	
  CGemFlashReadback m_gemFlashReadback;
	HANDLE            m_mutex;
	BOOL              m_bLocked;

	#ifndef _WIN32
		pthread_mutex_t m_hPTHRMutex;
	#endif

	CFlashReadbackMutex()
	{
    #ifdef _WIN32
   	m_mutex = CreateMutex(NULL,  // default security attributes
       	                   FALSE, // initially not owned
       	                   NULL); // unnamed mutex
   #else
   	m_mutex = &m_hPTHRMutex;
   	CreateMutex(m_mutex, FALSE, NULL);
   #endif

		m_bLocked = false;
	}
};

class CAcknowledgeMutex
{
public:	
  CGemAcknowledge m_gemAcknowledge;
	HANDLE          m_mutex;
	BOOL            m_bLocked;
	
	#ifndef _WIN32
		pthread_mutex_t m_hPTHRMutex;
	#endif

	CAcknowledgeMutex()
	{
    #ifdef _WIN32
   	m_mutex = CreateMutex(NULL,  // default security attributes
       	                   FALSE, // initially not owned
       	                   NULL); // unnamed mutex
   #else
   	m_mutex = &m_hPTHRMutex;
   	CreateMutex(m_mutex, FALSE, NULL);
   #endif

		m_bLocked = false;
	}
};

class CVelocimeterMutex
{
public:	
  CGemVelocimeterData m_gemVelocimeterData;
	HANDLE              m_mutex;
	BOOL                m_bLocked;
	
	#ifndef _WIN32
		pthread_mutex_t m_hPTHRMutex;
	#endif

	CVelocimeterMutex()
	{
   #ifdef _WIN32
   	m_mutex = CreateMutex(NULL,  // default security attributes
       	                   FALSE, // initially not owned
       	                   NULL); // unnamed mutex
   #else
   	m_mutex = &m_hPTHRMutex;
   	CreateMutex(m_mutex, FALSE, NULL);
   #endif

		m_bLocked = false;
	}
};

class CSerialMutex
{
public:	
  CGemRS232RXPacket m_gemRS232RXPacket;
	HANDLE            m_mutex;
	BOOL              m_bLocked;
	
	#ifndef _WIN32
		pthread_mutex_t m_hPTHRMutex;
	#endif

	CSerialMutex()
	{
    #ifdef _WIN32
   	m_mutex = CreateMutex(NULL,  // default security attributes
       	                   FALSE, // initially not owned
       	                   NULL); // unnamed mutex
   #else
   	m_mutex = &m_hPTHRMutex;
   	CreateMutex(m_mutex, FALSE, NULL);
   #endif
		m_bLocked = false;
	}
};

class CTestResultMutex
{
public:	
  CGemTestResult  m_gemTestResult;
	HANDLE          m_mutex;
	BOOL            m_bLocked;
	
	#ifndef _WIN32
		pthread_mutex_t m_hPTHRMutex;
	#endif

	CTestResultMutex()
	{
    #ifdef _WIN32
   	m_mutex = CreateMutex(NULL,  // default security attributes
       	                   FALSE, // initially not owned
       	                   NULL); // unnamed mutex
   #else
   	m_mutex = &m_hPTHRMutex;
   	CreateMutex(m_mutex, FALSE, NULL);
   #endif

		m_bLocked = false;
	}
};

class CMDIOAckMutex
{
public:	
  CGemMDIOAck   m_gemGPIOAckPacket;
	HANDLE        m_mutex;
	BOOL          m_bLocked;

	#ifndef _WIN32
		pthread_mutex_t m_hPTHRMutex;
	#endif

	CMDIOAckMutex()
	{
    #ifdef _WIN32
   	m_mutex = CreateMutex(NULL,  // default security attributes
       	                   FALSE, // initially not owned
       	                   NULL); // unnamed mutex
   #else
   	m_mutex = &m_hPTHRMutex;
   	CreateMutex(m_mutex, FALSE, NULL);
   #endif

		m_bLocked = false;
	}
};

class CGPIDataMutex
{
public:	
  CGemGPIData   m_gemGPIDataPacket;
	HANDLE        m_mutex;
	BOOL          m_bLocked;
	
	#ifndef _WIN32
		pthread_mutex_t m_hPTHRMutex;
	#endif

	CGPIDataMutex()
	{
    #ifdef _WIN32
   	m_mutex = CreateMutex(NULL,  // default security attributes
       	                   FALSE, // initially not owned
       	                   NULL); // unnamed mutex
   #else
   	m_mutex = &m_hPTHRMutex;
   	CreateMutex(m_mutex, FALSE, NULL);
   #endif

		m_bLocked = false;
	}
};

class CDiagDataMutex
{
public:	
  CGemDiagData  m_gemDiagDataPacket;
	HANDLE        m_mutex;
	BOOL          m_bLocked;
	
	#ifndef _WIN32
		pthread_mutex_t m_hPTHRMutex;
	#endif

	CDiagDataMutex()
	{
    #ifdef _WIN32
   	m_mutex = CreateMutex(NULL,  // default security attributes
       	                   FALSE, // initially not owned
       	                   NULL); // unnamed mutex
   #else
   	m_mutex = &m_hPTHRMutex;
   	CreateMutex(m_mutex, FALSE, NULL);
   #endif

		m_bLocked = false;
	}
};

class CGemPingConfig: public CGemNetMsg
{
public:	
	unsigned short m_rxIdent1;
	unsigned short m_txIdent1;
	unsigned short m_rxIdent2;
	unsigned short m_txIdent2;
	unsigned short m_runMode;
	unsigned short m_extModeFlags;
	unsigned short m_startRange;
	unsigned short m_endRange;
	unsigned int   m_interping;
	unsigned short m_txLength;
	unsigned short m_txAngle;
	unsigned short m_mainGain;
	unsigned short m_absorption;
	unsigned int   m_softwareGain;
	unsigned short m_spreading;
	unsigned short m_autoGain;
	unsigned short m_filterBank;
	unsigned short m_shadeBank;
	unsigned short m_spdSnd;	
	unsigned short m_velMode;
	unsigned short m_rngComp;
	unsigned short m_rleThreshold;
	unsigned short m_chanSamp;	
	unsigned short m_velGain;
	unsigned char  m_velTxLength;
	unsigned char  m_txMark;
	unsigned short m_imageMode;
	unsigned char  m_powerManagement;
	unsigned char  m_softwareGainRamp;
	unsigned char  m_velocimeterTXPeriod;
	unsigned char  m_velocimeterTXMark;

	CGemPingConfig()
	{
		m_head.m_type   = 0x21;

    InitPingConfig();
  }
  
  void InitPingConfig(void)
  {		
		m_rxIdent1            = 0;
		m_txIdent1            = 0;
		m_rxIdent2            = 0;
		m_txIdent2            = 0;
		m_runMode             = 0;
		m_extModeFlags        = 4;            // Default - 8 bit beamformed magnitude
		m_startRange          = 0;
		m_endRange            = 33;           // Default -
    m_interping           = 200000;       // Default - 
    m_txLength            = 4;            // Default - 
    m_txAngle             = 0;
 		m_mainGain            = 1000;
		m_absorption          = geminiPingAbsorbtionGainDefault;
		m_softwareGain        = 0x0000ff00;   // Default - 
		m_spreading           = geminiPingSpeadingGainDefault;
		m_autoGain            = 0;
		m_filterBank          = 0;
		m_shadeBank           = geminiPingShadeBankDefault;
    m_spdSnd              = 15000;        // Default - 1500 m/s
		m_velMode             = 1;            // Default -
		m_rngComp             = 0;
		m_rleThreshold        = 0;
		m_chanSamp            = 0;		
		m_velGain             = 0;
		m_velTxLength         = geminiVelocimeterTXLengthDefault;
		m_txMark              = geminiTXMarkDefault;
    m_imageMode           = 0;
    m_powerManagement     = geminiPingPowerManagementDefault;
    m_softwareGainRamp    = geminiSoftwareGainRampDefault;
	  m_velocimeterTXPeriod = geminiVelocimeterTXPeriodDefault;
	  m_velocimeterTXMark   = geminiVelocimeterTXMarkDefault;
	}
};

class CGemStatusRequest: public CGemNetMsg
{
public:
	CGemStatusRequest()
	{
		m_head.m_type   = 0x20;
	}
};

class CGemFlashWritePacket: public CGemNetMsg
{
public:
	unsigned int   m_addr;
	unsigned short m_flags;
	unsigned short m_spare;
	unsigned char  m_data[1024];

	CGemFlashWritePacket()
	{
		m_head.m_type = 0x22; // for a flash write
		m_addr        = 0;
		m_flags       = 0;
		m_spare       = 0;
	}
};

class CGemFlashReadPacket: public CGemNetMsg
{
public:
	unsigned int   m_addr;

	CGemFlashReadPacket()
	{
		m_head.m_type = 0x23; // for a flash read
		m_addr        = 0;
	}
};

class CGemInfoWritePacket: public CGemNetMsg
{
public:	
	unsigned int   m_addr;
	unsigned short m_flags;
	unsigned short m_spare;

	// These three variables are read from flash by the FPGA on power up and 
	// should stay at the head of the data block

	CGemInfoWritePacket()
	{
		m_head.m_type = 0x24; // for a flash info packet write
		m_addr        = 0;
		m_flags       = 0;
		m_spare       = 0;
	}
};

//Info Packet for info bank 0
//This is the Sonar ID bank that is not writable by customer
class CGemInfoPacketBank0 : public CGemInfoWritePacket 
{
public:
	unsigned int m_sonarID;
	unsigned int m_transducerFreq;
	unsigned int m_dummy;

	CGemInfoPacketBank0()
	{
		m_addr   = 0;
		m_flags  = 0;
		m_spare  = 0;
		m_dummy  = 0;
	}
};

//Info Packet for info bank 1
//This contains the velocimeter coefficients
class CGemInfoPacketBank1 : public CGemInfoWritePacket 
{
public:
	unsigned int m_velMCoeff;
	int		       m_velCCoeff;
	unsigned int m_dummy[32];

	CGemInfoPacketBank1()
	{
		m_addr  = 1;
		m_flags = 0;
		m_spare = 0;
	}
};

//Info Packet for info bank 2
//This contains the user modifiable parameters
class CGemInfoPacketBank2 : public CGemInfoWritePacket 
{
public:
	unsigned int m_alt_ip;
	unsigned int m_subnet_mask;
	unsigned int dummy[32];

	CGemInfoPacketBank2()
	{
		m_addr        = 2;
		m_flags       = 0;
		m_subnet_mask = 0;
		m_spare       = 0;
	}
};

class CGemInfoReadPacket: public CGemNetMsg
{
public:
	unsigned int   m_blockNo;

	CGemInfoReadPacket()
	{
		m_head.m_type = 0x25; // for a flash read
		m_blockNo     = 0;
	}
};

class CGemRS232TXPacket : public CGemNetMsg
{
public:
  unsigned char m_length;
  unsigned char m_data[1024];

	CGemRS232TXPacket()
	{
		m_head.m_type = 0x2a;
		m_length      = 0;
	}
};

class CGemRebootPacket: public CGemNetMsg
{
public:
	unsigned char  m_rebootControl;
	unsigned char  m_spare1;
	unsigned short m_spare2;
	
	CGemRebootPacket()
	{
		m_head.m_type   = 0x2b; // for a reboot command
		m_rebootControl = 0;
		m_spare1        = 0;
		m_spare2        = 0;
	}
};

class CGemResultRequestPacket: public CGemNetMsg
{
public:
	unsigned int m_dummy;
	
	CGemResultRequestPacket()
	{
		m_head.m_type = 0x2c; // for a result request command
		m_dummy       = 0;
	}
};

class CGemProdTestPacket: public CGemNetMsg
{
public:
	unsigned long long m_PRBSSeed;
	unsigned char      m_flags;
	unsigned char      m_spare1;
	unsigned short     m_spare2;
	
	CGemProdTestPacket()
	{
		m_head.m_type = 0x2d; // for a production test command
    m_PRBSSeed    = 0;
		m_flags       = 0;
		m_spare1      = 0;
		m_spare2      = 0;
	}
};

class CGemPOSTPacket: public CGemNetMsg
{
public:
	unsigned int m_dummy;
	
	CGemPOSTPacket()
	{
		m_head.m_type = 0x2e; // for a POST command
		m_dummy = 0;
	}
};

class CGemProdTestAbortPacket: public CGemNetMsg
{
public:
	unsigned int m_dummy;
	
	CGemProdTestAbortPacket()
	{
		m_head.m_type = 0x2f; // for a production test abort command
		m_dummy = 0;
	}
};

class CGemRetryRequestPacket: public CGemNetMsg
{
public:
	unsigned short m_type;
	unsigned short m_lineNo;
	unsigned short m_noOfLines;
  unsigned short m_spare;
	
	CGemRetryRequestPacket()
	{
		m_head.m_type = 0x30; // for a retry request command
		m_type        = 0;
		m_lineNo      = 0;
		m_noOfLines   = 0;
		m_spare       = 0;
	}
};

#pragma pack (push,1)

class CGemNetworkConfigPacket: public CGemNetMsg
{
public:
	unsigned char  m_ethernetIFG;
	unsigned char  m_VDSLIFG;
	unsigned char  m_advertiseGigaBit;
	unsigned char  m_retriggerRateAdaption;
	unsigned short m_baudRate;
	unsigned short m_RS232PollPeriod;
	unsigned short m_interPacketGap;
	unsigned short m_RS232Routing;
  	
	CGemNetworkConfigPacket()
	{
		m_head.m_type           = 0x31; // for a network config command
	
    InitNetworkConfig();
	}
	
	void InitNetworkConfig(void)
	{
		m_ethernetIFG           = 15;
		m_VDSLIFG               = 15;
		m_advertiseGigaBit      = 0;
		m_retriggerRateAdaption = 0;
		m_baudRate              = geminiBaud9600;
		m_RS232PollPeriod       = geminiPollPeriodFor9600Baud;
    m_interPacketGap        = 0;
    m_RS232Routing          = 0;
	}
};

#pragma pack (pop)

#define MDIO_WRITE  0x53
#define MDIO_ETH    0x55
#define MDIO_VDSL   0x56

class CGemMDIOPacket: public CGemNetMsg
{
public:
  unsigned short m_length;
  unsigned short m_data[1024];
  
  CGemMDIOPacket()
  {
		m_head.m_type = 0x32;
		Clear();
  }

  void Clear(void)
  {
    m_length = 2;
    
    for (int i = 0; i < 1024; i++)
    {
      m_data[i] = 0;
    }
  }
  
  void AddVDSLRegWriteCommand(unsigned short regAddr, unsigned short value)
  {
    AddWriteCommand(0x0010, regAddr);
    AddWriteCommand(0x0011, ((value << 8) | 0x0001));  
  }
  
  void AddSwitchEthernetCommand(void)
  {
    int base = (m_length - 2) / 2;
    
    m_data[base]      = MDIO_ETH;
    m_length         += 2;
  }
  
  void AddSwitchVDSLCommand(void)
  {
    int base = (m_length - 2) / 2;
    
    m_data[base]      = MDIO_VDSL;
    m_length         += 2;
  }
  
  int GetLength(void)
  {
    return (m_length + 8);
  }
  
private:

  void AddWriteCommand(unsigned short addr, unsigned short value)
  {
    int base = (m_length - 2) / 2;
    
    m_data[base]      = MDIO_WRITE;
    m_data[base + 1]  = addr;
    m_data[base + 2]  = value;
    m_length         += 6;
  }
};

class CGemTxDriveGPIPacket: public CGemNetMsg
{
public:
	unsigned short m_txDrv;
	unsigned short m_dummy;
	
	CGemTxDriveGPIPacket()
	{
		m_head.m_type = 0x33; // for a TX drive and GPI test command
    m_txDrv = 0;
		m_dummy = 0;
	}
};

class CGemGPOWritePacket: public CGemNetMsg
{
public:
	unsigned short m_gpo;
	unsigned short m_dummy;
	
	CGemGPOWritePacket()
	{
		m_head.m_type = 0x34; // for a GPO write command
    m_gpo   = 0;
		m_dummy = 0;
	}
};

class CGemGPIOReadPacket: public CGemNetMsg
{
public:
  unsigned int m_dummy;
	
	CGemGPIOReadPacket()
	{
		m_head.m_type = 0x35; // for a GPIO read command
		m_dummy = 0;
	}
};

class CGemDiagReadPacket: public CGemNetMsg
{
public:
  unsigned int m_dummy;
	
	CGemDiagReadPacket()
	{
		m_head.m_type = 0x36; // for a diagnostic read command
		m_dummy = 0;
	}
};

class CGemInternalPingLine: public CGemNetMsg
{
public:	
	unsigned char  m_gain;
	unsigned char  m_pingID;
	unsigned short m_lineID;
	unsigned short m_scale;
	unsigned short m_lineInfo;
  unsigned char  m_data[512];

	CGemInternalPingLine()
	{
		m_head.m_type = 0x42;
		m_gain        = 0;
		m_pingID      = 0;
		m_lineID      = 0;
		m_scale       = 0;
		m_lineInfo    = 0;
	}	
};

class CGeminiNetwork
{
public:
  CGeminiNetwork();
  ~CGeminiNetwork();

  bool Init();
  void ResetCounters();
  void Stop();

	bool CreateSocketRX();
	bool CreateSocketTX(bool InitWSA);
	void DestroySocketRX();
	void DestroySocketTX();
	bool Reconnect();

  bool InitComPort(int comPortNo, unsigned short echoData, 
                                  unsigned short sendToCalling);
  int  WriteToComPort(char *data, unsigned short len);
  
  void TimerTick5mS(void);
  void TimerTick50mS(void);

  void SetHandlerFunction(void (cdecl *FnPtr)(int eType, int len, char *dataBlock));
  void SetProgressFunction(void (cdecl *FnPtr)(int sType, int retryCount, int packet, 
                                               int block, int totalBlocks, int otherValue));

  void SendMsg(CGemNetMsg* pMsg, int size);
  void SendMsg(CGemNetMsg* pMsg, int size, bool useAltIPAddress);

  unsigned short GetSonarID(void);
  void SetDLLSonarID(unsigned short sonarID);
  void ProgramSonarID(unsigned short sonarID, int useHighFreq);

  void SetAltSonarIPAddress(unsigned char a1, unsigned char a2, unsigned char a3, unsigned char a4,
                            unsigned char s1, unsigned char s2, unsigned char s3, unsigned char s4);
  void GetAltSonarIPAddress(unsigned char *a1, unsigned char *a2, unsigned char *a3, unsigned char *a4,
                            unsigned char *s1, unsigned char *s2, unsigned char *s3, unsigned char *s4);
  void UseAltSonarIPAddress(unsigned char a1, unsigned char a2, unsigned char a3, unsigned char a4,
                            unsigned char s1, unsigned char s2, unsigned char s3, unsigned char s4);
  void TxToAltIPAddress(bool useAltIPAddress);

  void RebootSonar(void);
  void InhibitSonarReboot(void);

  void SendRS232Data(int len, char *data);

  void CommandPOST(void);
  void CommandProductionTest(bool runProdTest, unsigned long long seedValue);
  void AbortProductionTest(void);
  void RequestTestResult(void);
  
  void InitCalibrationValues(bool useFiles, char *txFile, char *rxFile);
  bool CalculateModifiers(void);
  void ProgramModifiers(void);
  void AbortModifierProgramming(void);
  void ProgramModifiersCode(void);

  int  DownloadBitFile(char *fileName);
  void ProgramFPGA(char *fileName, char *destination);
  void AbortFPGAProgramming(void);
  void ProgramFPGACode(void);
  void ReadFromFlash(unsigned int addressToRead);
  void WriteToFlash(unsigned int addressToWrite, unsigned short readbackFlag, unsigned char *data);
  void ReadFlashInfoBlock(unsigned short blockToRead);
  void SendTxDriveGPICommand(unsigned short command);
  void SendGPOWriteCommand(unsigned short command);
  void SendGPIOReadCommand(void);
  void RequestDiagnosticInfo(void);

	void RxStatus(char* pData);
	void RxPingHead(char* pData);
	void RxPingLine(int len, char* pData);
	void RxPingTail(char* pData);
	void RxFlashResultPacket(char* pData);
	void RxFlashReadbackPacket(char* pData);
	void RxScopePacket(char* pData);	
	void RxVelPacket(char* pData);
	void RxSerialPacket(int len, char* pData);
  void RxTestResultPacket(char* pData);
  void RxAcknowledgePacket(char* pData);
  void RxMDIOAckPacket(char* pData);  
  void RxGPIDataPacket(char* pData);  
  void RxDiagDataPacket(char* pData);  
	void RxUnknown(char* pData);

  void RequestRetry(unsigned short type, unsigned short lineNo, unsigned short noOfLines);
  void SendNetworkConfig(void);
  void SendMDIOPacket(void);
  
  void IPHelperFunction(int codeToAction);

  void ProgramVelocimeterCoeffs(unsigned int CCoeff, unsigned int MCoeff);

  int  WriteModifiersToFile(char *fileName);
  int  ReadModifiersFromFile(char *fileName);

	bool              m_sockRXConnected;
	bool              m_sockTXConnected;
  SOCKET            m_socketRX;
  SOCKET            m_socketTX;

  bool              m_keepSocketTaskRunning;
  bool              m_keepTimerTaskRunning;

  bool              m_socketTaskRunning;
  bool              m_serialTaskRunning; 
  bool              m_serialEchoData;
  bool              m_serialSendToCalling;
  bool              m_modifiersCalculated;
  bool              m_abortModProgramming;
  bool              m_modProgrammingInProgress;
  bool              m_abortFPGAProgramming;
  bool              m_FPGAProgrammingInProgress;

  unsigned short    m_geminiCommsMode;

  CGeminiModifiers  m_geminiModifiers;
	CGemFlashResult   m_flashResult;
	CGemFlashReadback m_flashReadback;
	
	unsigned short    m_pingStartBeam;
  unsigned short    m_pingEndBeam;

  unsigned char     m_geminiEvoRangeCompression;

  CGemNetworkConfigPacket m_geminiNetworkConfig;
  unsigned short m_geminiCompressionLines[8];
  unsigned short m_geminiCompressionFactorUsed;
  unsigned long  m_packetCount;
  unsigned long  m_recvErrorCount;
  unsigned long  m_generalCount;
	
	CGemMDIOPacket m_geminiMDIOPacket;
	
  void ConfigureVDSLToCurrentDefault(void);
  
  unsigned short m_VDSLSettingsToUse;

private:

  void (cdecl *m_dataHandlerFunction)(int eType, int len, char *dataBlock);
  void (cdecl *m_progressCallback)(int sType, int retryCount, int packet, int block, int totalBlocks, int otherValue);

	CStatusMutex          m_statusBuf[BUF_SIZE];
	CTgtImgMutex          m_tgtImgBuf[BUF_SIZE];
  CFlashResultMutex     m_flashResultBuf[BUF_SIZE];
  CFlashReadbackMutex   m_flashReadbackBuf[BUF_SIZE];
	CAcknowledgeMutex     m_ackBuf[BUF_SIZE];
	CVelocimeterMutex     m_velocimeterBuf[BUF_SIZE];
	CSerialMutex          m_serialBuf[BUF_SIZE];
  CTestResultMutex      m_testResultBuf[BUF_SIZE];
  CMDIOAckMutex         m_MDIOAckBuf[BUF_SIZE];
  CGPIDataMutex         m_GPIDataBuf[BUF_SIZE];
  CDiagDataMutex        m_diagDataBuf[BUF_SIZE];
  
	int m_statusIpPtr;
	int m_statusOpPtr;
	int m_tgtImgIpPtr;
	int m_tgtImgOpPtr;
	int m_flashResultIpPtr;
	int m_flashResultOpPtr;
	int m_flashReadbackIpPtr;
	int m_flashReadbackOpPtr;
	int m_ackIpPtr;
	int m_ackOpPtr;
  int m_velocimeterIpPtr;
  int m_velocimeterOpPtr;
	int m_serialIpPtr;
	int m_serialOpPtr;
  int m_testResultIpPtr;
  int m_testResultOpPtr;
  int m_MDIOAckIpPtr;
  int m_MDIOAckOpPtr;
  int m_GPIDataIpPtr;
  int m_GPIDataOpPtr;
  int m_diagDataIpPtr;
  int m_diagDataOpPtr;
  
  int m_commsInterMessageGap;
  int m_fpgaBlockCount;
      
  unsigned short m_sonarID;

  bool           m_useAltIPAddress;
  
  unsigned char  m_altIPAddr1;
  unsigned char  m_altIPAddr2;
  unsigned char  m_altIPAddr3;
  unsigned char  m_altIPAddr4;

  unsigned char  m_geminiSubnetMask1;
  unsigned char  m_geminiSubnetMask2;
  unsigned char  m_geminiSubnetMask3;
  unsigned char  m_geminiSubnetMask4;
  
  unsigned char  m_flashProgRetryCount;

  bool m_serialPortOpened;
  bool m_programMain;
  bool m_programBoot;
  
  CGemRS232TXPacket m_geminiRS232Tx;
  
  CGeminiFlashBlock       m_FPGAFlashBlocks[2];

  void ConfigureVDSL(unsigned short DSDataRateMBS, unsigned short USDataRateMBS,
                     unsigned short DSSNR1,        unsigned short DSSNR2,
                     unsigned short USSNR1,        unsigned short USSNR2,
                     unsigned short DSInterleave,  unsigned short USInterleave);
};
#endif // __GEMININETWORK_H__

