// Implementation of CGeminiNetwork class

#ifdef _WIN32
	#include "stdafx.h"
	#include "winsock2.h"
	#include "process.h"
	#include "AtlBase.h"
	#include "AtlConv.h"
	#include <iphlpapi.h>
	#include "Serial.h"
#endif

#include "GeminiNetwork.h"
#include <cstdio>
#include <cstring>
#include <unistd.h>


#pragma pack (push, 1)

#define PRESET_IP_MSB 10
#define PRESET_IP_SB3 61
#define PRESET_IP_SB2 19
#define PRESET_IP_LSB 200

struct TIMESTAMP_DATA
{
	unsigned int   timestampL;
	unsigned short timestampM;
	unsigned char  timestampH;
	unsigned char  data;
};

union TIMESTAMP_UNION
{
  unsigned char   timestampBytes[8];
  TIMESTAMP_DATA  timestampData;
};

#pragma pack (pop)

/* NO RS232 IN LINUX JUST NOW*/
#ifdef _WIN32
	CSerial   serial;
#endif

// Events (global)

#ifdef _WIN32
	HANDLE hFlashResultReceived   = NULL;
	HANDLE hFlashReadbackReceived = NULL;  
#endif

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// Code launched in a separate thread to handle receipt of data from sonar

//----------------------------------------------------------------------------
void RXDATA(void *pClassPtr)
{
  CGeminiNetwork *pGeminiNetwork = (CGeminiNetwork *)pClassPtr;

  char buf[2048]; // More than enough to accomodate non-jumbo packets

  while (pGeminiNetwork->m_keepSocketTaskRunning)
	{
		if (pGeminiNetwork->m_sockRXConnected)
		{
			// Try to rx data        
			int ret = recv(pGeminiNetwork->m_socketRX, buf, 2048, 0);

      if (ret == SOCKET_ERROR)
      {
      	int error;
      	#ifdef _WIN32
		      error = WSAGetLastError();
		   	WSASetLastError(0); 
		   #else
		   	error = errno;
		   #endif

			  pGeminiNetwork->m_recvErrorCount++;
      }
      else
      {
			  int packetType = buf[0];
			  
			  pGeminiNetwork->m_packetCount++;
			  
			  switch (packetType)
			  {
				  case 0x40 : 
				    pGeminiNetwork->RxStatus(buf); 
				    break;

				  case 0x41 : 
				    pGeminiNetwork->RxPingHead(buf); 
				    break;

				  case 0x42 :
				    pGeminiNetwork->RxPingLine(ret, buf); 
				    break; 

				  case 0x43 :  
				    pGeminiNetwork->RxPingTail(buf); 
				    break;

				  case 0x44 : 
				    pGeminiNetwork->RxFlashResultPacket(buf); 
				    break;

				  case 0x45 : 
				    pGeminiNetwork->RxFlashReadbackPacket(buf); 
				    break;

				  case 0x46 : 
				    pGeminiNetwork->RxScopePacket(buf); 
				    break;

				  case 0x47 : 
				    pGeminiNetwork->RxVelPacket(buf); 
				    break;

				  case 0x48 : 
				    pGeminiNetwork->RxSerialPacket(ret, buf); 
				    break;

				  case 0x49 : 
				    pGeminiNetwork->RxAcknowledgePacket(buf); 
				    break;

          case 0x4A:
				    pGeminiNetwork->RxTestResultPacket(buf); 
				    break;

          case 0x4B:
				    pGeminiNetwork->RxMDIOAckPacket(buf); 
				    break;

          case 0x4C:
				    pGeminiNetwork->RxGPIDataPacket(buf); 
				    break;

          case 0x4D:
				    pGeminiNetwork->RxDiagDataPacket(buf); 
				    break;

				  default : 
				    pGeminiNetwork->RxUnknown(buf);
				    break;
        }
			}
		}
	}
	
  pGeminiNetwork->m_socketTaskRunning = false;
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// Code launched in a separate thread to create a timer ticks every 5ms and 50mS

//----------------------------------------------------------------------------
void TIMERTICK(void *pClassPtr)
{
  CGeminiNetwork* pGeminiNetwork = (CGeminiNetwork*) pClassPtr;	

  int counterFive  = 0;
  int counterFifty = 0;

  while (pGeminiNetwork->m_keepTimerTaskRunning)
  {
  	#ifdef _WIN32
  		Sleep(1);
  	#else
  		usleep(1000);
  	#endif

    // Generate a 5mS timer tick and process it
    
    counterFive++;
    
    if (counterFive == 5)
    {
      pGeminiNetwork->TimerTick5mS();
      counterFive = 0;
    }

    // Generate a 50mS timer tick and process it
    
    counterFifty++;
    
    if (counterFifty == 50)
    {
      pGeminiNetwork->TimerTick50mS();
      counterFifty = 0;
    }
  }
  
  pGeminiNetwork->m_serialTaskRunning = false;
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// Code launched in a separate thread to detect if the IP address has been changed
// The code has been written on the expectation that the DLL will be closed and
// reopened when the IP address has been changed

//----------------------------------------------------------------------------
void IPHELPER(void *pClassPtr)
{
// NOTE: Not implemented this in linux yet as there is not a straight mapping
// of what is happening. We should be able to use netlink (see man 7 netlink).
#ifdef _WIN32
  CGeminiNetwork* pGeminiNetwork = (CGeminiNetwork*) pClassPtr;	

  OVERLAPPED overlap;
  DWORD ret;
	
  HANDLE hand = WSACreateEvent();
  overlap.hEvent = WSACreateEvent();

  ret = NotifyAddrChange(&hand, &overlap);

  if (ret != NO_ERROR)
  {
    if (WSAGetLastError() != WSA_IO_PENDING)
    {
      pGeminiNetwork->IPHelperFunction(0);
      return;
    }
  }

  if (WaitForSingleObject(overlap.hEvent, INFINITE) == WAIT_OBJECT_0)
  {
    pGeminiNetwork->IPHelperFunction(1);
  }
#endif
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// Initialisation and configuration 

//----------------------------------------------------------------------------
CGeminiNetwork::CGeminiNetwork()
{
  m_dataHandlerFunction         = NULL;
  m_progressCallback            = NULL;
  
	m_sockRXConnected             = false;
	m_sockTXConnected             = false;
  m_socketRX                    = NULL;
  m_socketTX                    = NULL;

	m_statusIpPtr                 = 0;
	m_statusOpPtr                 = 0;
	m_tgtImgIpPtr                 = 0;
	m_tgtImgOpPtr                 = 0;
	m_flashResultIpPtr            = 0;
	m_flashResultOpPtr            = 0;
	m_flashReadbackIpPtr          = 0;
	m_flashReadbackOpPtr          = 0;
	m_ackIpPtr                    = 0;
	m_ackOpPtr                    = 0;
	m_velocimeterIpPtr            = 0;
	m_velocimeterOpPtr            = 0;
	m_serialIpPtr                 = 0;
	m_serialOpPtr                 = 0;
  m_testResultIpPtr             = 0;
  m_testResultOpPtr             = 0;
  m_MDIOAckIpPtr                = 0;
  m_MDIOAckOpPtr                = 0;
  m_GPIDataIpPtr                = 0;
  m_GPIDataOpPtr                = 0;
  m_diagDataIpPtr               = 0;
  m_diagDataOpPtr               = 0;
  
  m_sonarID                     = 0;

  m_useAltIPAddress             = false;

  m_altIPAddr1                  = 0;
  m_altIPAddr2                  = 0;
  m_altIPAddr3                  = 0;
  m_altIPAddr4                  = 0;
  
  m_keepSocketTaskRunning       = true;
  m_keepTimerTaskRunning        = true;

  m_socketTaskRunning           = true;
  m_serialTaskRunning           = true;
  m_modifiersCalculated         = false;

  m_geminiCommsMode             = geminiCommsModeDefault;

  m_serialEchoData              = false;
  m_serialSendToCalling         = false;
#ifdef _WIN32
	hFlashResultReceived          = CreateEvent(NULL, FALSE, FALSE, NULL);
	hFlashReadbackReceived        = CreateEvent(NULL, FALSE, FALSE, NULL);
#endif
	
	m_flashProgRetryCount         = 1;

	m_abortModProgramming         = false;
  m_modProgrammingInProgress    = false;

	m_abortFPGAProgramming        = false;
  m_FPGAProgrammingInProgress   = false;

  m_programMain                 = false;
  m_programBoot                 = false;
  
  m_commsInterMessageGap        = geminiInterMessageGapMin;
  m_geminiEvoRangeCompression   = geminiEvoRangeCompressionDefault;
  
  m_geminiCompressionLines[7]   = 4096;
  m_geminiCompressionLines[6]   = 2048;
  m_geminiCompressionLines[5]   = 1024;
  m_geminiCompressionLines[4]   = 512;
  m_geminiCompressionLines[3]   = 256;
  m_geminiCompressionLines[2]   = 128;
  m_geminiCompressionLines[1]   = 64;
  m_geminiCompressionLines[0]   = 32;

  m_geminiCompressionFactorUsed = 1;
  
  m_VDSLSettingsToUse           = 0;

  ResetCounters();
}

//----------------------------------------------------------------------------
CGeminiNetwork::~CGeminiNetwork()
{
  Stop();
}

//----------------------------------------------------------------------------
bool CGeminiNetwork::Init(void)
{
  bool success = false;
    
	success = CreateSocketRX();
	
	if (success)
  	success = CreateSocketTX(false);

  m_keepSocketTaskRunning = true;
  m_keepTimerTaskRunning  = true;

#ifdef _WIN32
  // Only appropriate to start RX thread if network initialised correctly
  if (success)
  {
    _beginthread(RXDATA,    0, (void *)this);
  }
  _beginthread(TIMERTICK, 0, (void *)this);
  _beginthread(IPHELPER,  0, (void *)this);	
#else
	// The supplied windows code didn't seem to care about checking for return
	// codes or handles so we'll just do the same in linux.
	pthread_t hRXID, hTimerID, hIPID;
	if(success)
	{
		pthread_create(&hRXID, NULL, (void*(*)(void*))RXDATA, this);
	}
	pthread_create(&hTimerID, NULL, (void*(*)(void*))TIMERTICK, this);
	pthread_create(&hIPID, NULL, (void*(*)(void*))IPHELPER, this);
#endif
  
  return success;
}

//----------------------------------------------------------------------------
void CGeminiNetwork::Stop(void)
{
  m_keepSocketTaskRunning = false;
  m_keepTimerTaskRunning  = false;

  if (m_modProgrammingInProgress)
    m_abortModProgramming = true;

  if (m_FPGAProgrammingInProgress)
    m_abortFPGAProgramming = true;

	#ifdef _WIN32
  		Sleep(1010); // Sleep to allow tasks to stop
  	#else
  		// linux dosen't have a direct microsecond timer so we split the 1010
  		// milliseconds into 1 second and 10 milliseconds.
  		sleep(1);
  		usleep(10000);
  	#endif

/* NO RS232 IN LINUX JUST NOW*/
#ifdef _WIN32
  if (serial.IsOpen())
  {
    serial.Close();
    m_serialPortOpened = false;
  }
#endif

  DestroySocketTX();
  DestroySocketRX();
#ifdef _WIN32
  WSACleanup();
#endif

  if (m_modifiersCalculated)
  {
    m_geminiModifiers.FreeModifiers();
    m_modifiersCalculated = false;
  }

  if (m_FPGAFlashBlocks[0].m_pData)
  {
    free(m_FPGAFlashBlocks[0].m_pData);
    m_FPGAFlashBlocks[0].m_pData = NULL;
  }
  
  for (int i = 0; i < BUF_SIZE; i++)
  {
    if (m_tgtImgBuf[i].m_retryQueuePtr != NULL)
	  {
	    delete(m_tgtImgBuf[i].m_retryQueuePtr);
	    m_tgtImgBuf[i].m_retryQueuePtr = NULL;
	  }

    if (m_tgtImgBuf[i].m_dataPtr != NULL)
    {
      delete(m_tgtImgBuf[i].m_dataPtr);
      m_tgtImgBuf[i].m_dataPtr = NULL;
    }
#ifdef _WIN32
	  if (ReleaseMutex(m_tgtImgBuf[i].m_mutex) != 0)
#else
	  if (ReleaseMutex(m_tgtImgBuf[i].m_mutex) != 0)
#endif
  	  m_tgtImgBuf[i].m_bLocked = false;
  }
}

//----------------------------------------------------------------------------
void CGeminiNetwork::ResetCounters(void)
{
  m_packetCount                 = 0;
  m_recvErrorCount              = 0;
  m_generalCount                = 0;
}
  
//----------------------------------------------------------------------------
unsigned short CGeminiNetwork::GetSonarID(void)
{
  return m_sonarID;
}

//----------------------------------------------------------------------------
void CGeminiNetwork::SetDLLSonarID(unsigned short sonarID)
{
  m_sonarID = sonarID;  
}

//----------------------------------------------------------------------------
void CGeminiNetwork::ProgramSonarID(unsigned short sonarID, int useHighFreq)
{
  // Build the packet to send to the sonar
  
  CGemInfoPacketBank0 newSonarData;
  
  newSonarData.m_sonarID = sonarID;
  
  if (useHighFreq == 0)
    newSonarData.m_transducerFreq = 1;
  else
    newSonarData.m_transducerFreq = 0;
  
  // Actually send the message
  
	SendMsg(&newSonarData, sizeof(newSonarData));

}

//----------------------------------------------------------------------------
void CGeminiNetwork::SetHandlerFunction(void (cdecl *FnPtr)(int eType, int len, char *dataBlock))
{
  m_dataHandlerFunction = FnPtr;
}

//----------------------------------------------------------------------------
void CGeminiNetwork::SetProgressFunction(void (cdecl *FnPtr)(int sType, int retryCount, int packet, 
                                                             int block, int totalBlocks, int otherValue))
{
  m_progressCallback = FnPtr;
}

//----------------------------------------------------------------------------
bool CGeminiNetwork::CreateSocketRX()
{
#ifdef _WIN32
  WORD    versionRequested;
  WSADATA wsaData;
#endif
  int     err;
  int     socketResult;
  DWORD   off         = 0;
  DWORD   receiveSize = 0x10000; // 64K receive buffer
	//DWORD   receiveSize = 0xffffff; // 16M  receive buffer

  sockaddr_in socketAddress;

  char hostName[256];
  hostent *hp;
  
  bool    retValue = true;

  // First thing to check - have we already connected?
  
  if (m_sockRXConnected == true)
    return true;

  // It's safe to carry on!
    
  // Request winsock 2.0 (or better)
#ifdef _WIN32   
  versionRequested  = MAKEWORD(2, 0);
#endif
  m_sockRXConnected = false;

  // Initialise sockets
#ifdef _WIN32
  err = WSAStartup(versionRequested, &wsaData);
#else
	err = 0;
#endif

  if (err == 0) 
  {
    // Create the socket

    m_socketRX = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    
    if (m_socketRX != INVALID_SOCKET)
    {
      // Build address for binding socket
    
      socketAddress.sin_family      = AF_INET;
      socketAddress.sin_port        = htons(52900);
            socketResult = gethostname(hostName, sizeof(hostName));

      if (socketResult != INVALID_SOCKET)
      {
        hp = gethostbyname(hostName);

        /* Check for NULL pointer */
        if (hp == NULL)
        {
          socketResult = INVALID_SOCKET;
        }
        else
        {
#ifdef _WIN32
          /* Assign the address */
		       socketAddress.sin_addr.S_un.S_un_b.s_b1 = hp->h_addr_list[0][0];
		       socketAddress.sin_addr.S_un.S_un_b.s_b2 = hp->h_addr_list[0][1];
		       socketAddress.sin_addr.S_un.S_un_b.s_b3 = hp->h_addr_list[0][2];
		       socketAddress.sin_addr.S_un.S_un_b.s_b4 = hp->h_addr_list[0][3];
#else
				socketAddress.sin_addr.s_addr = INADDR_ANY;
#endif
          // And bind it
          socketResult = bind(m_socketRX, (sockaddr *)(&socketAddress), sizeof(socketAddress));
        }
      }
      
      if (socketResult != SOCKET_ERROR)
      {
        // Turn non-blocking off
      
// Currently commented out, even though it may not have any effect, or actually turn out to be beneficial      
//        socketResult = ioctlsocket(m_socketRX, FIONBIO, &off);

        if (socketResult != SOCKET_ERROR)
        {
          // Increase the size of the receive buffer

          socketResult = setsockopt(m_socketRX, SOL_SOCKET, SO_RCVBUF, (const char *)&receiveSize, sizeof(receiveSize));
          
          if (socketResult != SOCKET_ERROR)
          {
            // And if everything worked

            m_sockRXConnected = true;
          }
          else
          {
            // setsockopt() failed
            retValue = false;
          }
        }
        else
        {
          // ioctlsocket() failed
          retValue = false;
        }
      }
      else
      {
        // bind() failed
        #ifdef _WIN32
	        err = WSAGetLastError();
   	     WSASetLastError(0); 
   	  #else
   	  	  err = errno;
   	  #endif
        retValue = false;
      }
    }
    else
    {
      // socket() failed
      retValue = false;
    }
  }
  else
  {
    // WSAStartUp() failed
    retValue = false;
  }

  return retValue;
}

//----------------------------------------------------------------------------
bool CGeminiNetwork::CreateSocketTX(bool InitWSA)
{
#ifdef _WIN32
  WORD    versionRequested;
  WSADATA wsaData;
#endif
  int     err;
  int     socketResult;
  DWORD   off         = 0;
  DWORD   receiveSize = 0x10000; // 64K receive buffer 
	//DWORD   receiveSize = 0xffffff; // 16M  receive buffer

  sockaddr_in socketAddress;

  char hostName[256];
  hostent *hp;

  bool    retValue = true;

  // First thing to check - have we already connected?
  
  if (m_sockTXConnected == true)
    return true;

  // It's safe to carry on!

  if (InitWSA)
  {    
    // Request winsock 2.0 (or better)
#ifdef _WIN32     
    versionRequested  = MAKEWORD(2, 0);
#endif
    m_sockTXConnected = false;

    // Initialise sockets
#ifdef _WIN32
    err = WSAStartup(versionRequested, &wsaData);
#else
	err = 0;
#endif
  }
  else
  {
    err = 0;
  }
  
  if (err == 0) 
  {
    // Create the socket

    m_socketTX = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    
    if (m_socketTX != INVALID_SOCKET)
    {
      // Build address for binding socket
    
      socketAddress.sin_family      = AF_INET;
      socketAddress.sin_port        = htons(52901);

      socketResult = gethostname(hostName, sizeof(hostName));

      if (socketResult != INVALID_SOCKET)
      {
        hp = gethostbyname(hostName);

        /* Check for NULL pointer */
        if (hp == NULL)
        {
          socketResult = INVALID_SOCKET;
        }
        else
        {
#ifdef _WIN32
          /* Assign the address */
          socketAddress.sin_addr.S_un.S_un_b.s_b1 = hp->h_addr_list[0][0];
          socketAddress.sin_addr.S_un.S_un_b.s_b2 = hp->h_addr_list[0][1];
          socketAddress.sin_addr.S_un.S_un_b.s_b3 = hp->h_addr_list[0][2];
          socketAddress.sin_addr.S_un.S_un_b.s_b4 = hp->h_addr_list[0][3];
#else
		    socketAddress.sin_addr.s_addr = INADDR_ANY;
#endif

          // And bind it
          socketResult = bind(m_socketTX, (sockaddr *)(&socketAddress), sizeof(socketAddress));
        }
      }
      
      if (socketResult != SOCKET_ERROR)
      {
        // Turn blocking off

// Currently commented out, even though it may not have any effect, or actually turn out to be beneficial      
//        socketResult = ioctlsocket(m_socketRX, FIONBIO, &off);

        if (socketResult != SOCKET_ERROR)
        {
          // Increase the size of the receive buffer
          socketResult = setsockopt(m_socketTX, SOL_SOCKET, SO_RCVBUF, (const char *)&receiveSize, sizeof(receiveSize));
          
          if (socketResult != SOCKET_ERROR)
          {
            // And if everything worked

            m_sockTXConnected = true;
          }
          else
          {
            // setsockopt() failed
            retValue = false;
          }
        }
        else
        {
          // ioctlsocket() failed
          retValue = false;
        }
      }
      else
      {
        // bind() failed
        #ifdef _WIN32
	        err = WSAGetLastError();
   	     WSASetLastError(0); 
   	  #else
   	  	  err = errno;
   	  #endif
        retValue = false;
      }
    }
    else
    {
      // socket() failed
      retValue = false;
    }
  }
  else
  {
    // WSAStartUp() failed
    retValue = false;
  }

  return retValue;
}

//----------------------------------------------------------------------------
void CGeminiNetwork::DestroySocketRX()
{
  int socketResult;

	#ifdef _WIN32
		socketResult = closesocket(m_socketRX);
	#else
		socketResult = close(m_socketRX);
	#endif
	
  m_sockRXConnected = false;
}

//----------------------------------------------------------------------------
void CGeminiNetwork::DestroySocketTX()
{
  int socketResult;

	#ifdef _WIN32
		socketResult = closesocket(m_socketTX);
	#else
		socketResult = close(m_socketTX);
	#endif

  m_sockTXConnected = false;
}

//----------------------------------------------------------------------------
bool CGeminiNetwork::Reconnect()
{
  bool success = false;

	DestroySocketTX();
	DestroySocketRX();
#ifdef _WIN32
  WSACleanup();
#endif
	
	success = CreateSocketRX();
	
	if (success)
  	success = CreateSocketTX(false);

  return success;
}

//----------------------------------------------------------------------------
bool CGeminiNetwork::InitComPort(int comPortNo, unsigned short echoData, 
                                                unsigned short sendToCalling)
{
/* NO RS232 IN LINUX JUST NOW*/
#ifdef _WIN32
  USES_CONVERSION;

  char portName[6];

  LONG lastError = ERROR_SUCCESS;

  if (serial.IsOpen())
  {
    serial.Close();
    m_serialPortOpened = false;
  }

  if (comPortNo > 0 )
  {
  		if (comPortNo < 10)
			sprintf_s(portName, "COM%d", comPortNo);
	   else
			sprintf_s(portName, "\\\\.\\COM%d", comPortNo);

    LPCTSTR convPortName = A2CT(portName);
     
    lastError = serial.Open(convPortName, 0, 0, false);

    if (lastError == ERROR_SUCCESS)
    {
      lastError = serial.Setup(CSerial::EBaud9600, 
                               CSerial::EData8, 
                               CSerial::EParNone, 
                               CSerial::EStop1);


      if (lastError == ERROR_SUCCESS)
      {
      
        lastError = serial.SetupReadTimeouts(CSerial::EReadTimeoutNonblocking);
      
        if (lastError == ERROR_SUCCESS)
        {
          m_serialPortOpened = true;
        }
      }
    }
  }
  
  if (echoData == 1)
    m_serialEchoData = true;
  else
    m_serialEchoData = false;

  if (sendToCalling == 1)
    m_serialSendToCalling = true;
  else
    m_serialSendToCalling = false;

  return m_serialPortOpened;
#else
	return false;
#endif
}

//----------------------------------------------------------------------------
void CGeminiNetwork::TimerTick5mS(void)
{
  bool attemptRetry = false;
  
  // Take 2 of getting this timer tick to retry the tail if not received in time
  // Worst case (we can think of) is that we will receive 25 lines in 5mS.
  // Therefore if the ping has started, and we are within 25 lines of the end when
  // this timer tick is called, then the next time the ping should be complete. If not
  // we should call for a retry of the tail.
  
  // First test - has the ping started
  if (m_tgtImgBuf[m_tgtImgIpPtr].m_pingStarted == false)
    return;
  
  // Are we within 25 lines of the end, we should look to attempting a retry
  // Remember successful receipt of a ping tail will clear the started flag
  if ((m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed - m_tgtImgBuf[m_tgtImgIpPtr].m_lastRxLineNo) < 25)
  {
    attemptRetry = true;
  }
  
  if (attemptRetry)
  {  
    // No ping tail received, ensure we wait at least 5mS before we request it
    if (m_tgtImgBuf[m_tgtImgIpPtr].m_queuedTailRetry == false)
    {
      // If we are still data complete but no ping tail when this code ticks again, we will request a new tail
      m_tgtImgBuf[m_tgtImgIpPtr].m_queuedTailRetry = true;
    }
    else
    {
      // But no ping tail, so re-request it
      RequestRetry(GEM_RETRY_REQUEST_TAIL, 0, 1);
      m_tgtImgBuf[m_tgtImgIpPtr].m_tailRetries++;
    }
  }
}

//----------------------------------------------------------------------------
void CGeminiNetwork::TimerTick50mS(void)
{
/* NO RS232 IN LINUX JUST NOW*/
#ifdef _WIN32
  USES_CONVERSION;
  
  LONG  lastError;
  DWORD length = 0;
  
  char lData[1024];

  if (m_serialPortOpened)
  {
    do
    {
      lastError = serial.Read(lData, sizeof(lData),  &length);

      if (lastError == ERROR_SUCCESS)
      {
        if (length > 0)
        {
          if (m_serialSendToCalling)
          {
//            LPCTSTR convData = A2CT(lData);
            
            if (m_dataHandlerFunction)
              m_dataHandlerFunction(GEM_SERIAL_PORT_INPUT, length, lData); //(char *)&convData);
          }  
          else
            SendRS232Data(length, lData);
        }
      }  
    } while (length == sizeof(lData));
  }
#endif
}

//----------------------------------------------------------------------------
int CGeminiNetwork::WriteToComPort(char *data, unsigned short len)
{
/* NO RS232 IN LINUX JUST NOW*/
#ifdef _WIN32
  LONG result = 0;
  
  if (m_serialPortOpened == true)
  {
    result = serial.Write(data, len);
  }
  
  return (int)result;
#else
	return 0;
#endif
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// Transmit packet commands

//----------------------------------------------------------------------------
void CGeminiNetwork::SendMsg(CGemNetMsg* pMsg, int size)
{
  SendMsg(pMsg, size, m_useAltIPAddress);
}

//----------------------------------------------------------------------------
void CGeminiNetwork::SendMsg(CGemNetMsg* pMsg, int size, bool useAltIPAddress)
{
  int         err;
  int         socketResult;
  sockaddr_in socketAddress;

  // DLL does nothing if m_sonarID is set to zero or one, no transmission will take place
  // If in production tool mode, will transmit regardless of Sonar ID

  if ((m_sonarID > 1) || (m_geminiCommsMode == geminiCommsModeProd))
  {
    // Make sure message goes to intended sonar
	  pMsg->m_head.m_deviceID = m_sonarID;

    // Build address for sending data

    socketAddress.sin_family = AF_INET;
    socketAddress.sin_port   = htons(52901);

#ifdef _WIN32
    if (useAltIPAddress)
    {
      socketAddress.sin_addr.S_un.S_un_b.s_b1 = m_altIPAddr1;
      socketAddress.sin_addr.S_un.S_un_b.s_b2 = m_altIPAddr2;
      socketAddress.sin_addr.S_un.S_un_b.s_b3 = m_altIPAddr3;
      socketAddress.sin_addr.S_un.S_un_b.s_b4 = m_altIPAddr4;
    }
    else
    {
      socketAddress.sin_addr.S_un.S_un_b.s_b1 = PRESET_IP_MSB;
      socketAddress.sin_addr.S_un.S_un_b.s_b2 = PRESET_IP_SB3;
      socketAddress.sin_addr.S_un.S_un_b.s_b3 = PRESET_IP_SB2;
      socketAddress.sin_addr.S_un.S_un_b.s_b4 = PRESET_IP_LSB;
    }
#else
	char chIP[16] = {0};
	if(useAltIPAddress)
	{
		sprintf(chIP, "%d.%d.%d.%d\0", m_altIPAddr1, m_altIPAddr2, m_altIPAddr3, m_altIPAddr4);
	}
	else
	{
		sprintf(chIP, "%d.%d.%d.%d\0", PRESET_IP_MSB, PRESET_IP_SB3, PRESET_IP_SB2, PRESET_IP_LSB);
	}
	socketAddress.sin_addr.s_addr = inet_addr(chIP);
#endif

    // Only transmit if we are actually connected
    if (m_sockTXConnected == true)
    {
		// Send the message
	   socketResult = sendto(m_socketTX, (char *)pMsg, size, 0, (sockaddr *)(&socketAddress), sizeof(socketAddress));

      // Check the result
	    if (socketResult == SOCKET_ERROR)				
	    {
	      // Failed to send data for some reason
	      #ifdef _WIN32
        		err = WSAGetLastError();
        		WSASetLastError(0); 
        	#else
        		err = errno;
        	#endif
	    }
    }
  }
}

//----------------------------------------------------------------------------
void CGeminiNetwork::GetAltSonarIPAddress(unsigned char *a1, unsigned char *a2, unsigned char *a3, unsigned char *a4,
                                          unsigned char *s1, unsigned char *s2, unsigned char *s3, unsigned char *s4)
{
  // Return values from member variables
  
  *a1 = m_altIPAddr1;
  *a2 = m_altIPAddr2;
  *a3 = m_altIPAddr3;
  *a4 = m_altIPAddr4;

  *s1 = m_geminiSubnetMask1;
  *s2 = m_geminiSubnetMask2;
  *s3 = m_geminiSubnetMask3;
  *s4 = m_geminiSubnetMask4;
}

//----------------------------------------------------------------------------
void CGeminiNetwork::SetAltSonarIPAddress(unsigned char a1, unsigned char a2, unsigned char a3, unsigned char a4,
                                          unsigned char s1, unsigned char s2, unsigned char s3, unsigned char s4)
{
  CGemInfoPacketBank2 newIPAddr;

  // Use union to pack 4 bytes of IP address into a 32 bit value
  
  union packingType
  {
    unsigned int value;
    char         bytes[4];
  };
  
  packingType lValue;

  // Write individual bytes into union

  lValue.bytes[0] = a4;
  lValue.bytes[1] = a3;
  lValue.bytes[2] = a2;
  lValue.bytes[3] = a1;
  
  // Use 32 bit value in data to be sent to sonar
    
  newIPAddr.m_alt_ip = lValue.value;

  // Do the same with subnet mask
  
  lValue.bytes[0] = 0;  // We will never set the least significant byte of the subnet mask
  lValue.bytes[1] = s3;
  lValue.bytes[2] = s2;
  lValue.bytes[3] = s1;
  
  newIPAddr.m_subnet_mask = lValue.value;
  
  // Actually send the message
  
	SendMsg(&newIPAddr, sizeof(newIPAddr));

  // NB DO NOT SET values written to sonar into alternate values stored by class, 
  // as sonar will not use them until rebooted.
}

//----------------------------------------------------------------------------
void CGeminiNetwork::UseAltSonarIPAddress(unsigned char a1, unsigned char a2, unsigned char a3, unsigned char a4,
                                          unsigned char s1, unsigned char s2, unsigned char s3, unsigned char s4)
{
  // Save values into member variables for later use
  
  m_altIPAddr1 = a1;
  m_altIPAddr2 = a2;
  m_altIPAddr3 = a3;
  m_altIPAddr4 = a4;
  
  m_geminiSubnetMask1 = s1;
  m_geminiSubnetMask2 = s2;
  m_geminiSubnetMask3 = s3;
  m_geminiSubnetMask4 = s4;
}

//----------------------------------------------------------------------------
void CGeminiNetwork::TxToAltIPAddress(bool useAltIPAddress)
{
  // If m_useAltIPAddress is true, the data will be sent to the alternate IP address
  // configured with the UseAltSonarIPAddress function

  m_useAltIPAddress = useAltIPAddress;
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RebootSonar(void)
{
  // Build and send a reboot sonar command

  CGemRebootPacket rebootPacket;
  
  rebootPacket.m_rebootControl = 0x01;
  
	SendMsg(&rebootPacket, sizeof(rebootPacket));
}

//----------------------------------------------------------------------------
void CGeminiNetwork::InhibitSonarReboot(void)
{
  // Build and send a reboot sonar command

  CGemRebootPacket rebootPacket;
  
  rebootPacket.m_rebootControl = 0x02;
  
	SendMsg(&rebootPacket, sizeof(rebootPacket));
}

//----------------------------------------------------------------------------
void CGeminiNetwork::SendRS232Data(int len, char *data)
{
  int i;
  int size;

  m_geminiRS232Tx.m_length = len;
  
  for (i = 0; i < len; i++)
  {
    m_geminiRS232Tx.m_data[i] = data[i];
  }

  // Packet is of a variable length dependant on amount of data to be sent

  size = sizeof(CGemNetMsg);  // Length of packet header
  size += 1;                  // Space for length field
  size += len;                // 1 byte for each data byte to be sent out

	SendMsg(&m_geminiRS232Tx, size);
}

//----------------------------------------------------------------------------
void CGeminiNetwork::CommandPOST(void)
{
  // Build and send a POST command

  CGemPOSTPacket POSTPacket;

  // Only comply if in production test mode
  if (m_geminiCommsMode == geminiCommsModeProd)
  {  
  	SendMsg(&POSTPacket, sizeof(POSTPacket));
  }
}

//----------------------------------------------------------------------------
void CGeminiNetwork::CommandProductionTest(bool runProdTest, unsigned long long seedValue)
{
  // Build and send a production test command

  CGemProdTestPacket prodTestPacket;
  
  // Seed value of zero is used to indicate that the sonar head is to use its own seed value
  // Otherwise ensure that the most significant bit is set to indicate that this value should be used
  
  if (seedValue != 0)
    prodTestPacket.m_PRBSSeed = seedValue | 0x8000000000000000ULL;
  else
    prodTestPacket.m_PRBSSeed = 0;
      
  // Set the flags value according to the value of runProdTest
  
  if (runProdTest)
    prodTestPacket.m_flags = 0x01;
  else
    prodTestPacket.m_flags = 0;
  
  // Only comply if in production test mode
  if (m_geminiCommsMode == geminiCommsModeProd)
  {  
  	SendMsg(&prodTestPacket, sizeof(prodTestPacket));
  }
}

//----------------------------------------------------------------------------
void CGeminiNetwork::AbortProductionTest(void)
{
  // Build and send a production test abort command

  CGemProdTestAbortPacket prodTestAbortPacket;
  
  // Only comply if in production test mode
  if (m_geminiCommsMode == geminiCommsModeProd)
  {  
  	SendMsg(&prodTestAbortPacket, sizeof(prodTestAbortPacket));
  }
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RequestTestResult(void)
{
  // Build and send a request test result command

  CGemResultRequestPacket resultRequestPacket;
  
  // Only comply if in production test mode
  if (m_geminiCommsMode == geminiCommsModeProd)
  {  
  	SendMsg(&resultRequestPacket, sizeof(resultRequestPacket));
  }
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// Modifiers and Flash Memory handlers

//----------------------------------------------------------------------------
void CGeminiNetwork::InitCalibrationValues(bool useFiles, char *txFile, char *rxFile)
{
  m_geminiModifiers.InitCalibrationValues(useFiles, txFile, rxFile);
}

//----------------------------------------------------------------------------
bool CGeminiNetwork::CalculateModifiers(void)
{
  m_modifiersCalculated = m_geminiModifiers.Calculate();
  return m_modifiersCalculated;
}

//----------------------------------------------------------------------------
void CGeminiNetwork::AbortModifierProgramming(void)
{
  m_abortModProgramming = true;
}

//----------------------------------------------------------------------------
void PROGRAMMODIFIERSTASK(void *pClassPtr)
{
  CGeminiNetwork *pGeminiNetwork = (CGeminiNetwork *)pClassPtr;	

  pGeminiNetwork->ProgramModifiersCode();
}

//----------------------------------------------------------------------------
void CGeminiNetwork::ProgramModifiers(void)
{
  m_abortModProgramming = false;
#ifdef _WIN32
  _beginthread(PROGRAMMODIFIERSTASK, 0, (void *)this);
#else
	pthread_t hPMID;
	pthread_create(&hPMID, NULL, (void*(*)(void*))PROGRAMMODIFIERSTASK, this);
#endif
}

//----------------------------------------------------------------------------
void CGeminiNetwork::ProgramModifiersCode(void)
{
#ifdef _WIN32
	int goodCount   = 0;
	int badCount    = 0;
	int sent        = 0;
  int sentPackets = 0;
  int block       = 0;
  int tryCount    = 0;
  int differCount = 0;

  bool succeeded  = false;
  bool calculated = true;
  
  m_modProgrammingInProgress = true;

  if (m_modifiersCalculated == false)
  {
    if (m_progressCallback)
      m_progressCallback(GEM_MODPROG_CALCULATING, 0, 0, 0, 0, 0);

    calculated = CalculateModifiers();
  }

  // Stop at this point if we have not calculated the modifiers
  if (calculated == false)
  {
    if (m_progressCallback)
      m_progressCallback(GEM_MODPROG_CALCFAIL, 0, 0, 0, 0, 0);

    m_modProgrammingInProgress = false;
    
    return;  
  }

  // We are about to start programming the modifiers
  if (m_progressCallback)
    m_progressCallback(GEM_MODPROG_START, 0, 0, 0, 0, 0);

	CGemFlashWritePacket fp;
	fp.m_flags = 0x01; // bit 0 initiates a readback
				
	for (block = 0; block < MAX_FLASH_BLOCKS && m_geminiModifiers.m_flashBlocks[block].m_pData; block++)
	{			
		sent        = 0;
		sentPackets = 0;

		unsigned startAddress = m_geminiModifiers.m_flashBlocks[block].m_addr; // specified in bytes

		// Check that the start address is within the range that the modifiers should be written
		if (startAddress < 0x00800000)
		{
      if (m_progressCallback)
        m_progressCallback(GEM_MODPROG_LOW_ADDRESS, tryCount, sentPackets, block, MAX_FLASH_BLOCKS, (int)startAddress);

      m_modProgrammingInProgress = false;

      return;
		}

		// Check that the start address is on a block boundary to ensure proper flash erase
		if (startAddress & 0x0001ffff)
		{
      if (m_progressCallback)
        m_progressCallback(GEM_MODPROG_NOT_ALIGNED, tryCount, sentPackets, block, MAX_FLASH_BLOCKS, (int)startAddress);

      m_modProgrammingInProgress = false;

      return;
		}

    // OK, we are happy to try and program the modifiers

		fp.m_addr   = startAddress;
		BYTE* pBuf  = m_geminiModifiers.m_flashBlocks[block].m_pData;
		int toSend  = m_geminiModifiers.m_flashBlocks[block].m_size;

		// Clear the messages in case they were left set by another flash read
		WaitForSingleObject(hFlashResultReceived, 0);
		WaitForSingleObject(hFlashReadbackReceived, 0);

		while (toSend > 0)
		{
      // Check to see if the programming has been aborted
      if (m_abortModProgramming == true)
      {
        if (m_progressCallback)
          m_progressCallback(GEM_MODPROG_ABORTED, 0, 0, 0, 0, 0);
          
        m_modProgrammingInProgress = false;

        return;
      }
      
			// We will be sending in chunks of 1024 bytes
			int numSend = (std::min)(1024, toSend);

			memcpy(fp.m_data, pBuf, numSend);

			int numPad = 1024 - numSend;

			if (numPad)
				memset(&fp.m_data[numSend], 0xff, numPad);

		  tryCount  = 0;
      succeeded = false;
      
      while ((tryCount <= m_flashProgRetryCount) && !succeeded)
      {
			  SendMsg(&fp, sizeof(fp));

			  // Wait for the signal indicating a flash result under timeout
			  if (WaitForSingleObject(hFlashResultReceived, 5000) != WAIT_TIMEOUT)
			  {
          if ((m_flashResult.m_result & 0x7fff) != 0)
          {
            // Flash failed to program (according to Gemini)

            if (m_progressCallback)
              m_progressCallback(GEM_MODPROG_FAILURE, tryCount, sentPackets, block, MAX_FLASH_BLOCKS, (int)m_flashResult.m_result);

			      badCount++;
          }

		      // Wait for the signal indicating a flash readback under timeout
		      if (WaitForSingleObject(hFlashReadbackReceived, 1000) != WAIT_TIMEOUT)
		      {
            // Flash programmed OK (according to Gemini)
			      // Compare the flash result with the original sent flash packet
			      if (memcmp(fp.m_data, m_flashReadback.m_data, 1024))
			      {				
              // Compare to original data failed

              differCount = 0;

              for (int i = 0; i < 1024; i++)
              {
                if (fp.m_data[i] != m_flashReadback.m_data[i])
                  differCount++;
              }

              if (m_progressCallback)
                m_progressCallback(GEM_MODPROG_BAD_COMPARE, tryCount, sentPackets, block, MAX_FLASH_BLOCKS, differCount);

				      badCount++;
			      }
			      else
			      {
			        // Programmed and compared successfully - we can say the programming was successful
			        
              if (m_progressCallback)
                m_progressCallback(GEM_MODPROG_SUCCESS, tryCount, sentPackets, block, MAX_FLASH_BLOCKS, 0);

              // Stop retry mechanism. As we succeeded, we don't need to retry
              succeeded = true;

				      goodCount++;
              sentPackets++;
			      }
          }
		      else
		      {
  			    // Timed out waiting for readback packet

            if (m_progressCallback)
              m_progressCallback(GEM_MODPROG_TIMEOUT, tryCount, sentPackets, block, MAX_FLASH_BLOCKS, 0);
		      }
			  } 
			  else
			  {
			    // Timed out waiting for result packet

          if (m_progressCallback)
            m_progressCallback(GEM_MODPROG_TIMEOUT, tryCount, sentPackets, block, MAX_FLASH_BLOCKS,0 );
			  }
			  
			  tryCount++;
      }
      
      // Did we actually program the block?
      if (succeeded == false)
      {
        // No, stop the sending loop
			  toSend    = 0;
      }
      else
      {
			  pBuf      += numSend;
			  sent      += numSend;
			  toSend    -= numSend;
			  fp.m_addr += 1024;
      }
		}

    // Did we actually program the block?
    if (succeeded == false)
    {
      // No
      break;
    }
 	}

  if (m_progressCallback)
  {
    if (succeeded == false)
    {
      m_progressCallback(GEM_MODPROG_FAILED, 0, sentPackets, block, MAX_FLASH_BLOCKS, 0);
    }
    else
    {  
      m_progressCallback(GEM_MODPROG_COMPLETE, 0, sentPackets, block, MAX_FLASH_BLOCKS, 0);
    }
  }

  m_modProgrammingInProgress = false;
#endif
}

//----------------------------------------------------------------------------
int CGeminiNetwork::WriteModifiersToFile(char *fileName)
{
  int retValue = GEM_FILE_SUCCESS;
  int block    = 0;
  int counter;
  int fCount;
  
  bool calculated = true;
  
  errno_t err;
  
  FILE *fp;

  if (m_modifiersCalculated == false)
  {
    calculated = CalculateModifiers();
  }

  if (calculated == false)
  {
    return GEM_CALC_FAILED;    
  }

	#ifdef _WIN32
		err = fopen_s(&fp, fileName, "wt");
	#else
		fp = fopen(fileName, "wt");
		if(fp==NULL)
		{
			err = EINVAL;
		}
	#endif
  
  if (err == 0)
  {
	  for (block = 0; block < MAX_FLASH_BLOCKS && m_geminiModifiers.m_flashBlocks[block].m_pData; block++)
	  {			
      fCount = fprintf(fp, "%X\n", m_geminiModifiers.m_flashBlocks[block].m_addr);
      
      if (fCount == 0)
      {
        fclose(fp);
        return GEM_FILE_WRITE_FAILED;
      }
      
      fCount = fprintf(fp, "%d\n", m_geminiModifiers.m_flashBlocks[block].m_size);
    
      if (fCount == 0)
      {
        fclose(fp);
        return GEM_FILE_WRITE_FAILED;
      }
      
      for (counter = 0; counter < (int)m_geminiModifiers.m_flashBlocks[block].m_size; counter++)
      {
        fCount = fprintf(fp, "%c\n", m_geminiModifiers.m_flashBlocks[block].m_pData[counter]);

        if (fCount == 0)
        {
          fclose(fp);
          return GEM_FILE_WRITE_FAILED;
        }
      }
    }

    fclose(fp);
  }
  else
  {
    // File failed to open
    
    retValue = GEM_FILE_OPEN_FAILED;
  }
    
  return retValue;
}

//----------------------------------------------------------------------------
int CGeminiNetwork::ReadModifiersFromFile(char *fileName)
{
  int retValue = GEM_FILE_SUCCESS;
  int block    = 0;
  int counter;
  int fCount; 
  
  errno_t err;
  
  FILE *fp;

  m_modifiersCalculated = true;

	#ifdef _WIN32
  		err = fopen_s(&fp, fileName, "rt");
	#else
		fp = fopen(fileName, "rt");
		if(fp==NULL)
		{
			err = EINVAL;
		}
	#endif
  
  if (err == 0)
  {
	  for (block = 0; block < MAX_FLASH_BLOCKS; block++)
	  {			
	  		#ifdef _WIN32
		      fCount = fscanf_s(fp, "%X\n", &m_geminiModifiers.m_flashBlocks[block].m_addr);
		   #else
		   	fCount = fscanf(fp, "%X\n", &m_geminiModifiers.m_flashBlocks[block].m_addr);
		   #endif
      
      if (fCount == 0)
      {
        fclose(fp);

        if (block == 0)
        {
          // We haven't read any blocks from the file - therefore we have failed
          m_modifiersCalculated = false;
          return GEM_FILE_READ_FAILED;
        }
        else
        {
          // We have read at least one block, so (assume) that this is all the data in 
          // the file, and that we have successfully read the file
          return GEM_FILE_SUCCESS;
        }
      }
           
      #ifdef _WIN32 
	      fCount = fscanf_s(fp, "%d\n", &m_geminiModifiers.m_flashBlocks[block].m_size);
	   #else
	   	fCount = fscanf(fp, "%d\n", &m_geminiModifiers.m_flashBlocks[block].m_size);
	   #endif
    
      if (fCount == 0)
      {
        fclose(fp);
        m_modifiersCalculated = false;
        return GEM_FILE_READ_FAILED;
      }
      
      m_geminiModifiers.m_flashBlocks[block].m_pData = (BYTE*)realloc(m_geminiModifiers.m_flashBlocks[block].m_pData, 
                                                                      m_geminiModifiers.m_flashBlocks[block].m_size);
 
      if (m_geminiModifiers.m_flashBlocks[block].m_pData)
      {
        for (counter = 0; counter < (int)m_geminiModifiers.m_flashBlocks[block].m_size; counter++)
        {
        		#ifdef _WIN32
          		fCount = fscanf_s(fp, "%c\n", &m_geminiModifiers.m_flashBlocks[block].m_pData[counter]);
          	#else
          		fCount = fscanf(fp, "%c\n", &m_geminiModifiers.m_flashBlocks[block].m_pData[counter]);
          	#endif

          if (fCount == 0)
          {
            fclose(fp);
            m_modifiersCalculated = false;
            return GEM_FILE_READ_FAILED;
          }
        }
      }
      else
      {
        fclose(fp);
        m_modifiersCalculated = false;
        return GEM_FILE_READ_FAILED;
      }
    }
    
    fclose(fp);
  }
  else
  {
    // File failed to open
    
    m_modifiersCalculated = false;
    retValue = GEM_FILE_OPEN_FAILED;
  }
    
  return retValue;
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
int CGeminiNetwork::DownloadBitFile(char *fileName)
{
	FILE* fp;
	errno_t fpStatus;
	
	#ifdef _WIN32
		fpStatus = fopen_s(&fp, fileName, "rb");
	#else
		fp = fopen(fileName, "rb");
		if(fp==NULL)
		{
			fpStatus = EINVAL;
		}
	#endif

	BYTE* pBuf = NULL;
	BYTE  header[80];

	int totalSize = 0;
	int imageSize = (int)pow(2.0, 22); // 4 Mbytes
	int segCount = 0;
	int numRead;
  int retValue = 0;

  m_fpgaBlockCount = 0;

	if (fpStatus == 0)
	{
		// read the file into the buffer
		m_FPGAFlashBlocks[0].m_pData = (BYTE*)realloc(m_FPGAFlashBlocks[0].m_pData, imageSize);		
		pBuf = m_FPGAFlashBlocks[0].m_pData;

		numRead = (int)fread(header, 1, 80, fp);

		while (!feof(fp))
		{
			numRead = (int)fread(&pBuf[totalSize], 1, 1024, fp);

			totalSize += numRead;
			segCount++;
		}

    m_fpgaBlockCount = segCount;

		fclose(fp);

		// bit swap
		for (int i = 0; i < totalSize; i++)
		{
			BYTE b1 = pBuf[i];
			BYTE b2 = 0;
			for (int j = 0; j < 8; j++)
			{
				b2 <<= 1;
				b2 = b2 | (b1 & 0x01);				
				b1 >>= 1;				
			}
			pBuf[i] = b2;
		}

    if (m_programMain)
    {
  		m_FPGAFlashBlocks[0].m_addr = 0x400000; // 4 Mbytes
  	}
  	else
    if (m_programBoot)
  	{
  		m_FPGAFlashBlocks[0].m_addr = 0x000000; // 4 Mbytes
  	}
  	else
  	{
  	  retValue = GEM_FPGAPROG_NO_ADDR;
  	}
  	
		m_FPGAFlashBlocks[0].m_size = totalSize;

		if (m_FPGAFlashBlocks[1].m_pData) 
		{
			free(m_FPGAFlashBlocks[1].m_pData);
		}

		m_FPGAFlashBlocks[1].m_pData = 0;
	}
  else
  {
    retValue = GEM_FPGAPROG_NO_FILE;
  }
  
  return retValue;
}

//----------------------------------------------------------------------------
void CGeminiNetwork::AbortFPGAProgramming(void)
{
  m_abortFPGAProgramming = true;
}

//----------------------------------------------------------------------------
void PROGRAMFPGATASK(void *pClassPtr)
{
  CGeminiNetwork *pGeminiNetwork = (CGeminiNetwork *)pClassPtr;	

  pGeminiNetwork->ProgramFPGACode();
}

//----------------------------------------------------------------------------
void CGeminiNetwork::ProgramFPGA(char *fileName, char *destination)
{
  // Original filename used in code was
  // "C:\\Work\\Development\\Tasks\\D248_Gemini\\Source\\Xilinx\\Gemini\\project\\hfmultibeam.bit"
  
  m_programMain = false;
  m_programBoot = false;

  // Determine what we are programming

  if (strcmp(destination, "MAIN") == 0)
  {
    m_programMain = true;
    m_programBoot = false;
  }
  else
  if (strcmp(destination, "BOOT") == 0)
  {
    m_programMain = false;
    m_programBoot = true;
  }

  // Make sure a valid area was selected

  if ((m_programMain == false) &&
      (m_programBoot == false))
  {
    if (m_progressCallback)
      m_progressCallback(GEM_FPGAPROG_NO_ADDR, 0, 0, 0, 0, 0);
  }
  else
  {  
    // Attempt to download the bit file
    if (DownloadBitFile(fileName) == 0)
    {
      if (m_progressCallback)
        m_progressCallback(GEM_FPGAPROG_FILE_SIZE, 0, 0, 0, m_fpgaBlockCount, 0);

      m_abortFPGAProgramming = false;
#ifdef _WIN32
      _beginthread(PROGRAMFPGATASK, 0, (void *)this);
#else
		pthread_t hFPGAID;
		pthread_create(&hFPGAID, NULL, (void*(*)(void*))PROGRAMFPGATASK, this);
#endif
    }
    else
    {
      if (m_progressCallback)
        m_progressCallback(GEM_FPGAPROG_NO_FILE, 0, 0, 0, 0, 0);
    }
  }
}

//----------------------------------------------------------------------------
void CGeminiNetwork::ProgramFPGACode(void)
{
#ifdef _WIN32
	int goodCount   = 0;
	int badCount    = 0;
	int sent        = 0;
  int sentPackets = 0;
  int block       = 0;
  int tryCount    = 0;
  int differCount = 0;

  bool succeeded  = false;

  m_FPGAProgrammingInProgress = true;

  if (m_progressCallback)
    m_progressCallback(GEM_FPGAPROG_START, 0, 0, 0, 0, 0);

	CGemFlashWritePacket fp;
	fp.m_flags = 0x01; // bit 0 initiates a readback
				
	sent        = 0;
	sentPackets = 0;

	unsigned startAddress = m_FPGAFlashBlocks[block].m_addr; // specified in bytes

	// Check that the start address is on a block boundary to ensure proper flash erase
	if (startAddress & 0x0001ffff)
	{
    if (m_progressCallback)
      m_progressCallback(GEM_FPGAPROG_NOT_ALIGNED, tryCount, sentPackets, block, 1, (int)startAddress);

    m_FPGAProgrammingInProgress = false;

    return;
	}

  // OK, we are happy to try and program the FPGA code

	fp.m_addr   = startAddress;
	BYTE* pBuf  = m_FPGAFlashBlocks[0].m_pData;
	int toSend  = m_FPGAFlashBlocks[0].m_size;

	// Clear the messages in case they were left set by another flash read
	WaitForSingleObject(hFlashResultReceived, 0);
	WaitForSingleObject(hFlashReadbackReceived, 0);

	while (toSend > 0)
	{
    // Check to see if the programming has been aborted
    if (m_abortFPGAProgramming == true)
    {
      if (m_progressCallback)
        m_progressCallback(GEM_FPGAPROG_ABORTED, 0, 0, 0, 0, 0);
        
      m_FPGAProgrammingInProgress = false;

      return;
    }
    
		// We will be sending in chunks of 1024 bytes
		int numSend = (std::min)(1024, toSend);

		memcpy(fp.m_data, pBuf, numSend);

		int numPad = 1024 - numSend;

		if (numPad)
			memset(&fp.m_data[numSend], 0xff, numPad);

	  tryCount  = 0;
    succeeded = false;
    
    while ((tryCount <= m_flashProgRetryCount) && !succeeded)
    {
		  SendMsg(&fp, sizeof(fp));

		  // Wait for the signal indicating a flash result under timeout
		  if (WaitForSingleObject(hFlashResultReceived, 5000) != WAIT_TIMEOUT)
		  {
        // Check the result
        if ((m_flashResult.m_result & 0x7fff) != 0)
        {
          // Flash failed to program (according to Gemini)

          if (m_progressCallback)
            m_progressCallback(GEM_FPGAPROG_FAILURE, tryCount, sentPackets, block, 1, (int)m_flashResult.m_result);

		      badCount++;
        }

	      // Wait for the signal indicating a flash readback under timeout
	      if (WaitForSingleObject(hFlashReadbackReceived, 1000) != WAIT_TIMEOUT)
	      {
          // Flash programmed OK (according to Gemini)
		      // Compare the flash result with the original sent flash packet
		      if (memcmp(fp.m_data, m_flashReadback.m_data, 1024))
		      {				
            // Compare to original data failed

            differCount = 0;

            for (int i = 0; i < 1024; i++)
            {
              if (fp.m_data[i] != m_flashReadback.m_data[i])
                differCount++;
            }

            if (m_progressCallback)
              m_progressCallback(GEM_FPGAPROG_BAD_COMPARE, tryCount, sentPackets, block, 1, differCount);

			      badCount++;
		      }
		      else
		      {
		        // Programmed and compared successfully - we can say the programming was successful
		        
            if (m_progressCallback)
              m_progressCallback(GEM_FPGAPROG_SUCCESS, tryCount, sentPackets, block, 1, 0);

            // Stop retry mechanism. As we succeeded, we don't need to retry
            succeeded = true;

			      goodCount++;
            sentPackets++;
		      }
        }
	      else
	      {
			    // Timed out waiting for readback packet

          if (m_progressCallback)
            m_progressCallback(GEM_FPGAPROG_TIMEOUT, tryCount, sentPackets, block, 1, 0);
	      }
		  } 
		  else
		  {
		    // Timed out waiting for result packet

        if (m_progressCallback)
          m_progressCallback(GEM_FPGAPROG_TIMEOUT, tryCount, sentPackets, block, 1, 0);
		  }
		  
		  tryCount++;
    }
    
    // Did we actually program the block?
    if (succeeded == false)
    {
      // No, stop the sending loop
		  toSend    = 0;
    }
    else
    {
		  pBuf      += numSend;
		  sent      += numSend;
		  toSend    -= numSend;
		  fp.m_addr += 1024;
    }
	}

  if (m_progressCallback)
  {
    if (succeeded == false)
    {
      m_progressCallback(GEM_FPGAPROG_FAILED, 0, sentPackets, block, 1, 0);
    }
    else
    {  
      m_progressCallback(GEM_FPGAPROG_COMPLETE, 0, sentPackets, block, 1, 0);
    }
  }
  
  m_FPGAProgrammingInProgress = false;
#endif
}

//----------------------------------------------------------------------------
void CGeminiNetwork::ReadFromFlash(unsigned int addressToRead)
{
	CGemFlashReadPacket fp;
	
	// Copy the addess to be read
	fp.m_addr = addressToRead;

  // Send the message
  SendMsg(&fp, sizeof(fp));
}

//----------------------------------------------------------------------------
void CGeminiNetwork::WriteToFlash(unsigned int addressToWrite, unsigned short readbackFlag, unsigned char *data)
{
	CGemFlashWritePacket fp;
	
	// Ensure address is aligned to a 1K boundary
	fp.m_addr = addressToWrite & 0xFFFFFC00;

  // Trap the info block addresses so that we don't use this mechanism to write them
  if ((fp.m_addr == 0x3E0000) ||
      (fp.m_addr == 0x7C0000) ||
      (fp.m_addr == 0x7E0000))
  {
    // We don't want to program these addresses
    return;
  }
      
  // Do we want the data reading back?
  if (readbackFlag)
    fp.m_flags = 1;

  // Copy the data
  for (int i = 0; i < 1024; i++)
    fp.m_data[i] = data[i];

  // Send the message
  SendMsg(&fp, sizeof(fp));
}

//----------------------------------------------------------------------------
void CGeminiNetwork::ReadFlashInfoBlock(unsigned short blockToRead)
{
	CGemInfoReadPacket fp;
	
	// Copy the addess to be read
	fp.m_blockNo = blockToRead;

  // Send the message
  SendMsg(&fp, sizeof(fp));
}

//----------------------------------------------------------------------------
void CGeminiNetwork::SendTxDriveGPICommand(unsigned short command)
{
	CGemTxDriveGPIPacket fp;
	
	// Set the command
	fp.m_txDrv = command;
	
  // Send the message
  SendMsg(&fp, sizeof(fp));
}

//----------------------------------------------------------------------------
void CGeminiNetwork::SendGPOWriteCommand(unsigned short command)
{
	CGemGPOWritePacket fp;
	
	unsigned short localCommand = command & 0x01;
	
	// Set the command
	fp.m_gpo = localCommand;
	
  // Send the message
  SendMsg(&fp, sizeof(fp));
}

//----------------------------------------------------------------------------
void CGeminiNetwork::SendGPIOReadCommand(void)
{
	CGemGPIOReadPacket fp;
	
  // Send the message
  SendMsg(&fp, sizeof(fp));
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RequestDiagnosticInfo(void)
{
	CGemDiagReadPacket fp;
	
  // Send the message
  SendMsg(&fp, sizeof(fp));
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// Receive packet handlers

//----------------------------------------------------------------------------
void CGeminiNetwork::RxStatus(char* pData)
{
  DWORD waitResult; 

  CGemStatusPacket *pStatus = (CGemStatusPacket *)pData;
  
	if (!m_statusBuf[m_statusIpPtr].m_bLocked)
	{
#ifdef _WIN32
    waitResult = WaitForSingleObject(m_statusBuf[m_statusIpPtr].m_mutex, 1000L); // Wait for 1000mS
#else
	waitResult = WaitForSingleObject(m_statusBuf[m_statusIpPtr].m_mutex, 1000L); // Wait for 1000mS
#endif

    if (waitResult == WAIT_OBJECT_0)
    {
  		m_statusBuf[m_statusIpPtr].m_bLocked = true;

      if ((pStatus->m_linkType & 0x0001) == 0x0001)
      {
        unsigned short downstreamPart1 = (pStatus->m_VDSLDownstreamSpeed1 & 0x00ff) * ((pStatus->m_VDSLDownstreamSpeed1 & 0xff00) >> 8);
        unsigned short downstreamPart2 = (pStatus->m_VDSLDownstreamSpeed2 & 0x00ff) * ((pStatus->m_VDSLDownstreamSpeed2 & 0xff00) >> 8);
        unsigned short upstreamPart1   = (pStatus->m_VDSLUpstreamSpeed1 & 0x00ff)   * ((pStatus->m_VDSLUpstreamSpeed1 & 0xff00) >> 8);
        unsigned short upstreamPart2   = (pStatus->m_VDSLUpstreamSpeed2 & 0x00ff)   * ((pStatus->m_VDSLUpstreamSpeed2 & 0xff00) >> 8);

        double downlinkSpeedMbps = (((double)downstreamPart1 + (double)downstreamPart2) * 67500.0) / 1000000.0;
        double uplinkSpeedMbps   = (((double)upstreamPart1   + (double)upstreamPart2) * 67500.0) / 1000000.0;
      
        // Check the VDSL speeds and see if we need to reconfigure the VDSL link,
        // but only if rate adaption is not in progress and the links are too slow
        // and it is the Sonar we are supposed to be talking to
        if (pStatus->m_sonarId == m_sonarID)
        {
          if (((pStatus->m_linkType & 0x0008) == 0x0000) &&
               ((downlinkSpeedMbps < 2.0) ||
                (uplinkSpeedMbps   < 2.0)))
          {
            // Configure the VDSL via MDIO and trigger a rate adaption
            ConfigureVDSLToCurrentDefault();
          }
        }
      }  

      // Tell calling program about the status message  
      if (m_dataHandlerFunction)
        m_dataHandlerFunction(GEM_STATUS, -1, pData);
     
#ifdef _WIN32 
	    if (ReleaseMutex(m_statusBuf[m_statusIpPtr].m_mutex) != 0)
#else
		 if (ReleaseMutex(m_statusBuf[m_statusIpPtr].m_mutex) != 0)
#endif
  	    m_statusBuf[m_statusIpPtr].m_bLocked = false;

	    m_statusIpPtr = (m_statusIpPtr + 1) % BUF_SIZE;	
    }
	}
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RxPingHead(char* pData)
{
  CGemPingHead *pHeader = (CGemPingHead*)pData;

  DWORD waitResult = 0;

  unsigned short i;

	// Check the current structure is not locked
	if (m_tgtImgBuf[m_tgtImgIpPtr].m_bLocked)
  {
    // The structure was locked, we may have missed the previous tail
    // Rather than just block the system, we will release the mutex, and 
    // effectiveley start over again, which will replace the partial (tailless)
    // image we have, with the next one.
    
#ifdef _WIN32
	  if (ReleaseMutex(m_tgtImgBuf[m_tgtImgIpPtr].m_mutex) != 0)
	  	m_tgtImgBuf[m_tgtImgIpPtr].m_bLocked = false;
	  waitResult = WaitForSingleObject(m_tgtImgBuf[m_tgtImgIpPtr].m_mutex, 1000L); // Wait for 1000mS
#else
	  if (ReleaseMutex(m_tgtImgBuf[m_tgtImgIpPtr].m_mutex) != 0)
	  	m_tgtImgBuf[m_tgtImgIpPtr].m_bLocked = false;
	  waitResult = WaitForSingleObject(m_tgtImgBuf[m_tgtImgIpPtr].m_mutex, 1000L); // Wait for 1000mS
#endif
  	  
  }
  if (waitResult == WAIT_OBJECT_0)
  {
		m_tgtImgBuf[m_tgtImgIpPtr].m_bLocked = true;

    m_tgtImgBuf[m_tgtImgIpPtr].m_RLEThresholdUsed     = pHeader->m_RLEThresholdUsed;
    m_tgtImgBuf[m_tgtImgIpPtr].m_rangeCompressionUsed = pHeader->m_rangeCompressionUsed;
		m_tgtImgBuf[m_tgtImgIpPtr].m_extModeUsed          = pHeader->m_extMode;			
    m_tgtImgBuf[m_tgtImgIpPtr].m_numBeamsUsed         = pHeader->m_numBeams;
    m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed        = pHeader->m_endRange - pHeader->m_startRange;
    m_tgtImgBuf[m_tgtImgIpPtr].m_lastRxLineNo         = -1; // Will need changing if startRange ever enabled in head ToDo::
    m_tgtImgBuf[m_tgtImgIpPtr].m_pingStarted          = true;
    m_tgtImgBuf[m_tgtImgIpPtr].m_queuedTailRetry      = false;
    
    // Build the retry queue for this size of data, allowing a maximum of PACKET_RETRY_COUNT retries per line
    
    m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr        = (signed short *)malloc(m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed * 
                                                                               sizeof(signed short));
    m_tgtImgBuf[m_tgtImgIpPtr].m_firstPassRetries     = 0;
    m_tgtImgBuf[m_tgtImgIpPtr].m_secondPassRetries    = 0;
    m_tgtImgBuf[m_tgtImgIpPtr].m_tailRetries          = 0;
        
    if (m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr != NULL)
    {
      for (i = 0; i < m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed; i++)
      {
        m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr[i] = PACKET_RETRY_COUNT;
      }
    }
        
    if ((m_geminiCommsMode == geminiCommsModeSeaNetComp) ||
        (m_geminiCommsMode == geminiCommsModeEvoComp))
    {
      // In the DLL's compressed modes, the aim is to return fewer lines to the calling program.
      // To do this, the DLL, when sending the Ping Config will have turned on range
      // compression in the head so that fewer lines are returned (the head compressing the
      // data. As we don't want to expand these again before they are returned, force
      // the DLL code to treat them as normal (not compressed) data.
      m_tgtImgBuf[m_tgtImgIpPtr].m_outputLineMultiplier = 1;

      switch (m_tgtImgBuf[m_tgtImgIpPtr].m_rangeCompressionUsed & 0x07)
      {
        case 0 :
        default :
          break;

        case 1 :
          m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed /= 2;
          break;

        case 2 :
          m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed /= 4;
          break;

        case 3 :
          m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed /= 8;
          break;

        case 4 :
          m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed /= 16;
          break;
      }
    }
    else
    {
      // Normal mode handling for non-compressed data modes

      switch (m_tgtImgBuf[m_tgtImgIpPtr].m_rangeCompressionUsed & 0x07)
      {
        case 0 :
          m_tgtImgBuf[m_tgtImgIpPtr].m_outputLineMultiplier = 1;
          break;

        case 1 :
          m_tgtImgBuf[m_tgtImgIpPtr].m_outputLineMultiplier = 2;
          m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed /= 2;
          break;

        case 2 :
          m_tgtImgBuf[m_tgtImgIpPtr].m_outputLineMultiplier = 4;
          m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed /= 4;
          break;

        case 3 :
          m_tgtImgBuf[m_tgtImgIpPtr].m_outputLineMultiplier = 8;
          m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed /= 8;
          break;

        case 4 :
          m_tgtImgBuf[m_tgtImgIpPtr].m_outputLineMultiplier = 16;
          m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed /= 16;
          break;

        default :
          m_tgtImgBuf[m_tgtImgIpPtr].m_outputLineMultiplier = 1;
          break;
      }
    }
    
    if (m_dataHandlerFunction)
    {
      if ((m_geminiCommsMode == geminiCommsModeEvo) ||
          (m_geminiCommsMode == geminiCommsModeEvoComp) ||
          (m_geminiCommsMode == geminiCommsModeProd))
      {
        m_dataHandlerFunction(PING_HEAD, -1, pData);
      }
      else
      if ((m_geminiCommsMode == geminiCommsModeSeaNet) ||
          (m_geminiCommsMode == geminiCommsModeSeaNetComp))
      {
        // Malloc enough space to hold the image we are going to receive (as a 16 bit image)
        // including space for any range decompression which is going to take place.
             
        m_dataHandlerFunction(PING_HEAD, -1, pData);

        m_tgtImgBuf[m_tgtImgIpPtr].m_dataPtr = 
              (unsigned short *)malloc(m_tgtImgBuf[m_tgtImgIpPtr].m_numBeamsUsed * 
                                       m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed * 2 *
                                       m_tgtImgBuf[m_tgtImgIpPtr].m_outputLineMultiplier);
      }
    }
  }
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RxPingLine(int len, char* pData)
{
  int l;

 	CGemPingLine *pLine = (CGemPingLine*)pData;
  CGemInternalPingLine outLine;

  bool useDecodedData = false;

  unsigned char *pD;
  unsigned char  in;
  unsigned short inCount;
  unsigned short outCount = 0;

  // If this buffer is not locked, we have not received a ping header, so ignore this data
	if (!m_tgtImgBuf[m_tgtImgIpPtr].m_bLocked)
		return;

  // Upper bit of line number indicates that it is a retry response
  unsigned short originalLineID = pLine->m_lineID & 0x7fff;

  // First check is if the line number received is within the range that the ping
  // header indicated we would be receiving. There have been instances were it has
  // exceeded this (whisper it quietly, due to a hardware bug). Therefore to protect
  // the rest of the software, disregard any lines were this is the case.
  
  if (originalLineID >= m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed)
  {
    // We are in unknown territory, we have received more lines than we were told to expect
    // Run, run now!!
    return;
  }
  
  // Check for missing lines, but only on non retry response data
  if ((pLine->m_lineID & 0x8000) == 0)
  { 
    // Upper bit is not set, this is not a response to a retry request
    // See if there are any lines missing since the last one received
   
    while (pLine->m_lineID > m_tgtImgBuf[m_tgtImgIpPtr].m_lastRxLineNo + 1)
    {
      // Can we retry the missing lines? We need the retry queue to be valid, and the counter for
      // this line to be greater than zero for this to be so.    
      
      if (m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr != NULL)
      {
        if (m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr[m_tgtImgBuf[m_tgtImgIpPtr].m_lastRxLineNo + 1] > 0)
        {
          RequestRetry(GEM_RETRY_REQUEST_LINE, m_tgtImgBuf[m_tgtImgIpPtr].m_lastRxLineNo + 1, 1);
          m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr[m_tgtImgBuf[m_tgtImgIpPtr].m_lastRxLineNo + 1]--;
          m_tgtImgBuf[m_tgtImgIpPtr].m_firstPassRetries++;
        }      
      }
      
      m_tgtImgBuf[m_tgtImgIpPtr].m_lastRxLineNo++;
    }

    // Make sure the last received line number stays in sync
        
    m_tgtImgBuf[m_tgtImgIpPtr].m_lastRxLineNo = (int)pLine->m_lineID;
  }
  
  // Handle data

  if (m_dataHandlerFunction)
  {
    // Ignore retry flag in data
    
    pLine->m_lineID &= 0x7FFF;
    
    // Log the fact that this line is OK by setting the retry count for that line to -1
    
    if (m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr != NULL)
      m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr[pLine->m_lineID] = -1;

    // Has data been run length encoded? (Only if 8 bit data)
    
    if ((m_tgtImgBuf[m_tgtImgIpPtr].m_RLEThresholdUsed > 0) &&
        ((m_tgtImgBuf[m_tgtImgIpPtr].m_extModeUsed & 0x04) == 0x04))
    {
      // Remove RLE compression from data
      
     	CGemPingLine *pLine = (CGemPingLine*)pData;

      outLine.m_gain      = pLine->m_gain;
      outLine.m_pingID    = pLine->m_pingID;
      outLine.m_lineID    = pLine->m_lineID & 0x7fff;
      outLine.m_scale     = pLine->m_scale;
      outLine.m_lineInfo  = pLine->m_lineInfo;
     	
      pD = &pLine->m_startOfData;

      while ((pD < (unsigned char *)pLine + len) && (outCount < m_tgtImgBuf[m_tgtImgIpPtr].m_numBeamsUsed))
      {
        in = *pD++;

        if (in == 0)
        {
          inCount = (unsigned short)*pD++;
          
          // Initial bug (#11) in Gemini head meant 00 00 => 256 zeroes, has been corrected
          //if (inCount == 0)
          //  inCount = 256;
          
          for (int k = 0; k < inCount; k++)
          {
            outLine.m_data[outCount] = 0;
            
            if (outCount < m_tgtImgBuf[m_tgtImgIpPtr].m_numBeamsUsed)
              outCount++;
          }
        }
        else
        if (in == 1)
        {
          outLine.m_data[outCount] = 0;
          outCount++;
        }
        else
        {
          outLine.m_data[outCount] = in;
          outCount++;
        }
      }

      useDecodedData = true;

    }

    // Deal with range compression
    
    for (int i = 0; i < m_tgtImgBuf[m_tgtImgIpPtr].m_outputLineMultiplier; i++)
    { 

      // Have we had to remove RLE?

      if (useDecodedData)
      {
        outLine.m_lineID = (originalLineID * m_tgtImgBuf[m_tgtImgIpPtr].m_outputLineMultiplier) + i; 

        if ((m_geminiCommsMode == geminiCommsModeEvo) ||
            (m_geminiCommsMode == geminiCommsModeEvoComp) ||
            (m_geminiCommsMode == geminiCommsModeProd))
        {
          m_dataHandlerFunction(PING_DATA, outCount, (char *)&outLine);
        }
        else
        if ((m_geminiCommsMode == geminiCommsModeSeaNet) ||
            (m_geminiCommsMode == geminiCommsModeSeaNetComp))
        {
          // As this has been decompressed, we know it is only 8 bit incoming data
          for (l = 0; l < m_tgtImgBuf[m_tgtImgIpPtr].m_numBeamsUsed; l++)
          {
            *(m_tgtImgBuf[m_tgtImgIpPtr].m_dataPtr + (m_tgtImgBuf[m_tgtImgIpPtr].m_numBeamsUsed * (pLine->m_lineID & 0x7fff)) + l) =
              (unsigned short)outLine.m_data[l];
          }
        }                
      }
      else
      { 
        pLine->m_lineID = (originalLineID * m_tgtImgBuf[m_tgtImgIpPtr].m_outputLineMultiplier) + i; 
          
        if ((m_geminiCommsMode == geminiCommsModeEvo) ||
            (m_geminiCommsMode == geminiCommsModeEvoComp) ||
            (m_geminiCommsMode == geminiCommsModeProd))
        {
          m_dataHandlerFunction(PING_DATA, m_tgtImgBuf[m_tgtImgIpPtr].m_numBeamsUsed, pData);
        }  
        else
        if ((m_geminiCommsMode == geminiCommsModeSeaNet) ||
            (m_geminiCommsMode == geminiCommsModeSeaNetComp))
        {
          for (l = 0; l < m_tgtImgBuf[m_tgtImgIpPtr].m_numBeamsUsed; l++)
          {
            if ((m_tgtImgBuf[m_tgtImgIpPtr].m_extModeUsed & 0x04) == 0x04)
            {
              // 8 bit incoming data
              *(m_tgtImgBuf[m_tgtImgIpPtr].m_dataPtr + (m_tgtImgBuf[m_tgtImgIpPtr].m_numBeamsUsed * (pLine->m_lineID & 0x7fff)) + l) =
                (unsigned short)*(&pLine->m_startOfData + l);
            }
            else
            {
              // 16 bit incoming data
              *(m_tgtImgBuf[m_tgtImgIpPtr].m_dataPtr + (m_tgtImgBuf[m_tgtImgIpPtr].m_numBeamsUsed * (pLine->m_lineID & 0x7fff)) + l) =
                (*(&pLine->m_startOfData + (l * 2)) << 8) + *(&pLine->m_startOfData + (l * 2) + 1);
            }
          }
        }                
      }
    }
  }
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RxPingTail(char* pData)
{
  CGemBearingData       outputData;
  CGemPingTail         *pTail = (CGemPingTail *)pData;
  CGemPingTailExtended  exTail;
  
  bool dataComplete = true;
  unsigned short i;
  int linesLostCount;
    
  // If this buffer is not locked, we have not received a ping header, so ignore this ping tail
	if (!m_tgtImgBuf[m_tgtImgIpPtr].m_bLocked)
		return;

  // We have received a Ping Tail message - check to see if all the data has been received
  // If not request the missing lines, and a new ping tail (we will ignore this one to keep
  // the sequencing of the data correct)
  
  // If we request a start range of 0 (default in Gemini head, not currently implemented)
  // and an end range of 1000, we will receive line numbers from 0 to 999 (empirically from
  // inspecting the data).
  
  // Check to see if this is the first ping tail we have received or it is a retry. Any retry
  // requests we have issued will not be processed until after the ping tail (original) has been
  // sent. Therefore if this is the first ping tail, just look to see if we have any oustanding
  // lines. If we have, ask for another ping tail and nothing else. If this is repeated ping tail
  // we can request any lines that are still outstanding and a further ping tail.
  
  if (m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr != NULL)
  {
    // Is this the first ping tail, or a repeated one
    
    if ((pTail->m_flags & 0x80) == 0)
    {
      // This is the first (original) ping tail, check to see if all lines received,
      // this is indicated by the retry value not being zero

      for (i = 0; i < m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed; i++)
      {
        if (m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr[i] > 0)
        {
          // This entry is not complete, therefore the whole image is not complete
          dataComplete = false;
        }
      }

      if (!dataComplete)
      {
        // And request a new tail data so that we will retry this processing
        
        RequestRetry(GEM_RETRY_REQUEST_TAIL, 0, 1);
        m_tgtImgBuf[m_tgtImgIpPtr].m_tailRetries++;
      }
    }
    else
    {
      // This is a repeated ping tail
      // Check to see if there are any outstanding lines, if there are ask again
      
      for (i = 0; i < m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed; i++)
      {
        if (m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr[i] > 0)
        {
          RequestRetry(GEM_RETRY_REQUEST_LINE, i, 1);
          m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr[i]--;
          m_tgtImgBuf[m_tgtImgIpPtr].m_secondPassRetries++;
          dataComplete = false;
        }
      }

      if (!dataComplete)
      {
        // And request a new tail data so that we will retry this processing
        
        RequestRetry(GEM_RETRY_REQUEST_TAIL, 0, 1);
        m_tgtImgBuf[m_tgtImgIpPtr].m_tailRetries++;
      }
    }
  }
  
  // Have we received all the data for this ping?
  if (dataComplete)
  {
    m_tgtImgBuf[m_tgtImgIpPtr].m_pingStarted = false;

    // Count up number of lines where retries ran out before data was received
    
    linesLostCount = 0;
    
    for (i = 0; i < m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed; i++)
    {
      if (m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr != NULL &&
          m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr[i] == 0)
      {
        linesLostCount++; 
      }
    }
    
    // We have received all the data, so do the normal end of ping data processing
    
    if (m_dataHandlerFunction)
    {
      if ((m_geminiCommsMode == geminiCommsModeEvo) ||
          (m_geminiCommsMode == geminiCommsModeEvoComp) ||
          (m_geminiCommsMode == geminiCommsModeProd))
      {
	      if (m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr != NULL)
	      {
	        // Create an extended ping tail message with retry counters appended
	        exTail.m_head.m_type          = pTail->m_head.m_type;
	        exTail.m_head.m_version       = pTail->m_head.m_version;
	        exTail.m_head.m_deviceID      = pTail->m_head.m_deviceID;
	        exTail.m_head.m_packetLatency = pTail->m_head.m_packetLatency;
	        exTail.m_head.m_spare         = pTail->m_head.m_spare;
	        exTail.m_pingID               = pTail->m_pingID & 0x7fff;
	        exTail.m_spare                = pTail->m_spare;
	        exTail.m_firstPassRetries     = m_tgtImgBuf[m_tgtImgIpPtr].m_firstPassRetries;
	        exTail.m_secondPassRetries    = m_tgtImgBuf[m_tgtImgIpPtr].m_secondPassRetries;
	        exTail.m_tailRetries          = m_tgtImgBuf[m_tgtImgIpPtr].m_tailRetries;
	        exTail.m_interMessageGap      = m_commsInterMessageGap;
	        exTail.m_packetCount          = m_packetCount;
	        exTail.m_recvErrorCount       = m_recvErrorCount;
	        exTail.m_linesLostThisPing    = (unsigned long)linesLostCount;
          exTail.m_generalCount         = m_generalCount;
          
          m_dataHandlerFunction(PING_TAIL_EX, -1, (char *)&exTail);
	      }
	      else
	      {
          // Just ensure we don't accidently send up a ping id with the upper bit set
	        pTail->m_pingID &= 0x7fff;

	        // Retry queue was never created, so don't need to send up extended PING_TAIL message
          m_dataHandlerFunction(PING_TAIL, -1, pData);
        }
      }
      else
      if ((m_geminiCommsMode == geminiCommsModeSeaNet) ||
          (m_geminiCommsMode == geminiCommsModeSeaNetComp))
      {
        // We need to process the data block we have built up, and send it to
        // SeaNet as bearing lines not range lines
        
        outputData.m_noSamples = m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed;
        outputData.m_pData     = (unsigned char *)malloc(m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed);
        
        for (unsigned short j = m_pingStartBeam; j <= m_pingEndBeam; j++)
        { 
          outputData.m_bearingLineNo = j;

          if (outputData.m_pData)
          {
            for (unsigned short i = 0; i < m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed; i++)
            {
              *(outputData.m_pData + i) = 
                (unsigned char)*(m_tgtImgBuf[m_tgtImgIpPtr].m_dataPtr + (m_tgtImgBuf[m_tgtImgIpPtr].m_numBeamsUsed * i) + j);
            }
          }
          else
          {
            outputData.m_noSamples = 0;
          }      

          m_dataHandlerFunction(GEM_BEARING_DATA, m_tgtImgBuf[m_tgtImgIpPtr].m_numRangesUsed, (char *)&outputData);

        }

        // Send a PING_TAIL_EX message, so that Seanet has the counter values
	      if (m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr != NULL)
	      {
	        // Create an extended ping tail message with retry counters appended
	        exTail.m_head.m_type          = pTail->m_head.m_type;
	        exTail.m_head.m_version       = pTail->m_head.m_version;
	        exTail.m_head.m_deviceID      = pTail->m_head.m_deviceID;
	        exTail.m_head.m_packetLatency = pTail->m_head.m_packetLatency;
	        exTail.m_head.m_spare         = pTail->m_head.m_spare;
	        exTail.m_pingID               = pTail->m_pingID & 0x7fff;
	        exTail.m_spare                = pTail->m_spare;
	        exTail.m_firstPassRetries     = m_tgtImgBuf[m_tgtImgIpPtr].m_firstPassRetries;
	        exTail.m_secondPassRetries    = m_tgtImgBuf[m_tgtImgIpPtr].m_secondPassRetries;
	        exTail.m_tailRetries          = m_tgtImgBuf[m_tgtImgIpPtr].m_tailRetries;
	        exTail.m_interMessageGap      = m_commsInterMessageGap;
	        exTail.m_packetCount          = m_packetCount;
	        exTail.m_recvErrorCount       = m_recvErrorCount;
	        exTail.m_linesLostThisPing    = (unsigned long)linesLostCount;
          exTail.m_generalCount         = m_generalCount;
          
          m_dataHandlerFunction(PING_TAIL_EX, -1, (char *)&exTail);
	      }

        if (outputData.m_pData)
          free(outputData.m_pData);
      }
    }
    
    // This is the last processing which will be performed for this Ping
    // Look at the number of retries attempted and adaptively vary the inter message gap to 
    // try to improve the communications whilst not making it too big
    
    if (m_tgtImgBuf[m_tgtImgIpPtr].m_tailRetries < 3)
    {
      // Communications are good, reduce the IMG
      if (m_commsInterMessageGap > geminiInterMessageGapMin)
      {
        m_commsInterMessageGap--;
        m_geminiNetworkConfig.m_ethernetIFG = m_commsInterMessageGap;
        m_geminiNetworkConfig.m_VDSLIFG     = m_commsInterMessageGap;
        SendNetworkConfig();
      }
    }
    else
    if (m_tgtImgBuf[m_tgtImgIpPtr].m_tailRetries > 6)
    {
      // Communications are not so good, increase the IMG
      if (m_commsInterMessageGap < geminiInterMessageGapMax)
      {
        m_commsInterMessageGap++;
        m_geminiNetworkConfig.m_ethernetIFG = m_commsInterMessageGap;
        m_geminiNetworkConfig.m_VDSLIFG     = m_commsInterMessageGap;
        SendNetworkConfig();
      }
    }  

    // Data is complete so we have finished with the retry queue

	  if (m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr != NULL)
	  {
	    delete(m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr);
	    m_tgtImgBuf[m_tgtImgIpPtr].m_retryQueuePtr = NULL;
	  }

    if (m_tgtImgBuf[m_tgtImgIpPtr].m_dataPtr != NULL)
    {
      delete(m_tgtImgBuf[m_tgtImgIpPtr].m_dataPtr);
      m_tgtImgBuf[m_tgtImgIpPtr].m_dataPtr = NULL;
    }

#ifdef _WIN32
	  if (ReleaseMutex(m_tgtImgBuf[m_tgtImgIpPtr].m_mutex) != 0)
#else
	  if (ReleaseMutex(m_tgtImgBuf[m_tgtImgIpPtr].m_mutex) != 0)
#endif
  	  m_tgtImgBuf[m_tgtImgIpPtr].m_bLocked = false;

	  m_tgtImgIpPtr = (m_tgtImgIpPtr + 1) % BUF_SIZE;
  }
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RxFlashResultPacket(char* pData)
{
  DWORD waitResult; 

  CGemFlashResult* pFlashResult;
  
  pFlashResult = (CGemFlashResult *)pData;

	if (!m_flashResultBuf[m_flashResultIpPtr].m_bLocked)
	{
#ifdef _WIN32
    waitResult = WaitForSingleObject(m_flashResultBuf[m_flashResultIpPtr].m_mutex, 1000L); // Wait for 1000mS
#else
	 waitResult = WaitForSingleObject(m_flashResultBuf[m_flashResultIpPtr].m_mutex, 1000L); // Wait for 1000mS
#endif

    if (waitResult == WAIT_OBJECT_0)
    {
  		m_flashResultBuf[m_flashResultIpPtr].m_bLocked = true;

      // Copy block into local storage
      m_flashResult.m_head.m_deviceID       = pFlashResult->m_head.m_deviceID;
      m_flashResult.m_head.m_packetLatency  = pFlashResult->m_head.m_packetLatency;
      m_flashResult.m_head.m_spare          = pFlashResult->m_head.m_spare;
      m_flashResult.m_head.m_type           = pFlashResult->m_head.m_type;
      m_flashResult.m_head.m_version        = pFlashResult->m_head.m_version;
      m_flashResult.m_address               = pFlashResult->m_address;
      m_flashResult.m_result                = pFlashResult->m_result;
      m_flashResult.m_spare                 = pFlashResult->m_spare;
        
#ifdef _WIN32
      // And let whoever needs to know that call has occurred
      if (hFlashResultReceived)
        SetEvent(hFlashResultReceived);
#endif

      if (m_dataHandlerFunction)
      {
        if ((m_geminiCommsMode == geminiCommsModeEvo) ||
            (m_geminiCommsMode == geminiCommsModeEvoComp) ||
            (m_geminiCommsMode == geminiCommsModeProd))
        {
            m_dataHandlerFunction(GEM_FLASH_RESULT, -1, pData);
        }
        else
        if ((m_geminiCommsMode == geminiCommsModeSeaNet) ||
            (m_geminiCommsMode == geminiCommsModeSeaNetComp))
        {
        
        }                
      }
#ifdef _WIN32
	    if (ReleaseMutex(m_flashResultBuf[m_flashResultIpPtr].m_mutex) != 0)
#else
		 if (ReleaseMutex(m_flashResultBuf[m_flashResultIpPtr].m_mutex) != 0)
#endif
  	    m_flashResultBuf[m_flashResultIpPtr].m_bLocked = false;

	    m_flashResultIpPtr = (m_flashResultIpPtr + 1) % BUF_SIZE;	
    }
	}
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RxFlashReadbackPacket(char* pData)
{
  DWORD waitResult; 

  CGemFlashReadback* pFlashReadback;
  
  pFlashReadback = (CGemFlashReadback *)pData;

	if (!m_flashReadbackBuf[m_flashReadbackIpPtr].m_bLocked)
	{
#ifdef _WIN32
    waitResult = WaitForSingleObject(m_flashReadbackBuf[m_flashReadbackIpPtr].m_mutex, 1000L); // Wait for 1000mS
#else
	 waitResult = WaitForSingleObject(m_flashReadbackBuf[m_flashReadbackIpPtr].m_mutex, 1000L); // Wait for 1000mS
#endif

    if (waitResult == WAIT_OBJECT_0)
    {
  		m_flashReadbackBuf[m_flashReadbackIpPtr].m_bLocked = true;

      // Copy block into local storage
      m_flashReadback.m_head.m_deviceID       = pFlashReadback->m_head.m_deviceID;
      m_flashReadback.m_head.m_packetLatency  = pFlashReadback->m_head.m_packetLatency;
      m_flashReadback.m_head.m_spare          = pFlashReadback->m_head.m_spare;
      m_flashReadback.m_head.m_type           = pFlashReadback->m_head.m_type;
      m_flashReadback.m_head.m_version        = pFlashReadback->m_head.m_version;
      m_flashReadback.m_address               = pFlashReadback->m_address;
      m_flashReadback.m_spare                 = pFlashReadback->m_spare;

      for (int i = 0; i < 1024; i++)
        m_flashReadback.m_data[i] = pFlashReadback->m_data[i];
      
#ifdef _WIN32
      // And let whoever needs to know that call has occurred
      if (hFlashReadbackReceived)
        SetEvent(hFlashReadbackReceived);
#endif

      if (m_dataHandlerFunction)
      {
        if ((m_geminiCommsMode == geminiCommsModeEvo) ||
            (m_geminiCommsMode == geminiCommsModeEvoComp) ||
            (m_geminiCommsMode == geminiCommsModeProd))
        {
            m_dataHandlerFunction(GEM_FLASH_READBACK, -1, pData);
        }
        else
        if ((m_geminiCommsMode == geminiCommsModeSeaNet) ||
            (m_geminiCommsMode == geminiCommsModeSeaNetComp))
        {
        
        }                
      }

#ifdef _WIN32
	    if (ReleaseMutex(m_flashReadbackBuf[m_flashReadbackIpPtr].m_mutex) != 0)
#else
		 if (ReleaseMutex(m_flashReadbackBuf[m_flashReadbackIpPtr].m_mutex) != 0)
#endif
  	    m_flashReadbackBuf[m_flashReadbackIpPtr].m_bLocked = false;

	    m_flashReadbackIpPtr = (m_flashReadbackIpPtr + 1) % BUF_SIZE;	
    }
	}
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RxScopePacket(char* pData)	
{

}

//----------------------------------------------------------------------------
void CGeminiNetwork::RxVelPacket(char* pData)
{
  DWORD waitResult; 

	if (!m_velocimeterBuf[m_velocimeterIpPtr].m_bLocked)
	{
#ifdef _WIN32
    waitResult = WaitForSingleObject(m_velocimeterBuf[m_velocimeterIpPtr].m_mutex, 1000L); // Wait for 1000mS
#else
	 waitResult = WaitForSingleObject(m_velocimeterBuf[m_velocimeterIpPtr].m_mutex, 1000L); // Wait for 1000mS
#endif

    if (waitResult == WAIT_OBJECT_0)
    {
  		m_velocimeterBuf[m_velocimeterIpPtr].m_bLocked = true;

      if ((m_geminiCommsMode == geminiCommsModeEvo) ||
          (m_geminiCommsMode == geminiCommsModeEvoComp) ||
          (m_geminiCommsMode == geminiCommsModeProd))
      {
        if (m_dataHandlerFunction)
        {
          m_dataHandlerFunction(GEM_VELOCIMETER_DATA, -1, pData);
        }
      }
      else
      if ((m_geminiCommsMode == geminiCommsModeSeaNet) ||
          (m_geminiCommsMode == geminiCommsModeSeaNetComp))
      {
      }
      
#ifdef _WIN32
	    if (ReleaseMutex(m_velocimeterBuf[m_velocimeterIpPtr].m_mutex) != 0)
#else
		 if (ReleaseMutex(m_velocimeterBuf[m_velocimeterIpPtr].m_mutex) != 0)
#endif
  	    m_velocimeterBuf[m_velocimeterIpPtr].m_bLocked = false;

	    m_velocimeterIpPtr = (m_velocimeterIpPtr + 1) % BUF_SIZE;	
    }
	}
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RxSerialPacket(int len, char* pData)
{
/* NO RS232 IN LINUX JUST NOW*/
#ifdef _WIN32
  DWORD waitResult; 
  CGemRS232RXPacket *pSerial;

  char               lData[1024];

  int                length = 0;
  
  TIMESTAMP_UNION    serialDataTimestamp;

  unsigned long long timeuSec = 0;

	if (!m_serialBuf[m_serialIpPtr].m_bLocked)
	{
#ifdef _WIN32
    waitResult = WaitForSingleObject(m_serialBuf[m_serialIpPtr].m_mutex, 1000L); // Wait for 1000mS
#else
	 waitResult = WaitForSingleObject(m_serialBuf[m_serialIpPtr].m_mutex, 1000L); // Wait for 1000mS
#endif

    if (waitResult == WAIT_OBJECT_0)
    {
  		m_serialBuf[m_serialIpPtr].m_bLocked = true;

      if ((m_geminiCommsMode == geminiCommsModeEvo) ||
          (m_geminiCommsMode == geminiCommsModeEvoComp) ||
          (m_geminiCommsMode == geminiCommsModeProd))
      {
        if (m_dataHandlerFunction)
        {
          m_dataHandlerFunction(GEM_SERIAL, len, pData);
        }
      }
      else
      {
        // 8 bytes received - 7 timestamp followed by 1 serial

        pSerial = (CGemRS232RXPacket*)pData;

        length = 0;

        for (int i = 0; i < ((len - 8) / 8); i++)
        {
          for (int j = 0; j < 8; j++)
          {
            serialDataTimestamp.timestampBytes[j] = *(&pSerial->m_data1 + ((i * 8) + j));          
          }

          timeuSec =  (unsigned long long)serialDataTimestamp.timestampData.timestampL + 
                     ((unsigned long long)serialDataTimestamp.timestampData.timestampM << 32) +      
                     ((unsigned long long)serialDataTimestamp.timestampData.timestampH << 48);      

          lData[i] = (unsigned char)(serialDataTimestamp.timestampData.data);

          length++;
        }

        if (m_serialPortOpened == true)
        {
          serial.Write((char *)lData, length);
        }

        // Echo received data back to Gemini (only really useful for testing)

        if (m_serialEchoData == true)
          SendRS232Data(length, (char *)lData);
      }
#ifdef _WIN32
	    if (ReleaseMutex(m_serialBuf[m_serialIpPtr].m_mutex) != 0)
#else
		 if (ReleaseMutex(m_serialBuf[m_serialIpPtr].m_mutex) != 0)
#endif
  	    m_serialBuf[m_serialIpPtr].m_bLocked = false;

	    m_serialIpPtr = (m_serialIpPtr + 1) % BUF_SIZE;	
    }
	}
#endif
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RxAcknowledgePacket(char* pData)
{
  DWORD waitResult; 

	if (!m_ackBuf[m_ackIpPtr].m_bLocked )
	{
#ifdef _WIN32
    waitResult = WaitForSingleObject(m_ackBuf[m_ackIpPtr].m_mutex, 1000L); // Wait for 1000mS
#else
	 waitResult = WaitForSingleObject(m_ackBuf[m_ackIpPtr].m_mutex, 1000L); // Wait for 1000mS
#endif

    if (waitResult == WAIT_OBJECT_0)
  		m_ackBuf[m_ackIpPtr].m_bLocked = true;
	}

  if (m_dataHandlerFunction)
  {
    if ((m_geminiCommsMode == geminiCommsModeEvo) ||
        (m_geminiCommsMode == geminiCommsModeEvoComp) ||
        (m_geminiCommsMode == geminiCommsModeProd))
    {
      m_dataHandlerFunction(GEM_ACKNOWLEDGE, -1, pData);
    }
    else
    if ((m_geminiCommsMode == geminiCommsModeSeaNet) ||
        (m_geminiCommsMode == geminiCommsModeSeaNetComp))
    {

    }                
  }
#ifdef _WIN32
	if (ReleaseMutex(m_ackBuf[m_ackIpPtr].m_mutex) != 0)
#else
	if (ReleaseMutex(m_ackBuf[m_ackIpPtr].m_mutex) != 0)
#endif
  	m_ackBuf[m_ackIpPtr].m_bLocked = false;

	m_ackIpPtr = (m_ackIpPtr + 1) % BUF_SIZE;	
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RxTestResultPacket(char* pData)
{
  DWORD waitResult; 

	if (!m_testResultBuf[m_testResultIpPtr].m_bLocked )
	{
#ifdef _WIN32
    waitResult = WaitForSingleObject(m_testResultBuf[m_testResultIpPtr].m_mutex, 1000L); // Wait for 1000mS
#else
	 waitResult = WaitForSingleObject(m_testResultBuf[m_testResultIpPtr].m_mutex, 1000L); // Wait for 1000mS
#endif

    if (waitResult == WAIT_OBJECT_0)
  		m_testResultBuf[m_testResultIpPtr].m_bLocked = true;
	}

  if (m_dataHandlerFunction)
  {
    if ((m_geminiCommsMode == geminiCommsModeEvo) ||
        (m_geminiCommsMode == geminiCommsModeEvoComp) ||
        (m_geminiCommsMode == geminiCommsModeProd))
    {
      m_dataHandlerFunction(GEM_TEST_RESULT, -1, pData);
    }
    else
    if ((m_geminiCommsMode == geminiCommsModeSeaNet) ||
        (m_geminiCommsMode == geminiCommsModeSeaNetComp))
    {

    }                
  }
  
#ifdef _WIN32
	if (ReleaseMutex(m_testResultBuf[m_testResultIpPtr].m_mutex) != 0)
#else
	if (ReleaseMutex(m_testResultBuf[m_testResultIpPtr].m_mutex) != 0)
#endif
  	m_testResultBuf[m_testResultIpPtr].m_bLocked = false;

	m_testResultIpPtr = (m_testResultIpPtr + 1) % BUF_SIZE;	
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RxMDIOAckPacket(char* pData)
{
  DWORD waitResult; 

	if (!m_MDIOAckBuf[m_MDIOAckIpPtr].m_bLocked )
	{
#ifdef _WIN32
    waitResult = WaitForSingleObject(m_MDIOAckBuf[m_MDIOAckIpPtr].m_mutex, 1000L); // Wait for 1000mS
#else
	 waitResult = WaitForSingleObject(m_MDIOAckBuf[m_MDIOAckIpPtr].m_mutex, 1000L); // Wait for 1000mS
#endif

    if (waitResult == WAIT_OBJECT_0)
  		m_MDIOAckBuf[m_MDIOAckIpPtr].m_bLocked = true;
	}

  if (m_dataHandlerFunction)
  {
    if ((m_geminiCommsMode == geminiCommsModeEvo) ||
        (m_geminiCommsMode == geminiCommsModeEvoComp) ||
        (m_geminiCommsMode == geminiCommsModeProd))
    {
      m_dataHandlerFunction(GEM_MDIO_ACK, -1, pData);
    }
    else
    if ((m_geminiCommsMode == geminiCommsModeSeaNet) ||
        (m_geminiCommsMode == geminiCommsModeSeaNetComp))
    {

    }                
  }
  
#ifdef _WIN32
	if (ReleaseMutex(m_MDIOAckBuf[m_MDIOAckIpPtr].m_mutex) != 0)
#else
	if (ReleaseMutex(m_MDIOAckBuf[m_MDIOAckIpPtr].m_mutex) != 0)
#endif
  	m_MDIOAckBuf[m_MDIOAckIpPtr].m_bLocked = false;

	m_MDIOAckIpPtr = (m_MDIOAckIpPtr + 1) % BUF_SIZE;	
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RxGPIDataPacket(char* pData)
{
  DWORD waitResult; 

	if (!m_GPIDataBuf[m_GPIDataIpPtr].m_bLocked )
	{
#ifdef _WIN32
    waitResult = WaitForSingleObject(m_GPIDataBuf[m_GPIDataIpPtr].m_mutex, 1000L); // Wait for 1000mS
#else
	 waitResult = WaitForSingleObject(m_GPIDataBuf[m_GPIDataIpPtr].m_mutex, 1000L); // Wait for 1000mS
#endif

    if (waitResult == WAIT_OBJECT_0)
  		m_GPIDataBuf[m_GPIDataIpPtr].m_bLocked = true;
	}

  if (m_dataHandlerFunction)
  {
    if ((m_geminiCommsMode == geminiCommsModeEvo) ||
        (m_geminiCommsMode == geminiCommsModeEvoComp) ||
        (m_geminiCommsMode == geminiCommsModeProd))
    {
      m_dataHandlerFunction(GEM_GPI_DATA, -1, pData);
    }
    else
    if ((m_geminiCommsMode == geminiCommsModeSeaNet) ||
        (m_geminiCommsMode == geminiCommsModeSeaNetComp))
    {

    }                
  }
  
#ifdef _WIN32
	if (ReleaseMutex(m_GPIDataBuf[m_GPIDataIpPtr].m_mutex) != 0)
#else
	if (ReleaseMutex(m_GPIDataBuf[m_GPIDataIpPtr].m_mutex) != 0)
#endif
  	m_GPIDataBuf[m_GPIDataIpPtr].m_bLocked = false;

	m_GPIDataIpPtr = (m_GPIDataIpPtr + 1) % BUF_SIZE;	
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RxDiagDataPacket(char* pData)
{
  DWORD waitResult; 

	if (!m_diagDataBuf[m_diagDataIpPtr].m_bLocked )
	{
#ifdef _WIN32
    waitResult = WaitForSingleObject(m_diagDataBuf[m_diagDataIpPtr].m_mutex, 1000L); // Wait for 1000mS
#else
	 waitResult = WaitForSingleObject(m_diagDataBuf[m_diagDataIpPtr].m_mutex, 1000L); // Wait for 1000mS
#endif

    if (waitResult == WAIT_OBJECT_0)
  		m_diagDataBuf[m_diagDataIpPtr].m_bLocked = true;
	}

  if (m_dataHandlerFunction)
  {
    if ((m_geminiCommsMode == geminiCommsModeEvo) ||
        (m_geminiCommsMode == geminiCommsModeEvoComp) ||
        (m_geminiCommsMode == geminiCommsModeProd))
    {
      m_dataHandlerFunction(GEM_DIAG_DATA, -1, pData);
    }
    else
    if ((m_geminiCommsMode == geminiCommsModeSeaNet) ||
        (m_geminiCommsMode == geminiCommsModeSeaNetComp))
    {

    }                
  }
 
#ifdef _WIN32 
	if (ReleaseMutex(m_diagDataBuf[m_diagDataIpPtr].m_mutex) != 0)
#else
	if (ReleaseMutex(m_diagDataBuf[m_diagDataIpPtr].m_mutex) != 0)
#endif
  	m_diagDataBuf[m_diagDataIpPtr].m_bLocked = false;

	m_diagDataIpPtr = (m_diagDataIpPtr + 1) % BUF_SIZE;	
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RxUnknown(char* pData)
{
  if (m_dataHandlerFunction)
  {
    if ((m_geminiCommsMode == geminiCommsModeEvo) ||
        (m_geminiCommsMode == geminiCommsModeEvoComp) ||
        (m_geminiCommsMode == geminiCommsModeProd))
    {
      m_dataHandlerFunction(GEM_UNKNOWN_DATA, -1, pData);
    }
    else
    if ((m_geminiCommsMode == geminiCommsModeSeaNet) ||
        (m_geminiCommsMode == geminiCommsModeSeaNetComp))
    {

    }                
  }
}

//----------------------------------------------------------------------------
void CGeminiNetwork::RequestRetry(unsigned short type, unsigned short lineNo, unsigned short noOfLines)
{
  CGemRetryRequestPacket request;

  if ((m_geminiCommsMode == geminiCommsModeEvo) ||
      (m_geminiCommsMode == geminiCommsModeEvoComp) ||
      (m_geminiCommsMode == geminiCommsModeSeaNet) ||
      (m_geminiCommsMode == geminiCommsModeSeaNetComp))
  {
    // Build the request  
    request.m_type      = type;
    request.m_lineNo    = lineNo;
    request.m_noOfLines = noOfLines;

    // And send it
	  SendMsg(&request, sizeof(request));
  }
}

//----------------------------------------------------------------------------
void CGeminiNetwork::SendNetworkConfig(void)
{
  // Send the network config structure we hold locally
  SendMsg(&m_geminiNetworkConfig, sizeof(m_geminiNetworkConfig));
}

//----------------------------------------------------------------------------
void CGeminiNetwork::SendMDIOPacket(void)
{
  // Send the network config structure we hold locally
  SendMsg(&m_geminiMDIOPacket, m_geminiMDIOPacket.GetLength());
}

//----------------------------------------------------------------------------
void CGeminiNetwork::ConfigureVDSLToCurrentDefault(void)
{
  if (m_VDSLSettingsToUse == 0)
  {
    ConfigureVDSL(geminiVDSLSettings1DSDataRate,
                  geminiVDSLSettings1USDataRate,
                  geminiVDSLSettings1DSSNR1,
                  geminiVDSLSettings1DSSNR2,
                  geminiVDSLSettings1USSNR1,
                  geminiVDSLSettings1USSNR2,
                  geminiVDSLSettings1DSInterleave,
                  geminiVDSLSettings1USInterleave);
  }
  else
  if (m_VDSLSettingsToUse == 1)
  {
    ConfigureVDSL(geminiVDSLSettings2DSDataRate,
                  geminiVDSLSettings2USDataRate,
                  geminiVDSLSettings2DSSNR1,
                  geminiVDSLSettings2DSSNR2,
                  geminiVDSLSettings2USSNR1,
                  geminiVDSLSettings2USSNR2,
                  geminiVDSLSettings2DSInterleave,
                  geminiVDSLSettings2USInterleave);
  }
  else
  if (m_VDSLSettingsToUse == 2)
  {
    ConfigureVDSL(geminiVDSLSettings3DSDataRate,
                  geminiVDSLSettings3USDataRate,
                  geminiVDSLSettings3DSSNR1,
                  geminiVDSLSettings3DSSNR2,
                  geminiVDSLSettings3USSNR1,
                  geminiVDSLSettings3USSNR2,
                  geminiVDSLSettings3DSInterleave,
                  geminiVDSLSettings3USInterleave);
  }
}

//----------------------------------------------------------------------------
void CGeminiNetwork::ConfigureVDSL(unsigned short DSDataRateMBS, unsigned short USDataRateMBS,
                                   unsigned short DSSNR1,        unsigned short DSSNR2,
                                   unsigned short USSNR1,        unsigned short USSNR2,
                                   unsigned short DSInterleave,  unsigned short USInterleave)
{
  unsigned short calcBitRate;
  unsigned short calcBitRateLS;
  unsigned short calcBitRateMS;

  // Clear the MDIO packet
  m_geminiMDIOPacket.Clear();

  // Force switch to VDSL
  m_geminiMDIOPacket.AddSwitchVDSLCommand();

  // Send the signal to noise ratios
  m_geminiMDIOPacket.AddVDSLRegWriteCommand(0x5b11, DSSNR1 * 4);
  m_geminiMDIOPacket.AddVDSLRegWriteCommand(0x5b12, DSSNR2 * 4);
  m_geminiMDIOPacket.AddVDSLRegWriteCommand(0x5b13, USSNR1 * 4);
  m_geminiMDIOPacket.AddVDSLRegWriteCommand(0x5b14, USSNR2 * 4);

  // Downstream data rate
  calcBitRate = (unsigned short)(((double)DSDataRateMBS * 1024.0) / 67.5); 

  calcBitRateLS =  calcBitRate & 0xff;
  calcBitRateMS = (calcBitRate & 0xff00) >> 8;

  m_geminiMDIOPacket.AddVDSLRegWriteCommand(0x5b19, calcBitRateMS);
  m_geminiMDIOPacket.AddVDSLRegWriteCommand(0x5b1A, calcBitRateLS);

  // Upstream data rate
  calcBitRate = (unsigned short)(((double)USDataRateMBS * 1024.0) / 67.5); 

  calcBitRateLS =  calcBitRate & 0xff;
  calcBitRateMS = (calcBitRate & 0xff00) >> 8;

  m_geminiMDIOPacket.AddVDSLRegWriteCommand(0x5b1B, calcBitRateMS);
  m_geminiMDIOPacket.AddVDSLRegWriteCommand(0x5b1C, calcBitRateLS);

  // Interleaver delays
  m_geminiMDIOPacket.AddVDSLRegWriteCommand(0x5b2f, DSInterleave);
  m_geminiMDIOPacket.AddVDSLRegWriteCommand(0x5b30, USInterleave);

  // Turn on rate adaption and restart it without monitoring
  m_geminiMDIOPacket.AddVDSLRegWriteCommand(0x5b10, 0x03);

  // Send the whole lot to Gemini Sonar to send to the MDIO
  SendMDIOPacket();
}

//----------------------------------------------------------------------------
// This code is called when the IPHELPER task has information. Either the
// code could not setup the NofifyAddrChange function, or the IP address table
// has changed.
void CGeminiNetwork::IPHelperFunction(int codeToAction)
{
  switch (codeToAction)
  {
    case 0: // Task failed to set up code
      break;
          
    case 1: // IP address table has changed
      m_dataHandlerFunction(GEM_IP_CHANGED, 0, NULL);
      break;

    default: // ????
      break;
  }
}

//----------------------------------------------------------------------------
void CGeminiNetwork::ProgramVelocimeterCoeffs(unsigned int CCoeff, unsigned int MCoeff)
{
  // Build the packet to send to the sonar
  
  CGemInfoPacketBank1 newSonarData;
  
  newSonarData.m_velCCoeff = CCoeff;
  newSonarData.m_velMCoeff = MCoeff;
  
  // Actually send the message
  
	SendMsg(&newSonarData, sizeof(newSonarData));

}

//----------------------------------------------------------------------------
#ifndef _WIN32
	HANDLE CreateMutex(HANDLE & hMutex, BOOL bInitialOwner, LPCTSTR lpName)
	{
		// NOTE:
		// 1) On linux you can't initally own a mutex unless we explicitly lock it.
		// 2) On linux mutex cannot be shared amongst processes - we have to use 
		// System V semaphores to achieve this.
		pthread_mutexattr_t hAttr;
		int nResult = 0;
		
		pthread_mutexattr_init(&hAttr);
        #ifndef __APPLE__
		pthread_mutexattr_settype(&hAttr, PTHREAD_MUTEX_RECURSIVE_NP);
        #else
		pthread_mutexattr_settype(&hAttr, PTHREAD_MUTEX_RECURSIVE);
        #endif
		nResult = pthread_mutex_init((pthread_mutex_t*)hMutex, &hAttr);
		pthread_mutexattr_destroy(&hAttr);
		if(nResult!=0)
		{
			return NULL;
		}
		else
		{
			return hMutex;
		}
	}
	
	BOOL ReleaseMutex(HANDLE & hMutex)
	{
		int nResult = 0;
		nResult = pthread_mutex_unlock((pthread_mutex_t*)hMutex);
		return 0 == nResult;
	}	
	
	// NOTE: This only works for MUTEX handles!
	DWORD WaitForSingleObject(HANDLE & hHandle, DWORD dwMilliseconds)
	{
		struct timespec TDelay;	
		DWORD nTimeout = 0;
		int nStatus = 0;
		
		if(dwMilliseconds == (DWORD)INFINITE) // blocking calling
		{
			return pthread_mutex_lock((pthread_mutex_t*)hHandle);
		}
		else if(dwMilliseconds == 0)
		{
			return pthread_mutex_trylock((pthread_mutex_t*)hHandle);
		}
		else // keep trying the lock until timeout!
		{
			while(nTimeout<dwMilliseconds)
			{
				TDelay.tv_sec = 0;
				TDelay.tv_nsec = 1000000; // 1 millisecond delay
				nStatus = pthread_mutex_trylock((pthread_mutex_t*)hHandle);
				if(nStatus == 0)
				{
					return WAIT_OBJECT_0;
				}
				else
				{
					// NOTE: We should probably handle ALL the error conditions 
					// differently but for just now we'll loop until either the 
					// mutex is free or the timeout occurs.
					nanosleep(&TDelay, NULL);
					++nTimeout;
				}
			}
			
			// We will be at this point if we have a timeout.
			return nStatus;
		}
	}
#endif
//----------------------------------------------------------------------------
