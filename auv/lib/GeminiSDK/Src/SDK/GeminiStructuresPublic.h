#ifndef __GEMINISTRUCTURESPUBLIC_H__
#define __GEMINISTRUCTURESPUBLIC_H__

// This file contains Gemini SDK structure definitions

#define PING_HEAD                 0
#define PING_DATA                 1
#define PING_TAIL                 2
#define GEM_STATUS                3
#define GEM_ACKNOWLEDGE           4
#define GEM_SERIAL                5
#define GEM_FLASH_RESULT          6
#define GEM_BEARING_DATA          7
#define GEM_FLASH_READBACK        8
#define GEM_TEST_RESULT           9
#define PING_TAIL_EX              10
#define GEM_IP_CHANGED            11
#define GEM_UNKNOWN_DATA          12
#define GEM_VELOCIMETER_DATA      13
#define GEM_SERIAL_PORT_INPUT     14
#define GEM_MDIO_ACK              15
#define GEM_GPI_DATA              16
#define GEM_DIAG_DATA             17

#define GEM_RETRY_REQUEST_HEAD    0
#define GEM_RETRY_REQUEST_LINE    1
#define GEM_RETRY_REQUEST_TAIL    2

class CGemHdr
{
public:
	unsigned char  m_type;
	unsigned char  m_version;
	unsigned short m_deviceID;
	unsigned short m_packetLatency;
	unsigned short m_spare;

	CGemHdr()
	{
		m_type          = 0;
		m_version       = 1;
		m_deviceID      = 0;
		m_packetLatency = 0;
		m_spare         = 0;
	}
};

class CGemNetMsg
{
public:
	CGemHdr m_head;
};

#pragma pack (push,2)

class CGemStatusPacket: public CGemNetMsg
{
public:
	unsigned short m_firmwareVer;
	unsigned short m_sonarId;
	unsigned int   m_sonarFixIp;
	unsigned int   m_sonarAltIp;
	unsigned int   m_surfaceIp;
	unsigned short m_flags;
	unsigned short m_vccInt;
	unsigned short m_vccAux;
	unsigned short m_dcVolt;
	unsigned short m_dieTemp;
	unsigned short m_tempX;
	unsigned short m_vga1aTemp;
	unsigned short m_vga1bTemp;
	unsigned short m_vga2aTemp;
	unsigned short m_vga2bTemp;
	unsigned short m_psu1Temp;
	unsigned short m_psu2Temp;
	unsigned int   m_currentTimestampL;
	unsigned int   m_currentTimestampH;
	unsigned short m_transducerFrequency;
	unsigned int   m_subnetMask;
	unsigned short m_TX1Temp;
	unsigned short m_TX2Temp;
	unsigned short m_TX3Temp;
	unsigned int   m_BOOTSTSRegister;
	unsigned short m_shutdownStatus;
	unsigned short m_dieOverTemp;
	unsigned short m_vga1aShutdownTemp;
	unsigned short m_vga1bShutdownTemp;
	unsigned short m_vga2aShutdownTemp;
	unsigned short m_vga2bShutdownTemp;
	unsigned short m_psu1ShutdownTemp;
	unsigned short m_psu2ShutdownTemp;
	unsigned short m_TX1ShutdownTemp;
	unsigned short m_TX2ShutdownTemp;
	unsigned short m_TX3ShutdownTemp;
	unsigned short m_linkType;
	unsigned short m_VDSLDownstreamSpeed1;
	unsigned short m_VDSLDownstreamSpeed2;
	unsigned short m_macAddress1;
	unsigned short m_macAddress2;
	unsigned short m_macAddress3;
	unsigned short m_VDSLUpstreamSpeed1;
	unsigned short m_VDSLUpstreamSpeed2;

	CGemStatusPacket()
	{
		m_head.m_type = 0x40;
	}
};

#pragma pack (pop)

class CGemPingHead: public CGemNetMsg
{
public:	
	unsigned short m_pingID;
	unsigned short m_extMode;
	unsigned int   m_transmitTimestampL;
	unsigned int   m_transmitTimestampH;
	unsigned short m_startRange;
	unsigned short m_endRange;
	unsigned int   m_lineTime;
	unsigned short m_numBeams;
	unsigned short m_numChans;
	unsigned char  m_sampChan;
	unsigned char  m_baseGain;
  unsigned short m_spdSndVel;
	unsigned short m_velEchoTime;
	unsigned short m_velEntries;
	unsigned short m_txAngle;
	unsigned short m_sosUsed;
	unsigned char  m_RLEThresholdUsed;
	unsigned char  m_rangeCompressionUsed;

	CGemPingHead()
	{
		m_head.m_type = 0x41;
	}
};

class CGemPingLine: public CGemNetMsg
{
public:	
	unsigned char  m_gain;
	unsigned char  m_pingID;
	unsigned short m_lineID;
	unsigned short m_scale;
	unsigned short m_lineInfo;
  unsigned char  m_startOfData;

	CGemPingLine()
	{
		m_head.m_type = 0x42;
	}	
};

class CGemPingTail: public CGemNetMsg
{
public:	
	unsigned char  m_pingID;
	unsigned char  m_flags;
	unsigned short m_spare;

	CGemPingTail()
	{
		m_head.m_type = 0x43;
	}
};

class CGemPingTailExtended: public CGemPingTail
{
public:
  unsigned short m_firstPassRetries;
  unsigned short m_secondPassRetries;
  unsigned short m_tailRetries;
  unsigned short m_interMessageGap;
  unsigned long  m_packetCount;
  unsigned long  m_recvErrorCount;
  unsigned long  m_linesLostThisPing;
  unsigned long  m_generalCount;
  
	CGemPingTailExtended()
	{
		m_head.m_type       = 0x61;
    m_firstPassRetries  = 0;
    m_secondPassRetries = 0;
    m_tailRetries       = 0;
    m_interMessageGap   = 0;
    m_packetCount       = 0;
    m_recvErrorCount    = 0;
    m_linesLostThisPing = 0;
    m_generalCount      = 0;
	}
};

class CGemAcknowledge: public CGemNetMsg
{
public:	
	unsigned int   m_receiptTimestampL;
	unsigned int   m_receiptTimestampH;
	unsigned int   m_replyTimestampL;
	unsigned int   m_replyTimestampH;

	CGemAcknowledge()
	{
		m_head.m_type = 0x49;
	}
};

class CGemBearingData : public CGemNetMsg
{
public:
  unsigned short m_bearingLineNo;
  unsigned short m_noSamples;
  unsigned char  *m_pData;

	CGemBearingData()
	{
		m_head.m_type   = 0x60;
		m_bearingLineNo = 0;
		m_noSamples     = 0;
		m_pData         = NULL;
	}
};
#endif // __GEMINISTRUCTURESPUBLIC_H__

