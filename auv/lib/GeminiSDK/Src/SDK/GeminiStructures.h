#ifndef __GEMINISTRUCTURES_H__
#define __GEMINISTRUCTURES_H__

// This file contains Gemini structure definitions not made available in the Gemini SDK

// Include the ones that make up those exposed in the SDK
#include "GeminiStructuresPublic.h"

#define GEM_MODPROG_START         0
#define GEM_MODPROG_LOW           GEM_MODPROG_START
#define GEM_MODPROG_SUCCESS       1
#define GEM_MODPROG_FAILURE       2
#define GEM_MODPROG_COMPLETE      3
#define GEM_MODPROG_NOT_ALIGNED   4
#define GEM_MODPROG_BAD_COMPARE   5
#define GEM_MODPROG_TIMEOUT       6
#define GEM_MODPROG_CALCULATING   7
#define GEM_MODPROG_ABORTED       8
#define GEM_MODPROG_LOW_ADDRESS   9
#define GEM_MODPROG_FAILED        10
#define GEM_MODPROG_CALCFAIL      11
#define GEM_MODPROG_HIGH          GEM_MODPROG_CALCFAIL

#define GEM_FPGAPROG_START        20
#define GEM_FPGAPROG_LOW          GEM_FPGAPROG_START
#define GEM_FPGAPROG_SUCCESS      21
#define GEM_FPGAPROG_FAILURE      22
#define GEM_FPGAPROG_COMPLETE     23
#define GEM_FPGAPROG_NOT_ALIGNED  24
#define GEM_FPGAPROG_BAD_COMPARE  25
#define GEM_FPGAPROG_TIMEOUT      26
#define GEM_FPGAPROG_ABORTED      27
#define GEM_FPGAPROG_LOW_ADDRESS  28
#define GEM_FPGAPROG_NO_FILE      29
#define GEM_FPGAPROG_NO_ADDR      30
#define GEM_FPGAPROG_FAILED       31
#define GEM_FPGAPROG_FILE_SIZE    32
#define GEM_FPGAPROG_HIGH         GEM_FPGAPROG_FILE_SIZE

#define GEM_FILE_FAILED           0
#define GEM_FILE_SUCCESS          1
#define GEM_FILE_OPEN_FAILED      2
#define GEM_FILE_WRITE_FAILED     3
#define GEM_FILE_READ_FAILED      4
#define GEM_CALC_FAILED           5

class CGemFlashResult: public CGemNetMsg
{
public:	
	unsigned int   m_address;
	unsigned short m_result;
	unsigned short m_spare;

	CGemFlashResult()
	{
		m_head.m_type = 0x44;
		m_address = 0;
    m_result  = 0;
    m_spare   = 0;
	}
};

class CGemFlashReadback: public CGemNetMsg
{
public:	
	unsigned int   m_address;
	unsigned int   m_spare;
	unsigned char  m_data[1024];

	CGemFlashReadback()
	{
		m_head.m_type = 0x45;
	}
};

class CGemVelocimeterData: public CGemNetMsg
{
public:	
	unsigned short m_blockCount;
	unsigned short m_spare;
	unsigned char  m_data[1024];

	CGemVelocimeterData()
	{
		m_head.m_type = 0x47;
	}
};

class CGemRS232RXPacket : public CGemNetMsg
{
public:
  unsigned char m_data1;

	CGemRS232RXPacket()
	{
		m_head.m_type = 0x48;
		m_data1       = 0;
	}
};

#pragma pack (push,2)

class CGemTestResult: public CGemNetMsg
{
public:
  unsigned short      m_packing1;	
  unsigned short      m_packing2;	
  unsigned short      m_packing3;	
  unsigned short      m_packing4;	
	unsigned int        m_SRAM0Result;
	unsigned int        m_SRAM1Result;
  unsigned short      m_flashPostResult;
  unsigned int        m_flashProdResultL;
  unsigned int        m_flashProdResultH;
  unsigned int        m_firstErrorAddress;
  unsigned long long  m_seedValueWritten;
  unsigned long long  m_seedValueUsed;
    
	CGemTestResult()
	{
		m_head.m_type = 0x4A;
	}
};

#pragma pack (pop)

class CGemMDIOAck: public CGemNetMsg
{
public:
  unsigned short m_data1;	
    
	CGemMDIOAck()
	{
		m_head.m_type = 0x4B;
	}
};

class CGemGPIData: public CGemNetMsg
{
public:
  unsigned short m_GPIData;	
    
	CGemGPIData()
	{
		m_head.m_type = 0x4C;
	}
};

class CGemDiagData: public CGemNetMsg
{
public:
  unsigned short m_RX1BoardIdentData;	
  unsigned short m_RX2BoardIdentData;	
  unsigned short m_FPGABoardIdentData;	
  unsigned short m_PSUBoardIdentData;	
  unsigned short m_TXBoardIdentData;
    
	CGemDiagData()
	{
		m_head.m_type = 0x4D;
    m_RX1BoardIdentData  = 0;	
    m_RX2BoardIdentData  = 0;	
    m_FPGABoardIdentData = 0;	
    m_PSUBoardIdentData  = 0;	
    m_TXBoardIdentData   = 0;
	}
};
#endif // __GEMINISTRUCTURES_H__

