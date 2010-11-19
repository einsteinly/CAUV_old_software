#include <iostream>
#include <string>
#include <stdio.h>
#include <string.h>

#include <debug/cauv_debug.h>

#include "seanet_packet.h"


/* Fill with some sensible defaults */
SeanetHeadParams::SeanetHeadParams() :
        v3bParams(0x01),
        hdCtrl(HdCtrl::Adc8on | HdCtrl::ReplyASL | HdCtrl::HasMot | HdCtrl::Raw | HdCtrl::ScanRight | HdCtrl::Continuous),
        hdType(11),
        txnChn1(0),
        txnChn2(0),
        rxnChn1(0),
        rxnChn2(0),
        txPulseLen(0),
        rangeScale(60), // 6 metres
        leftLim(2400),  // scan a 90 degree sector...
        rightLim(4000), // ...straight ahead
        adSpan(38),
        adLow(40),
        igainChn1(84),
        igainChn2(84),
        slopeChn1(0),
        slopeChn2(0),
        moTime(25),
        stepSize(16),
        adInterval(20),
        nBins(500),  // Range of about 2.7m
        maxAdBuf(500),
        lockout(100),
        minAxisDir(1600), // 1/16 Grad
        majAxisPan(1),
        ctl2(0),
        scanZ(0)
        //.v3bData({0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}),
        /* These next values I stole by sniffing the windows example program
        v3b_adSpnChn1(0x45),
        v3b_adSpnChn2(0x32),
        v3b_adLowChn1(0x44),
        v3b_adLowChn2(0),
        v3b_igainChn1(0x5A),
        v3b_igainChn2(0x64),
        v3b_adcSetPChn1(0),
        v3b_adcSetPChn2(0),
        v3b_slopeChn1(0x6400),
        v3b_slopeChn2(0),
        v3b_slopeDelayChn1(0),
        v3b_slopeDelayChn2(0)*/
{
}

/* Fills in the header section of the data field for the packet */
void SeanetPacket::fillHeader()
{
	/* init byte */
	m_data[0] = 0x40;
	/* hex length */
	sprintf(reinterpret_cast<char*>(&m_data[1]), "%04X", m_length);
	/* bin length */
	*reinterpret_cast<unsigned short*>(&m_data[5]) = m_length;
	/* tx node */
	m_data[7] = m_sid;
	/* rx node */
	m_data[8] = m_did;
	/* num bytes */
	m_data[9] = m_count;
	/* msg type */
	m_data[10] = m_type;
	/* seq = end */
	m_data[11] = 0x80;
	/* Nde */
	m_data[12] = 0x02;

	/* LF */
	m_data[m_length + 5] = 0x0A;
}

const std::string& SeanetPacket::getData() const
{
	return m_data;
}

unsigned char SeanetPacket::getType() const
{
	return m_type;
}

unsigned short SeanetPacket::getLength() const
{
	return m_length;
}

SeanetHeadParamsPacket::SeanetHeadParamsPacket(SeanetHeadParams &params)
{
	m_type = SeanetMessageType::HeadCommand;
	m_sid = 0xFF;
	m_did = 0x02;
	m_length = 60;
    m_count = m_length - 5;

	m_data = std::string(m_length+6, '\0');
	fillHeader();

	debug() << "Head params are " << sizeof(SeanetHeadParams) << " bytes";

	memcpy(&m_data[13], &params, sizeof(SeanetHeadParams));
}

SeanetRebootPacket::SeanetRebootPacket()
{
	m_type = SeanetMessageType::ReBoot;
	m_sid = 0xFF;
	m_did = 0x02;
	m_length = 8;
    m_count = m_length - 5;

	m_data = std::string(m_length+6, '\0');
	fillHeader();
}

SeanetSendBBUserPacket::SeanetSendBBUserPacket()
{
	m_type =SeanetMessageType::SendBBuser;
	m_sid = 0xFF;
	m_did = 0x02;
	m_length = 8;
    m_count = m_length - 5;

	m_data = std::string(m_length+6, '\0');
	fillHeader();
}

SeanetSendDataPacket::SeanetSendDataPacket()
{	
	m_type = SeanetMessageType::SendData;
	m_sid = 0xFF;
	m_did = 0x02;
	m_length = 12;
    m_count = m_length - 5;

	m_data = std::string(m_length+6, '\0');
	fillHeader();

	*reinterpret_cast<uint32_t*>(&m_data[13]) = 0;  // Current time
}

SeanetSendVersionPacket::SeanetSendVersionPacket()
{
	m_type = SeanetMessageType::SendVersion;
	m_sid = 0xFF;
	m_did = 0x02;
	m_length = 8;
    m_count = m_length - 5;

	m_data = std::string(m_length+6, '\0');
	fillHeader();
}

