/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include <iostream>
#include <iomanip>
#include <string>
#include <stdio.h>
#include <string.h>

#include <debug/cauv_debug.h>
#include <utility/foreach.h>

#include "seanet_packet.h"

using namespace cauv;

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
SeanetPacket::SeanetPacket(unsigned char type, unsigned short length)
	: m_data(length+6, '\0')
{
	/* init byte */
	m_data[0] = 0x40;
	/* hex length */
	sprintf(reinterpret_cast<char*>(&m_data[1]), "%04X", length);
	/* bin length */
	*reinterpret_cast<unsigned short*>(&m_data[5]) = length;
	/* tx node */
	m_data[7] = 0xff; // Sender (computer)
	/* rx node */
	m_data[8] = 0x02; // Destination (0x02 is an imaging sonar)
	/* num bytes */
	m_data[9] = length - 5;
	/* msg type */
	m_data[10] = type;
	/* seq = end */
	m_data[11] = 0x80; // Always 0x80 for devices without multi-packet mode
	/* Nde */
	m_data[12] = 0x02; // Copy of bit 8, for some reason

	/* LF */
	m_data[length + 5] = 0x0A;
}
SeanetPacket::SeanetPacket(const std::string& data)
    : m_data(data)
{
}

const std::string& SeanetPacket::data() const { return m_data; }
std::string& SeanetPacket::data() { return m_data; }
const char* SeanetPacket::payload() const { return &m_data[13]; }
char* SeanetPacket::payload() { return &m_data[13]; }
unsigned char SeanetPacket::type() const { return m_data[10]; }
unsigned short SeanetPacket::length() const
{
	return *reinterpret_cast<const unsigned short*>(&m_data[5]);
}
std::ostream& operator<<(std::ostream& o, const SeanetPacket& p)
{
    o << "Seanet packet (type = " << (int)p.type() << ", length = " << p.length() << ")" << std::endl;
    o << "    data = [ ";
    o << std::hex << std::setw(2) << std::setfill('0');
    foreach (const char& c, p.data())
        o << (int)(unsigned char)c << " ";
    o << "]";
    return o;
}

SeanetHeadParamsPacket::SeanetHeadParamsPacket(SeanetHeadParams &params) : SeanetPacket(SeanetMessageType::HeadCommand, 60)
{
	debug() << "Head params are " << sizeof(SeanetHeadParams) << " bytes";

	memcpy(&payload()[0], &params, sizeof(SeanetHeadParams));
}

SeanetRebootPacket::SeanetRebootPacket() : SeanetPacket(SeanetMessageType::ReBoot, 8)
{
}

SeanetSendBBUserPacket::SeanetSendBBUserPacket() : SeanetPacket(SeanetMessageType::SendBBuser, 8)
{
}

SeanetSendDataPacket::SeanetSendDataPacket() : SeanetPacket(SeanetMessageType::SendData, 12)
{	
	*reinterpret_cast<uint32_t*>(&payload()[0]) = 0;  // Current time (can be 0 for imaging sonar)
}

SeanetSendVersionPacket::SeanetSendVersionPacket() : SeanetPacket(SeanetMessageType::SendVersion, 8)
{
}

