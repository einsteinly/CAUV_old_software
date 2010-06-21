#include <stdio.h>
#include <iostream>

#include "sonar_seanet_packet.h"
#include "sonar.h"

/* Fills in the header section of the data field for the packet */
void SonarSeanetPacket::createHeader()
{
	/* init byte */
	m_data[0] = 0x40;
	/* hex length */
	sprintf((char*)(m_data+1), "%04X", m_length);
	/* bin length */
	*((unsigned short*)(m_data+5)) = m_length;
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

unsigned char *SonarSeanetPacket::getData() const
{
	return m_data;
}

unsigned short SonarSeanetPacket::getLength() const
{
	return m_length;
}

SonarHeadParamsPacket::SonarHeadParamsPacket(SonarHeadParams &params)
{
	m_data = new unsigned char[66];

	m_type = SEANET_HEADCOMMAND;
	m_sid = 0xFF;
	m_did = 0x02;
	m_count = 55;
	m_length = 60;

	createHeader();

	cout << "Head params are " << sizeof(SonarHeadParams) << " bytes" << endl;

	memcpy(&m_data[13], &params, sizeof(SonarHeadParams));
}

SonarRebootPacket::SonarRebootPacket()
{
	m_data = new unsigned char[14];

	m_type = SEANET_REBOOT;
	m_sid = 0xFF;
	m_did = 0x02;
	m_count = 0x03;
	m_length = 0x08;

	createHeader();
}

SonarSendbbuserPacket::SonarSendbbuserPacket()
{
	m_data = new unsigned char[14];

	m_type = SEANET_SENDBBUSER;
	m_sid = 0xFF;
	m_did = 0x02;
	m_length = 0x08;
	m_count = 0x03;

	createHeader();
}

SonarSenddataPacket::SonarSenddataPacket()
{	
	m_data = new unsigned char[18];

	m_type = SEANET_SENDDATA;
	m_sid = 0xFF;
	m_did = 0x02;
	m_length = 12;
	m_count = 7;

	createHeader();

	*((uint32_t*)&m_data[13]) = 0;  // Current time
}

SonarSendversionPacket::SonarSendversionPacket()
{
	m_data = new unsigned char[14];

	m_type = SEANET_SENDVERSION;
	m_sid = 0xFF;
	m_did = 0x02;
	m_count = 0x03;
	m_length = 0x08;

	createHeader();
}

SonarSeanetPacket::~SonarSeanetPacket()
{
	delete[] m_data;
}
