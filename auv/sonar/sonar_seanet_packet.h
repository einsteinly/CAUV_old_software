#ifndef __SONAR_SEANET_PACKET_H__
#define __SONAR_SEANET_PACKET_H__

#include <stdint.h>
class SonarSeanetPacket {
protected:
	/* Length of payload not inc LF */
	unsigned short m_length;
	/* Source identifier */
	unsigned char m_sid;
	/* Desination identifier */
	unsigned char m_did;

	unsigned char m_count;

	unsigned char m_type;

	/* The full data of the packet */
	unsigned char *m_data;

	void createHeader();
public:
	SonarSeanetPacket() {};
	~SonarSeanetPacket();

	unsigned char *getData() const;
	unsigned short getLength() const;

	friend class SerialPort;
	friend void *readThread(void *data);
};

class SonarHeadParams;
class SonarHeadParamsPacket : public SonarSeanetPacket {
public:
	SonarHeadParamsPacket(SonarHeadParams &params);
};

class SonarRebootPacket : public SonarSeanetPacket {
public:
	SonarRebootPacket();
};

class SonarSendbbuserPacket : public SonarSeanetPacket {
public:
	SonarSendbbuserPacket();
};

class SonarSenddataPacket : public SonarSeanetPacket {
public:
	SonarSenddataPacket();
};

class SonarSendversionPacket : public SonarSeanetPacket {
public:
	SonarSendversionPacket();
};

#endif

