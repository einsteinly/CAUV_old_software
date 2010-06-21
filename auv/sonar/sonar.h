#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdint.h>
#include <pthread.h>
#include <vector>

#include <common/cauv_node.h>
#include <common/messages.h>
#include <common/blocking_queue.h>

#include "sonar_observer.h"
#include "sonar_seanet_packet.h"
#include "sonar_serial.h"

const unsigned char SEANET_NULL				= 0;
const unsigned char SEANET_VERSIONDATA		= 1;
const unsigned char SEANET_HEADDATA			= 2;
const unsigned char SEANET_SPECTDATA		= 3;
const unsigned char SEANET_ALIVE			= 4;
const unsigned char SEANET_PRGACK			= 5;
const unsigned char SEANET_BBUSERDATA		= 6;
const unsigned char SEANET_TESTDATA			= 7;
const unsigned char SEANET_AUXDATA			= 8;
const unsigned char SEANET_ADCDATA			= 9;
const unsigned char SEANET_ADCREQ			= 10;
const unsigned char SEANET_LANSTATUS		= 13;
const unsigned char SEANET_SETTIME			= 14;
const unsigned char SEANET_TIMEOUT			= 15;
const unsigned char SEANET_REBOOT			= 16;
const unsigned char SEANET_PERFORMANCEDATA	= 17;
const unsigned char SEANET_HEADCOMMAND		= 19;
const unsigned char SEANET_ERASESECTOR		= 20;
const unsigned char SEANET_PROGBLOCK		= 21;
const unsigned char SEANET_COPYBOOTBLK		= 22;
const unsigned char SEANET_SENDVERSION		= 23;
const unsigned char SEANET_SENDBBUSER		= 24;
const unsigned char SEANET_SENDDATA			= 25;
const unsigned char SEANET_SENDPERFNCEDATA	= 26;
const unsigned char SEANET_FPGAVERSDATA		= 57;
const unsigned char SEANET_FPGACALIBDATA	= 63;

const uint16_t HDCTRL_ADC8ON		= 1; // 4/8 bit res?
const uint16_t HDCTRL_CONT			= 2; // Continuous scanning?
const uint16_t HDCTRL_SCANRIGHT		= 4; // Scan direction
const uint16_t HDCTRL_INVERT		= 8; // Invert scan direction
const uint16_t HDCTRL_MOTOFF		= 16; // Disable motor
const uint16_t HDCTRL_TXOFF			= 32; // Disable sonar transmitter
const uint16_t HDCTRL_SPARE			= 64; // 0 always
const uint16_t HDCTRL_CHAN2			= 128; // Use channel 2?
const uint16_t HDCTRL_RAW			= 256; // 1 always
const uint16_t HDCTRL_HASMOT		= 512; // 1 always (we've got a motor)
const uint16_t HDCTRL_APPLYOFFSET	= 1024; // Apply heading offsets?
const uint16_t HDCTRL_PINGPONG		= 2048; // 0 always (used for sidescan)
const uint16_t HDCTRL_STARELLIM		= 4096; // 0 always (seasprite can't)
const uint16_t HDCTRL_REPLYASL		= 8192; // 1 always
const uint16_t HDCTRL_REPLYTHR		= 16384; // 0 always
const uint16_t HDCTRL_IGNSENSOR		= 32768; // 0 always

const unsigned char HDINF_CENTRED		= (1<<1);
const unsigned char HDINF_MOTORING		= (1<<2);
const unsigned char HDINF_MOTORON		= (1<<3);
const unsigned char HDINF_NOPARAMS		= (1<<6);
const unsigned char HDINF_SENTCFG		= (1<<7);

#define B2STR(a) ((a) ? "yes" : "no")

/*
 * A mtHeadData message has this as it's body, followed by 'dBytes' worth
 * of data bins.
 */
class SonarHeadData {
public:
	unsigned char messageSequence;
	unsigned char txNode;
	uint16_t byteCount;
	unsigned char dev_type;
	unsigned char hdStatus;
	unsigned char sweep;
	uint16_t hdCtrl;
	uint16_t rangeScale;
	uint32_t txNumber;
	unsigned char gain;
	uint16_t slope;
	unsigned char adSpan;
	unsigned char adLow;
	uint16_t headingOffset;
	uint16_t adInterval;
	uint16_t leftLim;
	uint16_t rightLim;
	unsigned char stepSize;
	uint16_t bearing;
	uint16_t dBytes;

	static bool same_settings(SonarHeadData &first, SonarHeadData &second);
} __attribute__((__packed__));

/*
 * WARNING
 * Don't add virtual functions to this class. I've no idea if the pointer
 * to the VFT will end up messing up the aligment of the members.
 */
class SonarHeadParams {
public:
	unsigned char v3bParams;
	uint16_t hdCtrl;
	unsigned char hdType;
	uint32_t txnChn1;
	uint32_t txnChn2;
	uint32_t rxnChn1;
	uint32_t rxnChn2;
	uint16_t txPulseLen;
	uint16_t rangeScale;
	uint16_t leftLim;
	uint16_t rightLim;
	unsigned char adSpan;
	unsigned char adLow;
	unsigned char igainChn1;
	unsigned char igainChn2;
	uint16_t slopeChn1;
	uint16_t slopeChn2;
	unsigned char moTime;
	unsigned char stepSize;
	uint16_t adInterval;
	uint16_t nBins;
	uint16_t maxAdBuf;
	uint16_t lockout;
	uint16_t minAxisDir;
	unsigned char majAxisPan;
	unsigned char ctl2;
	uint16_t scanZ;
	/* v3b params 
	unsigned char v3b_adSpnChn1;
	unsigned char v3b_adSpnChn2;
	unsigned char v3b_adLowChn1;
	unsigned char v3b_adLowChn2;
	unsigned char v3b_igainChn1;
	unsigned char v3b_igainChn2;
	unsigned char v3b_adcSetPChn1;
	unsigned char v3b_adcSetPChn2;
	uint16_t v3b_slopeChn1;
	uint16_t v3b_slopeChn2;
	uint16_t v3b_slopeDelayChn1;
	uint16_t v3b_slopeDelayChn2;
*/
public:
	SonarHeadParams();
} __attribute__((__packed__));

class MotorState {
public:
	/* From mtAlive messages */
	int centered;
	int motoron;
	int motoring;
	int noparams;
	int sentcfg;
	uint32_t time;

	/* From mtVersionData messages */
	uint32_t cpuid;
	uint32_t proglength;
	uint16_t checksum;

	MotorState();

	bool operator==(MotorState& rhs) const;
};

/*
 * This class holds one full 'sweep' of a sonar scan.
 * We know how much data to hold by inspecting the current sonar parameters.
 */
class SonarData
{
	public:
		unsigned char **m_data;
		// Number of different angles the sonar could be staring at
		unsigned int m_num_angles;
		unsigned int m_num_bins;

		SonarData(SonarHeadParams &params);
		~SonarData();
};

class Sonar
{
	private:
		SerialPort *m_serial_port;
		
		pthread_t m_read_thread;
		pthread_t m_process_thread;

		MotorState m_motor_state;

		// The params the sonar is going to change to
		SonarHeadParams m_next_params;

		/* The last set of params that were sent to the sonar */
		SonarHeadParams m_params;

		/* The scan mode the sonar defaults to when not being polled */
		SonarHeadParams m_default_params;
	
		/* The last YPR values we've seen */
		CauvImage *m_img;
		SonarData *m_head_data;

		//  Data queue
		BlockingQueue<pair<SonarSeanetPacket*, floatYPR>* > m_queue;

		// Set this to 1 when the sonar needs to complete a polling exercise...
		enum PollState {
			POLL_NOTSTARTED, POLL_GETBEARING, POLL_GOTBEARING,
		} m_poll_state;

		// FIXME:  Move this into the params class, and remove the attribute_packed
		// stuff and the dependecies on that.
		uint16_t m_img_update_rate;
		uint16_t m_poll_start_bearing;

		/* 
		 * How many data requests have been issued which haven't
		 * been replied to, multiplied by 2.  Used to make sure
		 * there is always one outstanding data request.
		 */
		int m_cur_data_reqs;
		
		enum SonarState {
			SENDREBOOT,
			WAITFORREADY, WAITFORVERSION,
			WAITFORPARAMS, SCANNING,
		} m_state;
	
		floatYPR m_last_attitude;

		vector<SonarObserver*> m_obs;
	
		/* Called when a line of data is received */
		void process_data(SonarSeanetPacket *pkt, floatYPR current_attitude);
	
		/* Returns true if parameters have been received and validated */
		int hasParams() const;
		int isReady() const;
		void wait_for_noparams();
		void wait_for_hasparams();
		void wait_for_packet(unsigned char type);
		void request_version();
	
		/* Use this to update the sonar's parameters */
		void upload_params(SonarHeadParams &params);
	
	public:
		/* A 256 x 1 image of colours to use for the sonar output */
		static u_char *m_heat_map;
	
		Sonar(const char *file);
		~Sonar();
	
		void addObserver(SonarObserver* o);
		void removeObserver(SonarObserver* o);
	
		/* Initialize and start scanning */
		void init();
	
		/* Send some default paramaters to the sonar */
		void sendDefaultParams();
		
		void set_scan_settings(u_char def, uint16_t direction, uint16_t width, u_char gain,
							   uint32_t range, uint32_t radial_res,
							   u_char angular_res, uint16_t update_rate);


		void update_image(SonarHeadData &head_data, floatYPR attitude);
		/* units of mm */
		static void set_res_range(SonarHeadParams &params, unsigned int res, unsigned int range);

		/* sets the operating mode - either sector or continuous */
		void set_mode_continuous();
		void set_mode_sector(uint16_t leftLim, uint16_t rightLim);
	
		/* sets the step size (angular resolution) */
		void set_step_size(unsigned char size);

		friend void *processThread(void *data);
		friend void *readThread(void *data);
		friend class SonarTelemetryMessageObserver;
};

class sonar_node : public cauv_node
{
	protected:
		Sonar *m_sonar;
		TCPSonarObserver *m_tcp_obs;
		TCPSonarObserver *m_tcp_processed_obs;

		DisplaySonarObserver *m_display_obs;

		virtual void on_connect();
		virtual void on_disconnect();
		virtual void on_run();
		pthread_t m_sonar_thread;

	public:
		sonar_node(const string& device);
		virtual ~sonar_node();
};

class SonarControlMessageObserver : public MessageObserver
{
	private:
		Sonar *m_sonar;
	public:
		SonarControlMessageObserver(Sonar *sonar);
		virtual void onMessage(const AuvMessage& auvmsg);
};

class SonarTelemetryMessageObserver : public MessageObserver
{
	private:
		Sonar *m_sonar;
	public:
		SonarTelemetryMessageObserver(Sonar *sonar);
		virtual void onMessage(const AuvMessage& auvmsg);
};
#endif

