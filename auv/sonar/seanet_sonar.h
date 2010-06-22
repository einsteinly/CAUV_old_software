#ifndef __SEANET_SONAR_H__
#define __SEANET_SONAR_H__

#include <stdint.h>
#include <pthread.h>
#include <list>

#include <common/cauv_node.h>
#include <common/messages.h>
#include <common/blocking_queue.h>

#include "sonar_observer.h"
#include "seanet_packet.h"
#include "seanet_serial_port.h"

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
        bool operator!=(MotorState& rhs) const;
};


class SeanetSonar
{
	public:
	
		SeanetSonar(std::string file);
		~SeanetSonar();
	
		void addObserver(boost::shared_ptr<SonarObserver> o);
		void removeObserver(boost::shared_ptr<SonarObserver> o);
        void clearObservers();
	
		/* Initialize and start scanning */
		void init();
	
		/* Send some default paramaters to the sonar */
		void sendDefaultParams();
		
		void set_scan_settings(unsigned char def, uint16_t direction, uint16_t width,
                               unsigned char gain, uint32_t range, uint32_t radial_res,
                               unsigned char angular_res);


		/* units of mm */
		static void set_res_range(SeanetHeadParams &params, unsigned int res, unsigned int range);

		/* sets the operating mode - either sector or continuous */
		void set_mode_continuous();
		void set_mode_sector(uint16_t leftLim, uint16_t rightLim);
	
		/* sets the step size (angular resolution) */
		void set_step_size(unsigned char size);

        friend void sonarReadThread(boost::shared_ptr<SeanetSonar> sonar);
        friend void sonarProcessThread(boost::shared_ptr<SeanetSonar> sonar);
	
    
    private:
		boost::shared_ptr<SeanetSerialPort> m_serial_port;
		
        boost::thread m_read_thread;
        boost::thread m_process_thread;

		MotorState m_motor_state;

		// The params the sonar is going to change to
		SeanetHeadParams m_next_params;

		/* The last set of params that were sent to the sonar */
		SeanetHeadParams m_params;

		/* The scan mode the sonar defaults to when not being polled */
		SeanetHeadParams m_default_params;
	
		//  Data queue
		BlockingQueue< boost::shared_ptr<SeanetPacket> > m_queue;

		// Set this to 1 when the sonar needs to complete a polling exercise...
		enum PollState {
			POLL_NOTSTARTED, POLL_GETBEARING, POLL_GOTBEARING,
		} m_poll_state;

		uint16_t m_poll_start_bearing;

		/* 
		 * How many data requests have been issued which haven't
		 * been replied to, multiplied by 2.  Used to make sure
		 * there is always one outstanding data request.
		 */
		int m_cur_data_reqs;
		
		enum SeanetSonarState {
			SENDREBOOT,
			WAITFORREADY, WAITFORVERSION,
			WAITFORPARAMS, SCANNING,
		} m_state;
	
        std::list< boost::shared_ptr<SonarObserver> > m_obs;
	
		/* Called when a line of data is received */
		void process_data(boost::shared_ptr<SeanetPacket> pkt);
	
		/* Returns true if parameters have been received and validated */
		int hasParams() const;
		int isReady() const;
		void wait_for_noparams();
		void wait_for_hasparams();
		void wait_for_packet(unsigned char type);
		void request_version();
	
		/* Use this to update the sonar's parameters */
		void upload_params(SeanetHeadParams& params);
};

#endif

