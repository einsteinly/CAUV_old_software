#ifndef __CAUV_SEANET_SONAR_H__
#define __CAUV_SEANET_SONAR_H__

#include <stdint.h>
#include <pthread.h>
#include <list>

#include <common/cauv_node.h>
#include <common/blocking_queue.h>
#include <generated/message_observers.h>

#include "sonar_observer.h"
#include "seanet_packet.h"
#include "seanet_serial_port.h"

namespace cauv{

class SeanetSonar;

class SonarControlMessageObserver : public MessageObserver
{
    public:
        SonarControlMessageObserver(boost::shared_ptr<SeanetSonar> sonar);
        virtual void onSonarControlMessage(SonarControlMessage_ptr m);

    protected:
        boost::shared_ptr<SeanetSonar> m_sonar;
};
    
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


void sonarReadThread(SeanetSonar& sonar);
void sonarProcessThread(SeanetSonar& sonar);

class SeanetSonar : public Observable<SonarObserver>
{
	public:
	
		SeanetSonar(std::string file);
		~SeanetSonar();
	   
        bool ok() const;

		/* Initialize and start scanning */
		void init();
	
        void set_direction(uint16_t direction);
        void set_width(uint16_t width);
        void set_gain (unsigned char gain);
        void set_range (uint32_t range);
        void set_range_res (uint32_t range_res);
        void set_angular_res (unsigned char angular_res);

        // Direction : 1/6400 of a circle
        // Width : 1/6400 of a circle
        // Gain : [0,255]
        // Range : mm
        // Range res : mm
        // Angular res : 1/6400 of a circle
        void set_params (uint16_t direction,
                         uint16_t width,
                         unsigned char gain,
                         uint32_t range,
                         uint32_t range_res,
                         unsigned char angular_res);

        
        friend void cauv::sonarReadThread(SeanetSonar& sonar);
        friend void cauv::sonarProcessThread(SeanetSonar& sonar);
	
    
    private:
		boost::shared_ptr<SeanetSerialPort> m_serial_port;
		
        boost::thread m_read_thread;
        boost::thread m_process_thread;

		MotorState m_motor_state;

		//  Data queue
		BlockingQueue< boost::shared_ptr<SeanetPacket> > m_queue;

		/* 
		 * How many data requests have been issued which haven't
		 * been replied to, multiplied by 2.  Used to make sure
		 * there is always one outstanding data request.
		 */
		int m_cur_data_reqs;

        struct SonarParams {
            uint16_t direction;
            uint16_t width;
            unsigned char gain;
            uint32_t range;
            uint32_t range_res;
            unsigned char angular_res;

            SonarParams() : direction(0), width(6400), gain(50), range(5000), range_res(10), angular_res(16) {
            }
        } m_current_params;
		
		// Use this if you change m_current_params
        void upload_current_params();
		void upload_head_params(SeanetHeadParams& params);



		enum SeanetSonarState {
			SENDREBOOT,
			WAITFORREADY, WAITFORVERSION,
			WAITFORPARAMS, SCANNING,
		} m_state;
			
		// Called when a line of data is received
		void process_data(boost::shared_ptr<SeanetPacket> pkt);
	
		// Returns true if parameters have been received and validated
		int hasParams() const;
		int isReady() const;
		void wait_for_noparams();
		void wait_for_hasparams();
		void wait_for_packet(unsigned char type);
		void request_version();
};

} // namespace cauv

#endif // ndef __CAUV_SEANET_SONAR_H__

