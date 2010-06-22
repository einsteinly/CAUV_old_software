#include "seanet_sonar.h"

#include <iostream>
#include <vector>
#include <cmath>

#include <boost/make_shared.hpp>

#include "seanet_packet.h"

using namespace std;


/*
SeanetSonarControlMessageObserver::SeanetSonarControlMessageObserver(SeanetSonar *sonar)
{
    m_sonar = sonar;
}

void SeanetSonarControlMessageObserver::onSonarControlMessage(const AuvMessage& auvmsg)
{
    switch(auvmsg.getType()) {
        case CMSG_SONAR_CONTROL:
        {
            SeanetSonarControlMessage m = dynamic_cast<const SeanetSonarControlMessage&>(auvmsg);
            m_sonar->set_scan_settings(m.getMode(), m.getDirection(), m.getWidth(), m.getGain(), m.getRange(), m.getRadialRes(), m.getAngularRes(), m.getUpdateRate());
        }
            break;
        default:
            break;
    }
}
*/


MotorState::MotorState() : centered(0), motoron(0), motoring(0), noparams(0),
                            sentcfg(0), time(0), cpuid(0), proglength(0),
                            checksum(0)
{}

bool MotorState::operator==(MotorState& rhs) const
{
    return centered == rhs.centered &&
           motoron == rhs.motoron &&
           motoring == rhs.motoring &&
           noparams == rhs.noparams &&
           sentcfg == rhs.sentcfg &&
           cpuid == rhs.cpuid &&
           proglength == rhs.proglength &&
           checksum == rhs.checksum;
}
bool MotorState::operator!=(MotorState& rhs) const
{
    return !(*this == rhs);
}



SeanetSonar::SeanetSonar(std::string str) : m_poll_state(POLL_NOTSTARTED), m_cur_data_reqs(0)
{
    m_serial_port = boost::make_shared<SeanetSerialPort>(str);
    m_state = SENDREBOOT;
}

void sonarReadThread(SeanetSonar& sonar);
void sonarProcessThread(SeanetSonar& sonar);
void SeanetSonar::init()
{
    debug() << "Initialising sonar";
    m_read_thread = boost::thread(sonarReadThread, boost::ref<SeanetSonar>(*this));
    m_process_thread = boost::thread(sonarProcessThread, boost::ref<SeanetSonar>(*this));
}

SeanetSonar::~SeanetSonar()
{
    debug() << "Attempting to interrupt threads";
    if (m_read_thread.get_id() != boost::thread::id()) {
        m_read_thread.interrupt();
        m_read_thread.join();
    }
    if (m_process_thread.get_id() != boost::thread::id()) {
        m_process_thread.interrupt();
        m_process_thread.join();
    }
}

void SeanetSonar::request_version()
{
    SeanetSendVersionPacket pkt;
    info() << "Requesting version data...";
    m_serial_port->sendPacket(pkt);
};

inline int SeanetSonar::hasParams() const
{
    //FIXME: cheat. Isn't validating parameters properly.
    return !m_motor_state.noparams; // && m_motor_state.sentcfg;
}

inline int SeanetSonar::isReady() const
{
    return (m_motor_state.centered &&
            m_motor_state.motoron &&
            !m_motor_state.motoring);
}

/* Send params to the sonar */
void SeanetSonar::upload_params(SeanetHeadParams &params)
{
    SeanetHeadParamsPacket pkt(params);

    m_next_params = params;
    m_serial_port->sendPacket(pkt);
}

static uint32_t range_from_nbins_adinterval(uint16_t nBins, uint16_t adInterval)
{
    uint32_t half_ping_time = nBins * adInterval * 320; // in nanoseconds
    uint32_t range_mm = (half_ping_time * 15) / 10000; // velocity of sound in mm per second

    return range_mm;
}

/* res has units of cm, range has units of meters */
void SeanetSonar::set_res_range(SeanetHeadParams &params, unsigned int res, unsigned int range)
{
    uint16_t nBins;
    uint16_t adInterval;
    float ping_time;

    nBins = (range) / res;

    if (nBins > 1400) {
        warning() << "Someone tried to set the sonar resolution too high. Reducing it.";
        nBins = 1400;
    }

    ping_time = (2 * range * 1000) / 1500; // units of microseconds. 1500 is VoS
    adInterval = ping_time / (nBins * 0.64); // Sonar has clock time of 640ns

    info() << "Setting nBins to " << nBins << " and adInterval to " << adInterval;
    info() << "Range has been set to " << range_from_nbins_adinterval(nBins, adInterval) << "mm";

    params.adInterval = adInterval;
    params.nBins = nBins;
    params.rangeScale = range;// * 10;
}

void SeanetSonar::set_scan_settings(u_char mode, uint16_t direction, uint16_t width,
                              u_char gain, uint32_t range, uint32_t radial_res,
                              u_char angular_res)
{
    SeanetHeadParams params;

    info() << "Setting scan settings";

    if (width == 6400)
    {
        // Continuous scanning
        params.hdCtrl |= HdCtrl::Continuous;
        params.hdCtrl &= ~HdCtrl::StareLLim;
    }
    else if (width == 0)
    {
        // Pinging
        params.hdCtrl &= ~HdCtrl::Continuous;
        params.hdCtrl |= HdCtrl::StareLLim;
    }
    else
    {
        // Sector
        params.hdCtrl &= ~HdCtrl::Continuous;
        params.hdCtrl &= ~HdCtrl::StareLLim;
    }

    params.stepSize = angular_res;
    params.igainChn1 = gain;
    params.igainChn2 = gain;
    params.rangeScale = range;

    if (direction - width/2 < 0 ) {
        direction += 6400;
    }

    params.leftLim = direction - width/2;
    params.rightLim = params.leftLim + width;

    set_res_range(params, radial_res, range);

    // Updating default parameters?
    /*switch(mode)
    {
        case CAUV_SONAR_MODE_DEFAULT:
            m_default_params = params;
            break;
        case CAUV_SONAR_MODE_POLL_IMG:
            m_params = params;
            upload_params(params);
            m_poll_state = POLL_GETBEARING;
            break;
    }*/
}



void SeanetSonar::addObserver(boost::shared_ptr<SonarObserver> o)
{
    m_obs.push_back(o);
}

void SeanetSonar::removeObserver(boost::shared_ptr<SonarObserver> o)
{
    m_obs.remove(o);
}

void SeanetSonar::clearObservers()
{
    m_obs.clear();
}


bool SeanetHeadData::same_settings(const SeanetHeadData& first, const SeanetHeadData& second)
{
    return first.hdCtrl == second.hdCtrl && first.gain == second.gain &&
           first.adSpan == second.adSpan && first.adLow == second.adLow &&
           first.adInterval == second.adInterval && first.leftLim == second.leftLim &&
           first.rightLim == second.rightLim && first.stepSize == second.stepSize;
}

void SeanetSonar::process_data(boost::shared_ptr<SeanetPacket> pkt)
{
    static SeanetHeadData last_head_data;
    const SeanetHeadData* head_data;

    // Get the received data from the packet
    head_data = reinterpret_cast<const SeanetHeadData*>(&pkt->getData()[11]);

    if (!SeanetHeadData::same_settings(last_head_data, *head_data)) {
        switch (m_poll_state) {
            case POLL_GETBEARING:
                m_poll_start_bearing = head_data->bearing;
                if (m_poll_start_bearing < m_next_params.leftLim || m_poll_start_bearing > m_next_params.rightLim)
                    m_poll_start_bearing = m_next_params.rightLim;

                m_poll_state = POLL_GOTBEARING;
                break;
            case POLL_GOTBEARING:
                if (head_data->bearing == m_poll_start_bearing) {
                    // Scan complete
                    m_poll_state = POLL_NOTSTARTED;
                }
                break;
            default:
                break;
        }
    
        m_params = m_next_params;
    }

    // FIXME:  Assuming 8-bit mode again
   
    boost::shared_ptr<SonarDataLine> dataline = boost::make_shared<SonarDataLine>();
    dataline->data = std::vector< uint8_t >(&head_data->bin1, &head_data->bin1 + head_data->dBytes);
    dataline->range = range_from_nbins_adinterval(head_data->dBytes, head_data->adInterval);
    dataline->bearing = head_data->bearing;
    
    debug() << "Sending data line with range" << dataline->range << "mm and bearing" << dataline->bearing << endl;
    
    foreach (boost::shared_ptr<SonarObserver> o, m_obs)
    {
        o->onReceiveDataLine(*dataline);
    }

    last_head_data = *head_data;
}



void sonarReadThread(SeanetSonar& sonar)
{
    try {
        debug() << "Starting read thread";
        
        boost::shared_ptr<SeanetPacket> pkt;
        MotorState last_state;
        SeanetSendDataPacket sendData;

        debug() << "Waiting for first sonar packet";

        info() << "Rebooting sonar";
        SeanetRebootPacket reboot;
        sonar.m_serial_port->sendPacket(reboot);
    
        while (true)
        {
            retry:
            boost::this_thread::interruption_point();

            try {
                pkt = sonar.m_serial_port->readPacket();
            }
            catch (SonarIsDeadException &e) {
                warning() << "Sonar hasn't sent an alive message in 3 secs.  Missing, presumed dead.";
                sonar.m_state = SeanetSonar::SENDREBOOT;
                sonar.m_cur_data_reqs = 0;
                goto retry;
            }

            /*cout << "Received packet: ";
            for (int j=0; j < pkt->m_length + 6; j++) {
                printf("%02x ", pkt->m_data[j]);
            }
            cout << endl;*/

            /* We want to update our interpretation of the state of the sonar
             * regardless of what state our state machine is currently in */
            
            switch  (pkt->getType()) {
                case SeanetMessageType::Alive:
                    debug() << "Alive message received" << endl;
                    sonar.m_motor_state.centered = pkt->getData()[20] & HdInf::Centred;
                    sonar.m_motor_state.motoron = pkt->getData()[20]  & HdInf::MotorOn;
                    sonar.m_motor_state.motoring = pkt->getData()[20] & HdInf::Motoring;
                    sonar.m_motor_state.noparams = pkt->getData()[20] & HdInf::NoParams;
                    sonar.m_motor_state.sentcfg = pkt->getData()[20]  & HdInf::SentCfg;
                    sonar.m_motor_state.time = *(uint32_t*)&pkt->getData()[14];

                    if (sonar.m_motor_state != last_state) {
                        info() << "Transducer centred: " << (sonar.m_motor_state.centered ? "yes" : "no");
                        info() << "Motor on: "           << (sonar.m_motor_state.motoron ? "yes" : "no");
                        info() << "Motoring: "           << (sonar.m_motor_state.motoring ? "yes" : "no");
                        info() << "No params: "          << (sonar.m_motor_state.noparams ? "yes" : "no");
                        info() << "Sent cfg: "           << (sonar.m_motor_state.sentcfg ? "yes" : "no");
                    }

                    break;
                case SeanetMessageType::VersionData:
                    debug() << "Version data received";
                    sonar.m_motor_state.cpuid = *(uint32_t*)&pkt->getData()[14];
                    sonar.m_motor_state.proglength = *(uint32_t*)&pkt->getData()[17];
                    sonar.m_motor_state.checksum = *(uint16_t*)&pkt->getData()[21];
                    break;
                default:
                    break;
            }

            /* Update the state machine and implement the protocol */
            switch (sonar.m_state) {
                case SeanetSonar::SENDREBOOT:
                    {
                        info() << "Rebooting sonar";
                        SeanetRebootPacket reboot;
                        sonar.m_serial_port->sendPacket(reboot);
                        sonar.m_state = SeanetSonar::WAITFORREADY;
                        sonar.m_motor_state.centered = 0;
                    }
                    break;
                case SeanetSonar::WAITFORREADY:
                    if (sonar.isReady()) {
                        debug() << "Moving to state WAITFORVERSION";
                        sonar.m_state = SeanetSonar::WAITFORVERSION;
                    }
                    break;
                case SeanetSonar::WAITFORVERSION:
                    if (pkt->getType() == SeanetMessageType::VersionData) {
                        debug() << "Moving to state WAITFORPARAMS";
                        sonar.m_state = SeanetSonar::WAITFORPARAMS;
                        sonar.upload_params(sonar.m_default_params);
                    } else {
                        static int i = 0;
                        if (++i % 10 == 0) {
                            sonar.request_version();
                            info() << "Requesting version...";
                        }
                    }
                    break;
                case SeanetSonar::WAITFORPARAMS:
                    if (sonar.hasParams()) {
                        sonar.m_state = SeanetSonar::SCANNING;
                        debug() << "Moving to state SCANNING";
                        //sonar.set_res_range(2, 700);
                    } else {
                        warning() << "Hasn't confirmed parameters...";
                    }

                    break;
                case SeanetSonar::SCANNING:
                    /* FIXME:  Maybe do this asynchronously, to ensure we can
                     * always pull data off the sonar fast enough */
                     if (pkt->getType() == SeanetMessageType::HeadData) {
                        sonar.m_cur_data_reqs--;
                        if (sonar.m_queue.size() > 200) {
                            error() << "Sonar buffer size exceeded";
                        } else {
                            sonar.m_queue.push(pkt);
                        }
                    }

                    //info() << "Requesting data..." << endl;
                    while (sonar.m_cur_data_reqs <= 2) {
                        sonar.m_serial_port->sendPacket(sendData);
                        sonar.m_cur_data_reqs++; //TODO: Make sure this is correct
                    }
                    break;
                default:
                    break;
            }

            last_state = sonar.m_motor_state;
        }
    } catch (boost::thread_interrupted& e) {
        debug() << "Read thread interrupted";
    }
    debug() << "Ending read thread";
}


void sonarProcessThread(SeanetSonar& sonar)
{
    try {
        debug() << "Starting processing thread";
        while (true)
        {
            boost::this_thread::interruption_point();

            // Process!
            boost::shared_ptr<SeanetPacket> pkt = sonar.m_queue.popWait();
            sonar.process_data(pkt);
        }
    } catch (boost::thread_interrupted& e) {
        debug() << "Processing thread interrupted";
    }
    debug() << "Ending processing thread";
}
