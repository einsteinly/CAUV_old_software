#include "seanet_sonar.h"

#include <iostream>
#include <vector>
#include <cmath>

#include <boost/make_shared.hpp>

#include "seanet_packet.h"

using namespace std;


SonarControlMessageObserver::SonarControlMessageObserver(boost::shared_ptr<SeanetSonar> sonar)
{
    m_sonar = sonar;
}

void SonarControlMessageObserver::onSonarControlMessage(boost::shared_ptr<SonarControlMessage> m)
{
    m_sonar->set_scan_settings(m->direction(), m->width(), m->gain(), m->range(), m->radialRes(), m->angularRes());
}


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



SeanetSonar::SeanetSonar(std::string str) : m_cur_data_reqs(0)
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
    m_serial_port->sendPacket(pkt);
}

static uint32_t range_from_nbins_adinterval(uint16_t nBins, uint16_t adInterval)
{
    uint32_t half_ping_time = nBins * adInterval * 320; // in nanoseconds
    uint32_t range_mm = (half_ping_time * 15) / 10000; // speed of sound in mm per second

    return range_mm;
}

/* res has units of cm, range has units of meters */
void SeanetSonar::set_res_range(SeanetHeadParams& params, unsigned int res, unsigned int range)
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

void SeanetSonar::set_scan_settings(uint16_t direction, uint16_t width,
                              unsigned char gain, uint32_t range, uint32_t radial_res,
                              unsigned char angular_res)
{
    info() << "Setting scan settings";

    if (width == 6400)
    {
        // Continuous scanning
        m_params.hdCtrl |= HdCtrl::Continuous;
        m_params.hdCtrl &= ~HdCtrl::StareLLim;
    }
    else if (width == 0)
    {
        // Pinging
        m_params.hdCtrl &= ~HdCtrl::Continuous;
        m_params.hdCtrl |= HdCtrl::StareLLim;
    }
    else
    {
        // Sector
        m_params.hdCtrl &= ~HdCtrl::Continuous;
        m_params.hdCtrl &= ~HdCtrl::StareLLim;
    }

    m_params.stepSize = angular_res;
    m_params.igainChn1 = gain;
    m_params.igainChn2 = gain;
    m_params.rangeScale = range;

    if (direction - width/2 < 0 ) {
        direction += 6400;
    }

    m_params.leftLim = direction - width/2;
    m_params.rightLim = m_params.leftLim + width;

    set_res_range(m_params, radial_res, range);

    upload_params(m_params);
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
    const SeanetHeadData* head_data;

    // Get the received data from the packet
    head_data = reinterpret_cast<const SeanetHeadData*>(&pkt->getData()[11]);

    // FIXME:  Assuming 8-bit mode again
   
    boost::shared_ptr<SonarDataLine> dataline = boost::make_shared<SonarDataLine>();
    dataline->data = std::vector< uint8_t >(&head_data->bin1, &head_data->bin1 + head_data->dBytes);
    dataline->range = range_from_nbins_adinterval(head_data->dBytes, head_data->adInterval);
    dataline->bearing = head_data->bearing;
    dataline->bearingRange = head_data->stepSize;
    
    //debug() << "Sending data line with range" << dataline->range << "mm, bearing" << dataline->bearing << ", and bearing range " << dataline->bearingRange;
    //{
    //    std::stringstream data;
    //    foreach(uint8_t c, dataline->data) {
    //        data << hex << setw(2) << setfill('0') << (int)c << " ";
    //    }
    //    debug() << data.str();
    //}

    foreach (observer_ptr_t o, m_observers)
    {
        o->onReceiveDataLine(*dataline);
    }
}



void sonarReadThread(SeanetSonar& sonar)
{
    try {
        debug() << "Starting read thread";
        
        boost::shared_ptr<SeanetPacket> pkt;
        MotorState last_state;
        SeanetSendDataPacket sendData;

        bool isalive = false;

        info() << "Rebooting sonar";
        SeanetRebootPacket reboot;
        sonar.m_serial_port->sendPacket(reboot);
        debug() << "Moving to state WAITFORREADY";
        sonar.m_state = SeanetSonar::WAITFORREADY;
        sonar.m_motor_state.centered = 0;

        while (true)
        {
            retry:
            boost::this_thread::interruption_point();

            try {
                pkt = sonar.m_serial_port->readPacket();
            }
            catch (SonarIsDeadException &e) {
                if (isalive) {
                    warning() << "Sonar hasn't sent an alive message in 3 secs.  Missing, presumed dead.";
                    sonar.m_state = SeanetSonar::SENDREBOOT;
                    sonar.m_cur_data_reqs = 0;
                    isalive = false;
                }
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
                    debug() << "Alive message received";
                    isalive = true;
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
                        debug() << "Moving to state WAITFORREADY";
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
                        sonar.upload_params(sonar.m_params);
                    } else {
                        static int i = 0;
                        if (i++ % 10 == 0) {
                            sonar.request_version();
                            info() << "Requesting version...";
                        }
                    }
                    break;
                case SeanetSonar::WAITFORPARAMS:
                    if (sonar.hasParams()) {
                        sonar.m_state = SeanetSonar::SCANNING;
                        debug() << "Moving to state SCANNING";
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

                    //info() << "Requesting data...";
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
