/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "seanet_sonar.h"

#include <iostream>
#include <vector>
#include <cmath>

#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>
#include <generated/types/SonarControlMessage.h>

#include "seanet_packet.h"

using namespace std;
using namespace cauv;

SonarControlMessageObserver::SonarControlMessageObserver(boost::shared_ptr<SeanetSonar> sonar)
{
    m_sonar = sonar;
}

void SonarControlMessageObserver::onSonarControlMessage(SonarControlMessage_ptr m)
{
    m_sonar->set_params(m->direction(), m->width(), m->gain(), m->range(), m->rangeRes(), m->angularRes());
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





SeanetSonar::SeanetSonar(const std::string& str) : m_cur_data_reqs(0)
{
    m_serial_port = boost::make_shared<SeanetSerialPort>(str);
    m_state = SENDREBOOT;
}

bool SeanetSonar::ok() const
{
    return m_serial_port && m_serial_port->ok();
}

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
void SeanetSonar::upload_head_params(SeanetHeadParams &params)
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


void SeanetSonar::set_direction(uint16_t direction)
{
    m_current_params.direction = direction;
    upload_current_params();
}
void SeanetSonar::set_width(uint16_t width)
{
    m_current_params.width = width;
    upload_current_params();
}
void SeanetSonar::set_gain (unsigned char gain)
{
    m_current_params.gain = gain;
    upload_current_params();
}
void SeanetSonar::set_range (uint32_t range)
{
    m_current_params.range = range;
    upload_current_params();
}
void SeanetSonar::set_range_res (uint32_t range_res)
{
    m_current_params.range_res = range_res;
    upload_current_params();
}
void SeanetSonar::set_angular_res (unsigned char angular_res)
{
    m_current_params.angular_res = angular_res;
    upload_current_params();
}
void SeanetSonar::set_params (uint16_t direction,
                              uint16_t width,
                              unsigned char gain,
                              uint32_t range,
                              uint32_t range_res,
                              unsigned char angular_res)
{
    debug() << "Setting params";
    m_current_params.direction = direction;
    m_current_params.width = width;
    m_current_params.gain = gain;
    m_current_params.range = range;
    m_current_params.range_res = range_res;
    m_current_params.angular_res = angular_res;
    upload_current_params();
}


void SeanetSonar::upload_current_params()
{
    SeanetHeadParams params;

    if (m_current_params.width == 6400)
    {
        // Continuous scanning
        params.hdCtrl |= HdCtrl::Continuous;
        params.hdCtrl &= ~HdCtrl::StareLLim;
    }
    else if (m_current_params.width == 0)
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

    params.stepSize = m_current_params.angular_res;
    params.igainChn1 = m_current_params.gain;
    params.igainChn2 = m_current_params.gain;
    params.rangeScale = m_current_params.range;

    if (m_current_params.direction - m_current_params.width/2 < 0 ) {
        m_current_params.direction += 6400;
    }

    params.leftLim = m_current_params.direction - m_current_params.width/2;
    params.rightLim = params.leftLim + m_current_params.width;
    
    uint16_t nBins = m_current_params.range / m_current_params.range_res;
    if (nBins > 1400) {
        warning() << "Someone tried to set the sonar resolution too high. Reducing it.";
        nBins = 1400;
    }

    float ping_time = (2 * m_current_params.range * 1000) / 1500; // units of microseconds. 1500 is VoS
    uint16_t adInterval = ping_time / (nBins * 0.64); // Sonar has clock time of 640ns

    debug() << "Setting nBins to " << nBins << " and adInterval to " << adInterval;
    debug() << "Range has been set to " << range_from_nbins_adinterval(nBins, adInterval) << "mm";

    params.adInterval = adInterval;
    params.nBins = nBins;
    params.rangeScale = m_current_params.range; // * 10;

    upload_head_params(params);
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
    head_data = reinterpret_cast<const SeanetHeadData*>(&pkt->data()[11]);

    // FIXME:  Assuming 8-bit mode again
   
    boost::shared_ptr<SonarDataLine> dataline = boost::make_shared<SonarDataLine>();
    dataline->data = std::vector< uint8_t >(&head_data->bin1, &head_data->bin1 + head_data->dBytes);
    dataline->range = range_from_nbins_adinterval(head_data->dBytes, head_data->adInterval);
    dataline->bearing = head_data->bearing;
    dataline->bearingRange = head_data->stepSize;
    dataline->scanWidth = head_data->rightLim - head_data->leftLim;
    
    //debug() << "Sending data line with range" << dataline->range << "mm, bearing" << dataline->bearing << ", and bearing range " << dataline->bearingRange;
    //{
    //    std::stringstream data;
    //    for (uint8_t c : dataline->data) {
    //        data << hex << setw(2) << setfill('0') << (int)c << " ";
    //    }
    //    debug() << data.str();
    //}

    for (observer_ptr_t o : m_observers)
    {
        o->onReceiveDataLine(*dataline);
    }
}



void cauv::sonarReadThread(SeanetSonar& sonar)
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
            boost::this_thread::interruption_point();

            try {
                pkt = sonar.m_serial_port->readPacket();
            }
            catch (SonarTimeoutException &e) {
                if (isalive) {
                    warning() << e.what();
                    sonar.m_serial_port->reset();
                    sonar.m_state = SeanetSonar::SENDREBOOT;
                    sonar.m_cur_data_reqs = 0;
                    isalive = false;
                }
	            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                continue;
            }
            catch (InvalidPacketException &e) {
                error() << "Sonar serial port received invalid packet: " << e.what();
                continue; 
            }

            /*cout << "Received packet: ";
            for (int j=0; j < pkt->m_length + 6; j++) {
                printf("%02x ", pkt->m_data[j]);
            }
            cout << endl;*/

            /* We want to update our interpretation of the state of the sonar
             * regardless of what state our state machine is currently in */
            
            switch  (pkt->type()) {
                case SeanetMessageType::Alive:
                    debug() << "Alive message received";
                    isalive = true;
                    sonar.m_motor_state.centered = pkt->data()[20] & HdInf::Centred;
                    sonar.m_motor_state.motoron = pkt->data()[20]  & HdInf::MotorOn;
                    sonar.m_motor_state.motoring = pkt->data()[20] & HdInf::Motoring;
                    sonar.m_motor_state.noparams = pkt->data()[20] & HdInf::NoParams;
                    sonar.m_motor_state.sentcfg = pkt->data()[20]  & HdInf::SentCfg;
                    sonar.m_motor_state.time = *(uint32_t*)&pkt->data()[14];

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
                    sonar.m_motor_state.cpuid = *(uint32_t*)&pkt->data()[14];
                    sonar.m_motor_state.proglength = *(uint32_t*)&pkt->data()[17];
                    sonar.m_motor_state.checksum = *(uint16_t*)&pkt->data()[21];
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
                    if (pkt->type() == SeanetMessageType::VersionData) {
                        debug() << "Moving to state WAITFORPARAMS";
                        sonar.m_state = SeanetSonar::WAITFORPARAMS;
                        sonar.upload_current_params();
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
                     if (pkt->type() == SeanetMessageType::HeadData) {
                        debug(3) << "Data received";
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
                        debug(3) << "Requesting" << sonar.m_cur_data_reqs << "sonar data";
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


void cauv::sonarProcessThread(SeanetSonar& sonar)
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
