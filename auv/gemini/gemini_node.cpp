#include "gemini_node.h"

#include <limits>
#include <cmath>

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#include <debug/cauv_debug.h>
#include <common/cauv_utils.h>
#include <common/image.h>
#include <utility/threadsafe-observable.h>
#include <utility/rounding.h>
#include <generated/types/TimeStamp.h>
#include <generated/types/GeminiStatusMessage.h>
#include <generated/types/SonarImageMessage.h>
#include <generated/types/SpeedOfSoundMessage.h>

// Gemini SDK comes last because it pollutes the global namespace
#include "GeminiStructuresPublic.h"
#include "GeminiCommsPublic.h"

namespace cauv{

static float floatTemp(uint16_t temp){
    int16_t signedtemp;
    float  ftemp;
    bool present = temp & 0x8000;
    if(present){
        // 10 bits of data:
        signedtemp = temp & 0x3ff;
        // sign extend
        bool negative = temp & 0x200;
        if(negative)
            signedtemp |= 0xfc00;
        ftemp = signedtemp / 4.0f;
        return ftemp;
    }else{
        return std::numeric_limits<float>::quiet_NaN();
    }
}

static std::string fmtTemp(uint16_t temp){
    if(temp & 0x8000)
        return mkStr() << floatTemp(temp);
    else
        return "X";
}

class GeminiObserver{
    public:
        virtual void onCGemPingHead(CGemPingHead const*, float){}
        virtual void onCGemPingLine(CGemPingLine const*){}
        virtual void onCGemPingTail(CGemPingTail const*){}
        virtual void onCGemPingTailExtended(CGemPingTailExtended const*){}
        virtual void onCGemStatusPacket(CGemStatusPacket const*){}
        virtual void onCGemAcknowledge(CGemAcknowledge const*){}
        virtual void onCGemBearingData(CGemBearingData const*){}
        virtual void onUnknownData(uint8_t const*){}
};

class ReBroadcaster: public GeminiObserver{
    public:
        ReBroadcaster(CauvNode& node)
            : m_node(node){
        }
        
        virtual void onCGemPingHead(CGemPingHead const* h, float range){
            const float sos = h->m_spdSndVel / 10.0f;
            debug() << "PingHead:"
                    << "start:" << h->m_startRange
                    << "end:" << h->m_endRange
                    << "numBeams:" << h->m_numBeams
                    << "numChans:" << (int)h->m_numChans
                    << "sampChan:" << (int)h->m_sampChan
                    << "speedOfSound:" << sos;
            // new ping:
            // each line is data from a particular distance away
            uint32_t num_lines = h->m_endRange - h->m_startRange;
            // each beam is data from a constant bearing
            uint32_t num_beams = h->m_numBeams;
            m_current_ping_id = h->m_pingID;
            m_current_ping_time = now();
            //m_current_image = boost::make_shared<Image>(cv::Mat::zeros(num_lines, num_beams, CV_8U));
            m_current_msg = boost::make_shared<SonarImageMessage>();
            m_current_msg->source(SonarID::Gemini);
            // !!! sneaky: probably okay, and really the only efficient way to
            // do this without introducing multableX members to messages. This
            // is okay because we haven't serialised it yet.
            const_cast<PolarImage&>(m_current_msg->image()).data.resize(num_lines*num_beams);
            const_cast<PolarImage&>(m_current_msg->image()).encoding = ImageEncodingType::RAW_uint8_1;
            const_cast<PolarImage&>(m_current_msg->image()).bearing_bins = computeBearingBins(num_beams);
            const_cast<PolarImage&>(m_current_msg->image()).rangeStart = h->m_startRange;
            const_cast<PolarImage&>(m_current_msg->image()).rangeEnd = h->m_endRange;
            // !!! TODO: not sure about this at all, check with Tritech
            const_cast<PolarImage&>(m_current_msg->image()).rangeConversion = range / h->m_endRange;
            debug() << "New ping: ID=" << m_current_ping_id << "lines:" << num_lines << "beams:" << num_beams;

            // This message also tells us the speed of sound: so broadcast it
            // now:
            m_node.send(boost::make_shared<SpeedOfSoundMessage>(sos));
        }

        virtual void onCGemPingLine(CGemPingLine const* l){
            // accumulate this line of equidistant data:
            int32_t line_id = l->m_lineID;
            int32_t line_idx = line_id - m_current_msg->image().rangeStart;
            uint16_t ping_id = l->m_pingID;
            if((ping_id & 0xff) != (m_current_ping_id & 0xff)){
                debug() << "bad pingID";
                return;
            }
            // !!! sneaky: probably okay, and really the only efficient way to
            // do this without introducing multableX members to messages. This
            // is okay because we haven't serialised it yet.
            std::vector<uint8_t> &data = const_cast<PolarImage&>(m_current_msg->image()).data;
            uint32_t num_beams = m_current_msg->image().bearing_bins.size() - 1;
            uint32_t offset = line_idx * num_beams;
            if(offset + num_beams > data.size())
                error() << "invalid line index";
            for(uint32_t i = 0; i != num_beams; i++)
                data[offset+i] = ((uint8_t*)&(l->m_startOfData))[i];
        }
         
        virtual void onCGemPingTailExtended(CGemPingTailExtended const*){
            pingComplete();
        }

        virtual void onCGemPingTail(CGemPingTail const*){
            pingComplete();
        }

        virtual void onCGemStatusPacket(CGemStatusPacket const* s){
            boost::shared_ptr<GeminiStatusMessage> r = boost::make_shared<GeminiStatusMessage>();
            r->sonarId(s->m_sonarId);
            r->vccInt(s->m_vccInt);
            r->vccAux(s->m_vccAux);
            r->dcVolt(s->m_dcVolt);
            r->dieTemp(((s->m_dieTemp * 503.975) / 1024.0) - 273.15);
            r->vga1aTemp(floatTemp(s->m_vga1aTemp));
            r->vga1bTemp(floatTemp(s->m_vga1bTemp));
            r->vga2aTemp(floatTemp(s->m_vga2aTemp));
            r->vga2bTemp(floatTemp(s->m_vga2bTemp));
            r->TX1Temp(floatTemp(s->m_TX1Temp));
            r->TX2Temp(floatTemp(s->m_TX2Temp));
            r->TX3Temp(floatTemp(s->m_TX3Temp));
            r->dieOverTemp(((s->m_dieOverTemp * 503.975) / 1024.0) - 273.15);
            r->vga1aShutdownTemp(floatTemp(s->m_vga1aShutdownTemp));
            r->vga1bShutdownTemp(floatTemp(s->m_vga1bShutdownTemp));
            r->vga2aShutdownTemp(floatTemp(s->m_vga2aShutdownTemp));
            r->vga2bShutdownTemp(floatTemp(s->m_vga2bShutdownTemp));
            r->TX1ShutdownTemp(floatTemp(s->m_TX1ShutdownTemp));
            r->TX2ShutdownTemp(floatTemp(s->m_TX2ShutdownTemp));
            r->TX3ShutdownTemp(floatTemp(s->m_TX3ShutdownTemp));
            switch(s->m_transducerFrequency){
                case 0: r->transducerFrequency(868.0f); break;
                case 1: r->transducerFrequency(723.0f); break;
            }
            r->linkType(s->m_linkType);
            r->BOOTSTSRegister(s->m_BOOTSTSRegister);
            r->shutdownStatus(s->m_shutdownStatus);
            
            m_node.send(r);
        }
         
    private:
        static std::vector<int32_t> computeBearingBinsNoCache(uint32_t num_beams){
            std::vector<int32_t> r;
            r.reserve(num_beams+2);
            const double radConvert = (180.0 / M_PI) * (6400.0/360.0) * 0x10000;
            for(uint32_t beam = 0; beam <= num_beams; beam++){
                // see Gemini Interface Specification Appendix B
                double radians = std::asin(((2*(beam+0.5) - num_beams) / num_beams) * 0.86602540);
                r.push_back(round(radConvert * radians));
            }
            return r;
        }
        static std::vector<int32_t> computeBearingBins(uint32_t num_beams){
            debug() << "computeBearings numbeams=" << num_beams;
            static std::vector<int32_t> bearings_256 = computeBearingBinsNoCache(256);
            if(num_beams == 256)
                return bearings_256;
            return computeBearingBinsNoCache(num_beams);
        }

        void pingComplete(){
            debug() << "sending SonarImageMessage...";
            m_node.send(m_current_msg);
        }

        uint16_t m_current_ping_id;
        TimeStamp m_current_ping_time;
        boost::shared_ptr<SonarImageMessage> m_current_msg;
        CauvNode& m_node;
};

class GeminiSonar: public ThreadSafeObservable<GeminiObserver>,
                   boost::noncopyable{
    public:
        GeminiSonar(uint16_t sonar_id, uint32_t inter_ping_musec=1000000)
            : m_sonar_id(sonar_id),
              m_inter_ping_musec(inter_ping_musec),
              m_range(0),
              m_gain_percent(0),
              m_conn_state(){
            assert(!the_sonar);
            the_sonar = this;
            m_conn_state.ok = true;
            m_conn_state.initialised = false;
        }

        ~GeminiSonar(){
            the_sonar = NULL;
        }

        float range() const{
            return m_range;
        }

        void autoConfig(float range, float gain_percent){
            m_range = range;
            m_gain_percent = gain_percent;
            GEM_AutoPingConfig(m_range, m_gain_percent, 1499.2f);
        }

        void init(){
            int success = 0;
            success = GEM_StartGeminiNetworkWithResult(m_sonar_id);
            if(!success){
                error() << "Could not start network connection to sonar" << m_sonar_id;
                m_conn_state.ok = false;
            }else{
                // ('Evo' is the native mode for the sonar, John told us to use
                // this mode, SeaNet is there for compatibility with Tritech's
                // old software only. 'EvoC' means use range-line compression:
                // lines further away are compres)
                GEM_SetGeminiSoftwareMode("EvoC");
                GEM_SetHandlerFunction(&CallBackFn);
            }
        }

        bool ok() const{
            return m_conn_state.ok;
        }

        bool initialised() const{
            return m_conn_state.initialised;
        }

    private:
        static std::string fmtIp(uint32_t ip4_addr){
            uint8_t ipchrs[4] = {
                (ip4_addr >> 24) & 0xff,
                (ip4_addr >> 16) & 0xff,
                (ip4_addr >>  8) & 0xff,
                (ip4_addr >>  0) & 0xff
            };
            return mkStr() << int(ipchrs[0]) << "."
                           << int(ipchrs[1]) << "."
                           << int(ipchrs[2]) << "."
                           << int(ipchrs[3]);
        }
        
        void onStatusPacket(CGemStatusPacket const* status_packet){
            // state transitions to manage the connection:
            // uninitialised
            debug(8) << "onStatusPacket:"
                    << "m_firmwareVer =" << status_packet->m_firmwareVer
                    << "m_sonarId =" << status_packet->m_sonarId
                    << "m_sonarFixIp =" << status_packet->m_sonarFixIp
                    << "m_sonarAltIp =" << status_packet->m_sonarAltIp
                    << "m_surfaceIp =" << status_packet->m_surfaceIp
                    << "m_flags =" << status_packet->m_flags
                    << "m_vccInt =" << status_packet->m_vccInt
                    << "m_vccAux =" << status_packet->m_vccAux
                    << "m_dcVolt =" << status_packet->m_dcVolt
                    << "m_dieTemp =" << status_packet->m_dieTemp
                    << "m_tempX =" << status_packet->m_tempX
                    << "m_vga1aTemp =" << fmtTemp(status_packet->m_vga1aTemp)
                    << "m_vga1bTemp =" << fmtTemp(status_packet->m_vga1bTemp)
                    << "m_vga2aTemp =" << fmtTemp(status_packet->m_vga2aTemp)
                    << "m_vga2bTemp =" << fmtTemp(status_packet->m_vga2bTemp)
                    << "m_psu1Temp =" << fmtTemp(status_packet->m_psu1Temp)
                    << "m_psu2Temp =" << fmtTemp(status_packet->m_psu2Temp)
                    << "m_currentTimestampL =" << status_packet->m_currentTimestampL
                    << "m_currentTimestampH =" << status_packet->m_currentTimestampH
                    << "m_transducerFrequency =" << status_packet->m_transducerFrequency
                    << "m_subnetMask =" << status_packet->m_subnetMask
                    << "m_TX1Temp =" << fmtTemp(status_packet->m_TX1Temp)
                    << "m_TX2Temp =" << fmtTemp(status_packet->m_TX2Temp)
                    << "m_TX3Temp =" << fmtTemp(status_packet->m_TX3Temp)
                    << "m_BOOTSTSRegister =" << status_packet->m_BOOTSTSRegister
                    << "m_shutdownStatus =" << status_packet->m_shutdownStatus
                    << "m_dieOverTemp =" << status_packet->m_dieOverTemp
                    << "m_vga1aShutdownTemp =" << fmtTemp(status_packet->m_vga1aShutdownTemp)
                    << "m_vga1bShutdownTemp =" << fmtTemp(status_packet->m_vga1bShutdownTemp)
                    << "m_vga2aShutdownTemp =" << fmtTemp(status_packet->m_vga2aShutdownTemp)
                    << "m_vga2bShutdownTemp =" << fmtTemp(status_packet->m_vga2bShutdownTemp)
                    << "m_psu1ShutdownTemp =" << fmtTemp(status_packet->m_psu1ShutdownTemp)
                    << "m_psu2ShutdownTemp =" << fmtTemp(status_packet->m_psu2ShutdownTemp)
                    << "m_TX1ShutdownTemp =" << fmtTemp(status_packet->m_TX1ShutdownTemp)
                    << "m_TX2ShutdownTemp =" << fmtTemp(status_packet->m_TX2ShutdownTemp)
                    << "m_TX3ShutdownTemp =" << fmtTemp(status_packet->m_TX3ShutdownTemp)
                    << "m_linkType =" << status_packet->m_linkType
                    << "m_VDSLDownstreamSpeed1 =" << status_packet->m_VDSLDownstreamSpeed1
                    << "m_VDSLDownstreamSpeed2 =" << status_packet->m_VDSLDownstreamSpeed2
                    << "m_macAddress1 =" << status_packet->m_macAddress1
                    << "m_macAddress2 =" << status_packet->m_macAddress2
                    << "m_macAddress3 =" << status_packet->m_macAddress3
                    << "m_VDSLUpstreamSpeed1 =" << status_packet->m_VDSLUpstreamSpeed1
                    << "m_VDSLUpstreamSpeed2 =" << status_packet->m_VDSLUpstreamSpeed2;
            debug() << "sonarId:"   << status_packet->m_sonarId
                    << "FixIp:"     << fmtIp(status_packet->m_sonarFixIp)
                    << "AltIp:"     << fmtIp(status_packet->m_sonarAltIp)
                    << "SurfaceIp:" << fmtIp(status_packet->m_surfaceIp)
                    << "NetMask:"   << fmtIp(status_packet->m_subnetMask)
                    << "ShutdownStatus:" <<  status_packet->m_shutdownStatus
                    << "LinkType:"  << status_packet->m_linkType;
            if(m_conn_state.sonarId == 0){
                if(status_packet){
                    m_conn_state.sonarId = status_packet->m_sonarId;
                    m_conn_state.sonarAltIp = status_packet->m_sonarAltIp;
                }
                if(m_conn_state.sonarId != 0){
                    GEM_SetDLLSonarID(m_conn_state.sonarId);
                   unsigned char ip_chrs[4] = {
                        (m_conn_state.sonarAltIp >> 24) & 0xff,
                        (m_conn_state.sonarAltIp >> 16) & 0xff,
                        (m_conn_state.sonarAltIp >> 8) & 0xff,
                        (m_conn_state.sonarAltIp >> 0) & 0xff
                    };
                    unsigned char nm_chrs[4] = {
                        255,255,255,0
                    };
                    // this prints in the right format (according to the
                    // manual), so if endian issues happen, they'll be visible
                    debug() << "GEM_UseAltSonarIPAddress("
                            << (int)ip_chrs[0] << "."
                            << (int)ip_chrs[1] << "."
                            << (int)ip_chrs[2] << "."
                            << (int)ip_chrs[3] << "/"
                            << (int)nm_chrs[0] << "."
                            << (int)nm_chrs[1] << "."
                            << (int)nm_chrs[2] << "."
                            << (int)nm_chrs[3] << ")";
                    // there also exists SetAltSonarIpAddress, which programs
                    // it in flash and requires a reboot
                    GEM_UseAltSonarIPAddress(
                        ip_chrs[0], ip_chrs[1], ip_chrs[2], ip_chrs[3],
                        nm_chrs[0], nm_chrs[1], nm_chrs[2], nm_chrs[3]
                    );
                    // otherwise, the default address of 10.61.19.200 is used
                    // to talk to the head
                    GEM_TxToAltIPAddress(true);
                    // 0 = only ping on receipt of ping configuration message,
                    // 1 = ping continuously
                    GEM_SetPingMode(0);
                    GEM_SetInterPingPeriod(m_inter_ping_musec);
                    //
                    GEM_SetVelocimeterMode(
                        0, // 0 = auto gain
                           // 1 = manual gain (actually, auto is always true anyway)
                        0  // 0 = use velocimeter calculated speed of sound,
                           // 1 = use speed set in config message
                    );
                    m_conn_state.initialised = true;
                }
            }
        }

        static void CallBackFn(int eType, int len, char *data){
            CGemPingHead const* ping_head = NULL;
            CGemPingLine const* ping_data = NULL;
            CGemPingTail const* ping_tail = NULL;
            CGemPingTailExtended const* ping_tail_ex = NULL;
            CGemStatusPacket const* status_packet = NULL;
            CGemAcknowledge const* acknowledge = NULL;
            CGemBearingData const* bearing_data = NULL;
            uint8_t const* unknown_data = NULL;

            debug(8) << "RX:" << eType;
            switch(eType){
                case PING_HEAD:
                    debug(6) << "RX: ping head";
                    ping_head = (CGemPingHead*)data;
                    break;

                case PING_DATA:
                    debug(7) << "RX: ping line: len=" << len;
                    ping_data = (CGemPingLine*)data;
                    break;

                case PING_TAIL:
                    // Normally we get PING_TAIL_EX, but this happens if the
                    // retry data structures were never created by the library,
                    // so there's no extra data for the library to add.
                    // (not sure why this might be, but it does happen)
                    ping_tail = (CGemPingTail*)data;
                    debug() << "RX: ping tail: id=" << ping_tail->m_pingID;
                    break;

                case PING_TAIL_EX:
                    ping_tail_ex = (CGemPingTailExtended*)data;
                    debug(6) << "RX: ping tail ex: id=" << int(ping_tail_ex->m_pingID);
                    debug() << BashColour::Purple
                            << "retry 1:"       << ping_tail_ex->m_firstPassRetries
                            << "retry 2:"       << ping_tail_ex->m_secondPassRetries
                            << "tail retries:"  << ping_tail_ex->m_tailRetries
                            << "RX Error count:"<< ping_tail_ex->m_recvErrorCount
                            << "RX Total:"      << ping_tail_ex->m_packetCount
                            << "Lines lost:"    << ping_tail_ex->m_linesLostThisPing;
                           
                    break;

                case GEM_STATUS:
                    debug(6) << "RX: status";
                    status_packet = (CGemStatusPacket*)data;
                    break;

                case GEM_ACKNOWLEDGE:
                    debug(6) << "RX: ack";
                    acknowledge = (CGemAcknowledge*)data;
                    break;

                case GEM_BEARING_DATA:
                    debug(6) << "RX: bearing data";
                    bearing_data = (CGemBearingData*)data;
                    break;

                case GEM_UNKNOWN_DATA:
                    debug() << "GEM: unknown data";
                    unknown_data = (uint8_t const*)data;
                    break;

                case GEM_IP_CHANGED:
                    debug() << "GEM: ip changed";
                    break;

                default:
                    debug() << "GEM: unknown type:" << eType;
                    break;
            }

            // actually dispatch messages to anyone interested:
            if(the_sonar){
                // first use status packet to drive state transitions (eg, for
                // initialisation):
                if(status_packet)
                    the_sonar->onStatusPacket(status_packet);

                boost::lock_guard<boost::recursive_mutex> l(the_sonar->m_observers_lock);
                foreach(observer_ptr_t& p, the_sonar->m_observers){
                    if(ping_head)
                        p->onCGemPingHead(ping_head, the_sonar->range());
                    else if(ping_data)
                        p->onCGemPingLine(ping_data);
                    else if(ping_tail)
                        p->onCGemPingTail(ping_tail);
                    else if(ping_tail_ex)
                        p->onCGemPingTailExtended(ping_tail_ex);
                    else if(status_packet)
                        p->onCGemStatusPacket(status_packet);
                    else if(acknowledge)
                        p->onCGemAcknowledge(acknowledge);
                    else if(bearing_data)
                        p->onCGemBearingData(bearing_data);
                    else if(unknown_data)
                        p->onUnknownData(unknown_data);
                }
            }else{
                error() << "No GeminiSonar object to receive callback";
            }
        }


        uint16_t m_sonar_id;
        uint32_t m_inter_ping_musec;
        float m_range;
        float m_gain_percent;

        volatile struct{
            bool ok;
            bool initialised;
            uint16_t sonarId;
            uint32_t sonarAltIp;
        } m_conn_state;

        // there should only be one instance of this at a time, keep track of
        // it so that callbacks can be dispatched
        static GeminiSonar* the_sonar;
};
GeminiSonar* GeminiSonar::the_sonar = NULL;

} // namespace cauv

using namespace cauv;

static GeminiNode* node;


GeminiNode::GeminiNode()
    : CauvNode("Gemini"){
}

void GeminiNode::onRun()
{

    if(!m_sonar){
        error() << "no sonar device";
        return;
    }

    /*
    joinGroup("sonarctl");
    joinGroup("telemetry");

    boost::shared_ptr<SpreadSonarObserver> spreadSonarObserver = boost::make_shared<SpreadSonarObserver>(mailbox());
    m_sonar->addObserver(spreadSonarObserver);
    addMessageObserver(spreadSonarObserver);

#ifdef DISPLAY_SONAR
    m_sonar->addObserver(boost::make_shared<DisplaySonarObserver>(m_sonar));
#endif

    addMessageObserver(boost::make_shared<SonarControlMessageObserver>(m_sonar));
    */

    m_sonar->init();

    while(!m_sonar->initialised()){
        debug() << "waiting for init...";
	    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }

    m_sonar->addObserver(boost::make_shared<ReBroadcaster>(boost::ref(*this)));

    debug() << "autoConfig...";
    m_sonar->autoConfig(5.0f, 50);

    while (true) {
	    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        //GEM_AutoPingConfig(float range, unsigned short gain, float speedofsound)
        //GEM_SendGeminiPingConfig();
        debug() << "GEM_SendPingConfig...";
        GEM_SendGeminiPingConfig();
    }
}



void GeminiNode::addOptions(po::options_description& desc,
                            po::positional_options_description& pos)
{
    CauvNode::addOptions(desc, pos);
    debug() << "GeminiNode::addOptions";

    desc.add_options()
        ("sonar_id,d", po::value<uint16_t>()->required(), "The sonar ID (eg 1)")
    ;

    pos.add("sonar_id", 1);

}

int GeminiNode::useOptionsMap(po::variables_map& vm,
                              po::options_description& desc)
{
    int ret = CauvNode::useOptionsMap(vm, desc);
    debug() << "GeminiNode::useOptionsMap";
    if (ret != 0) return ret;

    uint16_t sonar_id = vm["sonar_id"].as<uint16_t>();
    m_sonar = new GeminiSonar(sonar_id);

    /*
    if(!m_sonar->ok()){
        error() << "could not open device" << device;
        return 2;
    }
    */

    return 0;
}

void cleanup()
{
    info() << "Cleaning up...";
    CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    info() << "Clean up done.";
}

void interrupt(int sig)
{
    std::cout << std::endl;
    info() << BashColour::Red << "Interrupt caught!";
    cleanup();
    signal(SIGINT, SIG_DFL);
    raise(sig);
}

int main(int argc, char** argv)
{
    signal(SIGINT, interrupt);
    node = new GeminiNode();
    try{
        if(node->parseOptions(argc, argv) == 0)
            node->run();
    }catch(boost::program_options::error& e){
        error() << e.what();
    }
    cleanup();
    return 0;
}

