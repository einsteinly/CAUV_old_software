#include "gemini_node.h"

#include <boost/make_shared.hpp>
#include <boost/shared_array.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#include <debug/cauv_debug.h>
#include <common/cauv_utils.h>
#include <utility/threadsafe-observable.h>

#include "GeminiStructuresPublic.h"
#include "GeminiCommsPublic.h"

namespace cauv{

class GeminiObserver{
    public:
        virtual void onCGemPingHead(boost::shared_ptr<CGemPingHead const>){}
        virtual void onCGemPingLine(boost::shared_ptr<CGemPingLine const>){}
        virtual void onCGemPingTail(boost::shared_ptr<CGemPingTail const>){}
        virtual void onCGemPingTailExtended(boost::shared_ptr<CGemPingTailExtended const>){}
        virtual void onCGemStatusPacket(boost::shared_ptr<CGemStatusPacket const>){}
        virtual void onCGemAcknowledge(boost::shared_ptr<CGemAcknowledge const>){}
        virtual void onCGemBearingData(boost::shared_ptr<CGemBearingData const>){}
        virtual void onUnknownData(boost::shared_array<uint8_t>){}
};

class GeminiSonar: public ThreadSafeObservable<GeminiObserver>,
                   boost::noncopyable{
    public:
        GeminiSonar(uint16_t sonar_id, uint32_t inter_ping_musec=1000000)
            : m_sonar_id(sonar_id),
              m_inter_ping_musec(inter_ping_musec),
              m_conn_state(){
            assert(!the_sonar);
            the_sonar = this;
            m_conn_state.ok = true;
        }

        ~GeminiSonar(){
            the_sonar = NULL;
        }

        void init(){
            int success = 0;
            success = GEM_StartGeminiNetworkWithResult(m_sonar_id);
            if(!success){
                error() << "Could not start network connection to sonar" << m_sonar_id;
                m_conn_state.ok = false;
            }else{
                GEM_SetGeminiSoftwareMode("SeaNetC");
                GEM_SetHandlerFunction(&CallBackFn);
            }
        }

        bool ok() const{
            return m_conn_state.ok;
        }

    private:
        void onStatusPacket(boost::shared_ptr<CGemStatusPacket const> status_packet){
            // state transitions to manage the connection:
            // uninitialised
            debug() << "onStatusPacket:"
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
                    << "m_vga1aTemp =" << status_packet->m_vga1aTemp
                    << "m_vga1bTemp =" << status_packet->m_vga1bTemp
                    << "m_vga2aTemp =" << status_packet->m_vga2aTemp
                    << "m_vga2bTemp =" << status_packet->m_vga2bTemp
                    << "m_psu1Temp =" << status_packet->m_psu1Temp
                    << "m_psu2Temp =" << status_packet->m_psu2Temp
                    << "m_currentTimestampL =" << status_packet->m_currentTimestampL
                    << "m_currentTimestampH =" << status_packet->m_currentTimestampH
                    << "m_transducerFrequency =" << status_packet->m_transducerFrequency
                    << "m_subnetMask =" << status_packet->m_subnetMask
                    << "m_TX1Temp =" << status_packet->m_TX1Temp
                    << "m_TX2Temp =" << status_packet->m_TX2Temp
                    << "m_TX3Temp =" << status_packet->m_TX3Temp
                    << "m_BOOTSTSRegister =" << status_packet->m_BOOTSTSRegister
                    << "m_shutdownStatus =" << status_packet->m_shutdownStatus
                    << "m_dieOverTemp =" << status_packet->m_dieOverTemp
                    << "m_vga1aShutdownTemp =" << status_packet->m_vga1aShutdownTemp
                    << "m_vga1bShutdownTemp =" << status_packet->m_vga1bShutdownTemp
                    << "m_vga2aShutdownTemp =" << status_packet->m_vga2aShutdownTemp
                    << "m_vga2bShutdownTemp =" << status_packet->m_vga2bShutdownTemp
                    << "m_psu1ShutdownTemp =" << status_packet->m_psu1ShutdownTemp
                    << "m_psu2ShutdownTemp =" << status_packet->m_psu2ShutdownTemp
                    << "m_TX1ShutdownTemp =" << status_packet->m_TX1ShutdownTemp
                    << "m_TX2ShutdownTemp =" << status_packet->m_TX2ShutdownTemp
                    << "m_TX3ShutdownTemp =" << status_packet->m_TX3ShutdownTemp
                    << "m_linkType =" << status_packet->m_linkType
                    << "m_VDSLDownstreamSpeed1 =" << status_packet->m_VDSLDownstreamSpeed1
                    << "m_VDSLDownstreamSpeed2 =" << status_packet->m_VDSLDownstreamSpeed2
                    << "m_macAddress1 =" << status_packet->m_macAddress1
                    << "m_macAddress2 =" << status_packet->m_macAddress2
                    << "m_macAddress3 =" << status_packet->m_macAddress3
                    << "m_VDSLUpstreamSpeed1 =" << status_packet->m_VDSLUpstreamSpeed1
                    << "m_VDSLUpstreamSpeed2 =" << status_packet->m_VDSLUpstreamSpeed2;
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
                    
                    // !!! TODO
                    // GEM_AutoPingConfig(float range, unsigned short gain, float speedofsound)
                }
            }
        }

        // deleters are required for structures that are variable length,
        // either in-place (CGemPingLine), or holding pointers to data
        // (CGemBearingData) that is owned by the structure
        struct CGemPingLine_deleter{
            void operator()(CGemPingLine const* p) const{
                delete[] (uint8_t*) p;
            }
        };
        struct CGemBearingData_deleter{
            void operator()(CGemBearingData const* p) const{
                delete[] p->m_pData;
                delete p;
            }
        };
        static void CallBackFn(int eType, int len, char *data){
            boost::shared_ptr<CGemPingHead const> ping_head;
            boost::shared_ptr<CGemPingLine> ping_data;
            boost::shared_ptr<CGemPingTail const> ping_tail;
            boost::shared_ptr<CGemPingTailExtended const> ping_tail_ex;
            boost::shared_ptr<CGemStatusPacket const> status_packet;
            boost::shared_ptr<CGemAcknowledge const> acknowledge;
            boost::shared_ptr<CGemBearingData> bearing_data;
            boost::shared_array<uint8_t> unknown_data;
            
            switch(eType){
                case PING_HEAD:
                    ping_head = boost::make_shared<CGemPingHead const>(*(CGemPingHead*)data);
                    break;

                case PING_DATA:
                    debug() << "PING_DATA: len=" << len;
                    // here sizeof(X) + ... - 1 takes account that the first
                    // element of the ping data is stored *on* the last byte of
                    // the structure, then the ping data is continued in place.
                    // This is a very 'c' way of doing things, and really the
                    // structure should be POD...
                    ping_data = boost::shared_ptr<CGemPingLine>(
                        (CGemPingLine*) new uint8_t[sizeof(CGemPingLine) + len - 1],
                        CGemPingLine_deleter()
                    );
                    memcpy(&(*ping_data), data, sizeof(CGemPingLine) + len - 1);
                    break;

                case PING_TAIL:
                    ping_tail = boost::make_shared<CGemPingTail const>(*(CGemPingTail*)data);
                    break;

                case PING_TAIL_EX:
                    ping_tail_ex = boost::make_shared<CGemPingTailExtended const>(*(CGemPingTailExtended*)data);
                    break;

                case GEM_STATUS:
                    status_packet = boost::make_shared<CGemStatusPacket const>(*(CGemStatusPacket*)data);
                    break; 

                case GEM_ACKNOWLEDGE:
                    acknowledge = boost::make_shared<CGemAcknowledge const>(*(CGemAcknowledge*)data);
                    break;

                case GEM_BEARING_DATA:
                    bearing_data = boost::shared_ptr<CGemBearingData>(
                        new CGemBearingData(*(CGemBearingData*)data),
                        CGemBearingData_deleter()
                    );
                    bearing_data->m_pData = new uint8_t[bearing_data->m_noSamples];
                    if (bearing_data->m_pData) {
                        // Copy the data from the original structure to the new structure
                        std::memcpy(bearing_data->m_pData, ((CGemBearingData*)data)->m_pData, bearing_data->m_noSamples);
                    }else{
                        // Failed to allocate memory
                        bearing_data->m_noSamples = 0;
                    }
                    break;

                case GEM_UNKNOWN_DATA:
                    debug() << "GEM: unknown data";
                    /*unknown_data = boost::shared_array<uint8_t>(new uint8_t[len]);
                    if(unknown_data){
                        memcpy(unknown_data, data, len);
                    }*/
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
                        p->onCGemPingHead(ping_head);
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

        volatile struct{
            bool ok;
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

    while (true) {
	    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
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

