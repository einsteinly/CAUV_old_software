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
        virtual void onCGemStatusPacket(boost::shared_ptr<CGemStatusPacket const>){}
};

class GeminiSonar: public ThreadSafeObservable<GeminiObserver>,
                   boost::noncopyable{
    public:
        GeminiSonar(uint16_t sonar_id)
            : m_ok(true), m_sonar_id(sonar_id){
            assert(!the_sonar);
            the_sonar = this;
        }

        ~GeminiSonar(){
            the_sonar = NULL;
        }

        void init(){
            int success = 0;
            success = GEM_StartGeminiNetworkWithResult(m_sonar_id);
            if(!success){
                error() << "Could not start network connection to sonar" << m_sonar_id;
                m_ok = false;
            }else{
                GEM_SetGeminiSoftwareMode("SeaNetC");
                GEM_SetHandlerFunction(&CallBackFn);
            }

        }

        bool ok() const{
            return m_ok;
        }

    private:
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
            
            // deleters are required for structures that are variable length,
            // either in-place (CGemPingLine), or holding pointers to data
            // (CGemBearingData) that is owned by the structure
            const static CGemPingLine_deleter a_CGemPingLine_deleter;
            const static CGemBearingData_deleter a_CGemBearingData_deleter;

            CGemBearingData *pBearing;
            CGemBearingData *copyBearing;
            
            switch(eType){
                case PING_HEAD:
                    ping_head = boost::make_shared<CGemPingHead const>(*(CGemPingHead*)data);
                    break;
                case PING_DATA:
                    debug() << "PING_DATA: len=" << len;
                    ping_data = boost::shared_ptr<CGemPingLine>(
                        (CGemPingLine*) new uint8_t[offsetof(CGemPingLine, m_startOfData) + len],
                        CGemPingLine_deleter()
                    );
                    memcpy(&(*ping_data), data, offsetof(CGemPingLine, m_startOfData) + len);
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
                        std::memcpy(bearing_data->m_pData, pBearing->m_pData, copyBearing->m_noSamples);
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

            // actually dispatch messages:
            if(the_sonar){
                boost::lock_guard<boost::recursive_mutex> l(the_sonar->m_observers_lock);
                if(ping_data)
                    foreach(observer_ptr_t& p, the_sonar->m_observers)
                        ; // TODO
                else if(ping_tail)
                    foreach(observer_ptr_t& p, the_sonar->m_observers)
                        ; // TODO
                else if(ping_tail_ex)
                    foreach(observer_ptr_t& p, the_sonar->m_observers)
                        ; // TODO
                else if(status_packet)
                    foreach(observer_ptr_t& p, the_sonar->m_observers)
                        p->onCGemStatusPacket(status_packet);
                else if(acknowledge)
                    foreach(observer_ptr_t& p, the_sonar->m_observers)
                        ; // TODO
                else if(bearing_data)
                    foreach(observer_ptr_t& p, the_sonar->m_observers)
                        ; // TODO
                else if(unknown_data)
                    foreach(observer_ptr_t& p, the_sonar->m_observers)
                        ; // TODO
            }else{
                error() << "No GeminiSonar object to receive callback";
            }
        }
        

        bool m_ok;
        uint16_t m_sonar_id;
        struct{
            uint16_t sonarID;
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

