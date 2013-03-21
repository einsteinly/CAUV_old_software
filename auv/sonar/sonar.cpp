/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "sonar.h"

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>

#include <debug/cauv_debug.h>
#include <common/mailbox.h>
#include <generated/types/SonaroutGroup.h>
#include <generated/types/SonarctlGroup.h>
#include <generated/types/TelemetryMessage.h>
#include <sonar/seanet_sonar.h>

#include "display_sonar_observer.h"

using namespace cauv;

SonarNode::SonarNode() : CauvNode("Sonar")
{
}


class SpreadSonarObserver : public SonarObserver, public MessageObserver
{
    public:
        SpreadSonarObserver(boost::shared_ptr<Mailbox> mailbox) : m_mailbox(mailbox)
        {
        }
        
        virtual void onReceiveDataLine(const SonarDataLine& data) 
        {
            SonarDataLine yawAdjustedData = data;
            yawAdjustedData.bearing -= int(m_orientation.yaw*6400/360);
                
            boost::shared_ptr<SonarDataMessage> m = boost::make_shared<SonarDataMessage>(yawAdjustedData);
            std::cout << "=";
            m_mailbox->sendMessage(m, RELIABLE_MSG);
        }
    protected:
        boost::shared_ptr<Mailbox> m_mailbox;

        floatYPR m_orientation;

        virtual void onTelemetryMessage(TelemetryMessage_ptr m)
        {
            m_orientation = m->orientation();
        }
};

void SonarNode::onRun()
{
    if(!m_sonar){
        error() << "no sonar device";
        return;
    }
    
    joinGroup("sonarctl");
    joinGroup("telemetry");

    boost::shared_ptr<SpreadSonarObserver> spreadSonarObserver = boost::make_shared<SpreadSonarObserver>(mailbox());
    m_sonar->addObserver(spreadSonarObserver);
    addMessageObserver(spreadSonarObserver);
    
#ifdef DISPLAY_SONAR
    m_sonar->addObserver(boost::make_shared<DisplaySonarObserver>(m_sonar));
#endif

    addMessageObserver(boost::make_shared<SonarControlMessageObserver>(m_sonar));
    
    m_sonar->init();
}

void SonarNode::addOptions(boost::program_options::options_description& desc,
                           boost::program_options::positional_options_description& pos)
{
    namespace po = boost::program_options;
    CauvNode::addOptions(desc, pos);
   
    desc.add_options()
        ("device,d", po::value<std::string>()->required(), "The device (eg /dev/ttyUSB1)")
    ;

    pos.add("device", 1);
}

int SonarNode::useOptionsMap(boost::program_options::variables_map& vm,
                             boost::program_options::options_description& desc)
{
    namespace po = boost::program_options;
    int ret = CauvNode::useOptionsMap(vm, desc);
    if (ret != 0) return ret;
    
    std::string device = vm["device"].as<std::string>();
    m_sonar = boost::make_shared<SeanetSonar>(device);
    
    if(!m_sonar->ok()){
        error() << "could not open device" << device;
        return 2;
    }

    return 0;
}

static SonarNode* node;

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
    node = new SonarNode();
    if(node->parseOptions(argc, argv) == 0)
        node->run();
    cleanup();
    return 0;
}
