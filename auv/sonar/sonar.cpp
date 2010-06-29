#include "sonar.h"

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>

#include "display_sonar_observer.h"

#include <debug/cauv_debug.h>

SonarNode::SonarNode() : CauvNode("Sonar")
{
}


class SpreadSonarObserver : public SonarObserver
{
    public:
        SpreadSonarObserver(boost::shared_ptr<ReconnectingSpreadMailbox> mailbox) : m_mailbox(mailbox)
        {
        }
        
        virtual void onReceiveDataLine(const SonarDataLine& data) 
        {
            boost::shared_ptr<SonarDataMessage> m = boost::make_shared<SonarDataMessage>(data);
            m_mailbox->sendMessage(m, SAFE_MESS);
        }
    protected:
        boost::shared_ptr<ReconnectingSpreadMailbox> m_mailbox;
};

void SonarNode::onRun()
{
    if(!m_sonar){
        error() << "no sonar device";
        return;
    }

    m_sonar->addObserver(boost::make_shared<SpreadSonarObserver>(mailbox()));
#ifdef DISPLAY_SONAR
    m_sonar->addObserver(boost::make_shared<DisplaySonarObserver>(m_sonar));
#endif
    m_sonar->init();

    addMessageObserver(boost::make_shared<SonarControlMessageObserver>(m_sonar));

    while (true) {
	    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    }
}

void SonarNode::addOptions(boost::program_options::options_description& desc,
                           boost::program_options::positional_options_description& pos)
{
    namespace po = boost::program_options;
    CauvNode::addOptions(desc, pos);
   
    desc.add_options()
        ("device,c", po::value<std::string>()->required(), "The device (eg /dev/ttyUSB1)")
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
    
    if(!m_sonar){
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
    node->parseOptions(argc, argv);
    node->run();
    cleanup();
    return 0;
}
