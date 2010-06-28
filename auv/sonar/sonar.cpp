#include "sonar.h"

#include <boost/make_shared.hpp>

#include "display_sonar_observer.h"

#include <debug/cauv_debug.h>

SonarNode::SonarNode(const std::string& device) : CauvNode("Sonar")
{
    m_sonar = boost::make_shared<SeanetSonar>(device);
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
    if (argc != 2)
    {
        std::cout << "USAGE: " << argv[0] << " DEVICE" << std::endl;
        return 1;
    }
    std::string device(argv[1]);
    node = new SonarNode(device);
    node->run();
    cleanup();
    return 0;
}
