#include "sonar.h"

#include <boost/make_shared.hpp>

SonarNode::SonarNode(const std::string& device) : CauvNode("Sonar")
{
    m_sonar = boost::make_shared<SeanetSonar>(device);
}


void SonarNode::OnRun()
{
    m_sonar->init();
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
