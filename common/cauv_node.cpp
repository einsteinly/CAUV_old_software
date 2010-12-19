#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <utility/string.h>
#include <generated/messages_fwd.h>
#include <common/cauv_global.h> 
#include <common/spread/spread_rc_mailbox.h>
#include <common/spread/mailbox_monitor.h>
#include <common/spread/msgsrc_mb_observer.h>
#include <debug/cauv_debug.h>

#include "cauv_node.h"
#include "cauv_utils.h"

using namespace cauv;

CauvNode::~CauvNode()
{
	info() << "Shutting down node";
    debug::setCauvNode(NULL);
    m_event_monitor->stopMonitoringAsync();
}

void CauvNode::run(bool synchronous)
{
	cauv_global::print_module_header(m_name);

    m_mailbox->connect(MakeString() << m_port << "@" << m_server, m_name);
    if(!synchronous)
        m_event_monitor->startMonitoringAsync();

    debug::setCauvNode(this);

    onRun();

    if(synchronous)
        m_event_monitor->startMonitoringSync();
    else
        while(!m_interrupted)
        {
            msleep(500);
        }
}

void CauvNode::onRun()
{
}

void CauvNode::joinGroup(std::string const& group)
{
    if(m_mailbox)
        m_mailbox->joinGroup(group);
    else
        error() << "CauvNode::joinGroup: no mailbox";
}

void CauvNode::addMessageObserver(boost::shared_ptr<MessageObserver> o)
{
    if(m_mailbox_monitor)
        m_mailbox_monitor->addObserver(o);
    else
        error() << "CauvNode::addMessageObserver: no mailbox monitor";
}
void CauvNode::removeMessageObserver(boost::shared_ptr<MessageObserver> o)
{
    if(m_mailbox_monitor)
        m_mailbox_monitor->removeObserver(o);
    else
        error() << "CauvNode::removeMessageObserver: no mailbox monitor";
}
void CauvNode::clearMessageObservers()
{
    if(m_mailbox_monitor)
        m_mailbox_monitor->clearObservers();
    else
        error() << "CauvNode::clearObservers: no mailbox monitor";
}

int CauvNode::send(boost::shared_ptr<const Message> m, Spread::service st)
{
    if(m_mailbox)
        return m_mailbox->sendMessage(m, st);
    else
        error() << "CauvNode::sendMessage: no mailbox";
    return 0;
}

boost::shared_ptr<ReconnectingSpreadMailbox> CauvNode::mailbox() const
{
    return m_mailbox;
}

struct DBGLevelObserver: MessageObserver
{
    void onDebugLevelMessage(DebugLevelMessage_ptr m)
    {
        debug::setLevel(m->level());
    }
};

int CauvNode::defaultOptions()
{
    return parseOptions(0, NULL);
}

int CauvNode::parseOptions(int argc, char** argv)
{
    if(argv)
        debug::setProgramName(boost::filesystem::path(argv[0]).leaf());
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    po::positional_options_description pos;
    
    addOptions(desc, pos);
    
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
    po::notify(vm);

    return useOptionsMap(vm, desc);
}
void CauvNode::addOptions(boost::program_options::options_description& desc,
                          boost::program_options::positional_options_description& /*pos*/)
{
    namespace po = boost::program_options;
    desc.add_options()
        ("help,h", "Print this help message")
        ("server,s", po::value<std::string>(&m_server)->default_value("localhost"), "Server address for messages")
        ("port,p", po::value<unsigned int>(&m_port)->default_value(16707), "Server port for messages")
        ("verbose,v", po::value<unsigned int>()->implicit_value(1)->notifier(SmartStreamBase::setLevel), "Set the verbosity of debug messages")
    ;
}
int CauvNode::useOptionsMap(boost::program_options::variables_map& vm, boost::program_options::options_description& desc)
{
    if(vm.count("help"))
    {
        std::cout << desc;
        return 1;
    }
    return 0;
}


void CauvNode::stopNode(){
    m_interrupted = true;
    if(m_event_monitor)
        m_event_monitor->stopMonitoringAsync();
}

CauvNode::CauvNode(const std::string& name)
    : m_name(name),
      m_mailbox(boost::make_shared<ReconnectingSpreadMailbox>()),
      m_event_monitor(boost::make_shared<MailboxEventMonitor>(m_mailbox)),
      m_mailbox_monitor(boost::make_shared<MsgSrcMBMonitor>()),
      m_interrupted(false)
{
    debug::setProgramName(name);
    m_event_monitor->addObserver(m_mailbox_monitor);
    
    addMessageObserver(boost::make_shared<DBGLevelObserver>());
    joinGroup("debug");
}

