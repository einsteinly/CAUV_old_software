/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <utility/string.h>
#include <common/cauv_global.h> 
#include <common/spread/spread_rc_mailbox.h>
#include <common/spread/mailbox_monitor.h>
#include <debug/cauv_debug.h>
#include <generated/message_observers.h>
#include <generated/types/DebugLevelMessage.h>
#include <generated/types/MembershipChangedMessage.h>

#include "cauv_node.h"
#include "cauv_utils.h"
#include "mailbox.h"

using namespace cauv;

const static char Version_Information[] = {
    #include <generated/version.h>
    , '\0'
};

CauvNode::~CauvNode()
{
    info() << "Shutting down node";
    debug::setCauvNode(NULL);
    m_event_monitor->stopMonitoring();
}

void CauvNode::run(bool synchronous)
{
    cauv_global::print_module_header(m_name);
    info() << "Module: " << m_name;
    info() << "Version:\n" << Version_Information;

    if (m_spread_mailbox) {
        m_spread_mailbox->connect(MakeString() << m_port << "@" << m_server, m_name);
    }
    if(!synchronous)
        m_event_monitor->startMonitoringAsync();

    debug::setCauvNode(this);

    m_running = true;

    onRun();

    if(synchronous)
        m_event_monitor->startMonitoringSync();
    else
        while(m_event_monitor->isMonitoring())
            msleep(200);

    m_running = false;
}

bool CauvNode::isRunning(){
    return m_running;
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
    if(m_event_monitor)
        m_event_monitor->addMessageObserver(o);
    else
        error() << "CauvNode::addMessageObserver: no mailbox monitor";
}
void CauvNode::removeMessageObserver(boost::shared_ptr<MessageObserver> o)
{
    if(m_event_monitor)
        m_event_monitor->removeMessageObserver(o);
    else
        error() << "CauvNode::removeMessageObserver: no mailbox monitor";
}
void CauvNode::clearMessageObservers()
{
    if(m_event_monitor)
        m_event_monitor->clearMessageObservers();
    else
        error() << "CauvNode::clearObservers: no mailbox monitor";
}

int CauvNode::send(boost::shared_ptr<const Message> m, MessageReliability rel)
{
    if(m_mailbox)
        return m_mailbox->sendMessage(m, rel);
    else
        error() << "CauvNode::sendMessage: no mailbox";
    return 0;
}

boost::shared_ptr<Mailbox> CauvNode::mailbox() const
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
    if(argv && argc)
        debug::setProgramName(boost::filesystem::path(argv[0]).leaf());
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    po::positional_options_description pos;
    
    addOptions(desc, pos);
    info::addOptions(desc, pos);
    
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
        ("server,s", po::value<std::string>(&m_server)->default_value("localhost"), "Spread server address for messages")
        ("port,p", po::value<unsigned int>(&m_port)->default_value(16707), "Spread server port for messages")
        ("version,V", "show version information")
    ;
}
int CauvNode::useOptionsMap(boost::program_options::variables_map& vm, boost::program_options::options_description& desc)
{
    if(vm.count("help"))
    {
        std::cout << desc << std::flush;
        return 1;
    }
    if(vm.count("version"))
    {
        std::cout << Version_Information << std::flush;
        return 1;
    }
    return 0;
}


void CauvNode::stopNode(){
    if(m_event_monitor) {
        m_event_monitor->stopMonitoring();
    }
}

CauvNode::CauvNode(const std::string& name)
    : m_name(name),
      m_spread_mailbox(boost::make_shared<ReconnectingSpreadMailbox>()),
      m_mailbox(m_spread_mailbox),
      m_event_monitor(boost::make_shared<SpreadMailboxEventMonitor>(m_spread_mailbox)),
      m_running(false)
{
    debug::setProgramName(name);
    
    addMessageObserver(boost::make_shared<DBGLevelObserver>());
    joinGroup("debug");
}

