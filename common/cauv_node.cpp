/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <common/cauv_logo.h> 

#include <common/zeromq/zeromq_mailbox.h>

#include <debug/cauv_debug.h>
#include <generated/message_observers.h>
#include <generated/types/DebugLevelMessage.h>
#include <utility/time.h>

#include "cauv_node.h"
#include "mailbox.h"

using namespace cauv;

const static char Version_Information[] = {
    #include <generated/version.h>
    , '\0'
};

CauvNode::~CauvNode()
{
    info() << "Shutting down node";
    m_event_monitor->stopMonitoring();
}

void CauvNode::run(bool synchronous)
{
    print_module_header(m_name);
    info() << "Module: " << m_name;
    info() << "Version:\n" << Version_Information;

    m_running = true;
    if(!synchronous)
        m_event_monitor->startMonitoringAsync();

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

int CauvNode::defaultOptions()
{
    return parseOptions(0, nullptr);
}

int CauvNode::parseOptions(int argc, char** argv)
{
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
      m_zeromq_mailbox(boost::make_shared<ZeroMQMailbox>(name)),
      m_mailbox(m_zeromq_mailbox),
      m_event_monitor(m_zeromq_mailbox),
      m_running(false)
{
    debug::setProgramName(name);
    
    addMessageObserver(boost::make_shared<DBGLevelObserver>());
    subMessage(DebugLevelMessage());
}

