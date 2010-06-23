#include <iostream>
#include <limits>
#include <string>

#include <common/cauv_global.h>

#include <common/spread/spread_rc_mailbox.h>
#include <common/spread/mailbox_monitor.h>
#include <common/spread/msgsrc_mb_observer.h>

#include "cauv_node.h"
#include "cauv_utils.h"


using namespace std;


CauvNode::~CauvNode()
{
	info() << "Shutting down node";
    m_event_monitor->stopMonitoring();
}

void CauvNode::run()
{
	cauv_global::print_module_header(m_name);
    
    m_event_monitor->startMonitoring();

    onRun();
    
    while(true)
    {
	    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }
}

void CauvNode::onRun()
{
}

void CauvNode::join(std::string const& group)
{
    if(m_mailbox)
        m_mailbox->joinGroup(group);
    else
        error() << "CauvNode::join: no mailbox";
}

void CauvNode::addObserver(boost::shared_ptr<MessageObserver> o)
{
    if(m_mailbox_monitor)
        m_mailbox_monitor->addObserver(o);
    else
        error() << "CauvNode::addObserver: no mailbox monitor";
}

int CauvNode::send(boost::shared_ptr<const Message> m, Spread::service st)
{
    if(m_mailbox)
        return m_mailbox->sendMessage(m, st);
    else
        error() << "CauvNode::join: no mailbox";
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

CauvNode::CauvNode(const string& name, const char* host)
    : m_name(name),
      m_mailbox(boost::make_shared<ReconnectingSpreadMailbox>(host, name)),
      m_event_monitor(boost::make_shared<MailboxEventMonitor>(m_mailbox)),
      m_mailbox_monitor(boost::make_shared<MsgSrcMBMonitor>())
{
    m_event_monitor->addObserver(m_mailbox_monitor);
    addObserver(boost::make_shared<DBGLevelObserver>());
    join("debug");
}

