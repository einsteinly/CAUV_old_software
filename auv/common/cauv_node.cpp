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

void CauvNode::onRun()
{
}

boost::shared_ptr<ReconnectingSpreadMailbox> CauvNode::mailbox() const{
    return m_mailbox;
}

boost::shared_ptr<MailboxEventMonitor> CauvNode::eventMonitor() const{
    return m_event_monitor;
}

boost::shared_ptr<MsgSrcMBMonitor> CauvNode::mailboxMonitor() const{
    return m_mailbox_monitor;
}

CauvNode::CauvNode(const string& name, const char* host)
    : m_name(name),
      m_mailbox(new ReconnectingSpreadMailbox(host, name)),
      m_event_monitor(new MailboxEventMonitor(m_mailbox)),
      m_mailbox_monitor(new MsgSrcMBMonitor)
{
    m_event_monitor->addObserver(m_mailbox_monitor);
}

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

