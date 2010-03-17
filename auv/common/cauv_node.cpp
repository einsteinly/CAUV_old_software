#include <iostream>
#include <limits>
#include <string>

#include <common/cauv_global.h>

#include <common/spread/cauv_spread_rc_mailbox.h>
#include <common/spread/cauv_mailbox_monitor.h>
#include <common/spread/cauv_msgsrc_mb_observer.h>

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

CauvNode::CauvNode(const string& name)
    : m_name(name),
      m_mailbox(new ReconnectingSpreadMailbox("16707@localhost", name)),
      m_event_monitor(new MailboxEventMonitor(m_mailbox)),
      m_mailbox_monitor(new MsgSrcMBMonitor)
{
    m_event_monitor->addObserver(m_mailbox_monitor);
}

CauvNode::~CauvNode()
{
	cout << "Shutting down node" << endl;
    m_event_monitor->stopMonitoring();
}

void CauvNode::run()
{
	cauv_global::print_module_header(m_name);
    
    m_event_monitor->startMonitoring();

	onRun();

    while(true)
    {
        msleep(500);
    }
}

