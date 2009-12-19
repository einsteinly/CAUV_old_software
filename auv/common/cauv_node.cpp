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

void CauvNode::onConnect()
{
    // TODO: eh? when does this get called, and by what?
}

void CauvNode::onDisconnect()
{
    // TODO: eh? when does this get called, and by what?  
}

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

CauvNode::CauvNode(const string& name, const string& group)
    : m_name(name), m_group(group),
      m_mailbox(new ReconnectingSpreadMailbox("16707@localhost", name)),
      m_event_monitor(new MailboxEventMonitor(m_mailbox)),
      m_mailbox_monitor(new MsgSrcMBMonitor){
    m_event_monitor->addObserver(m_mailbox_monitor);
    // TODO: do we want to interpret 'group' as a spread group, and join it?
    // m_mailbox->joinGroup(group);
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

    for(;;){
        msleep(500);
    }
}

