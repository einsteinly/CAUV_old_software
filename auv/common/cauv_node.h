#ifndef __CAUV_NODE_H__
#define __CAUV_NODE_H__

#include <string>
#include <signal.h>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include <common/spread/cauv_spread_rc_mailbox.h>
#include <common/spread/cauv_mailbox_monitor.h>
#include <common/spread/cauv_msgsrc_mb_observer.h>

class CauvNode
{
	public:
		virtual ~CauvNode();
		
        void run();
		virtual void onRun();

	protected:
		std::string m_name;

        boost::shared_ptr<ReconnectingSpreadMailbox> mailbox() const;
        boost::shared_ptr<MailboxEventMonitor> eventMonitor() const; 
        boost::shared_ptr<MsgSrcMBMonitor> mailboxMonitor() const;

        CauvNode(const std::string& name, const char* host="16707@localhost");
    
    private:
        boost::shared_ptr<ReconnectingSpreadMailbox> m_mailbox;
        boost::shared_ptr<MailboxEventMonitor> m_event_monitor;
        boost::shared_ptr<MsgSrcMBMonitor> m_mailbox_monitor;
};

#endif//__CAUV_NODE_H__

