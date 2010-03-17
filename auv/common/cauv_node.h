#ifndef __CAUV_NODE_H__
#define __CAUV_NODE_H__

#include <string>
#include <signal.h>
#include <iostream>

#include <boost/shared_ptr.hpp>

class ReconnectingSpreadMailbox;
class MailboxEventMonitor;
class MsgSrcMBMonitor;

class CauvNode
{
	public:
		virtual ~CauvNode();
		
        void run();

	protected:
		std::string m_name;

		virtual void onRun();

        boost::shared_ptr<ReconnectingSpreadMailbox> mailbox() const;
        boost::shared_ptr<MailboxEventMonitor> eventMonitor() const; 
        boost::shared_ptr<MsgSrcMBMonitor> mailboxMonitor() const;

        CauvNode(const std::string& name);
    
    private:
        boost::shared_ptr<ReconnectingSpreadMailbox> m_mailbox;
        boost::shared_ptr<MailboxEventMonitor> m_event_monitor;
        boost::shared_ptr<MsgSrcMBMonitor> m_mailbox_monitor;
};

#endif//__CAUV_NODE_H__

