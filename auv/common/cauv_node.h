#ifndef __CAUV_NODE_H__
#define __CAUV_NODE_H__

#include <string>
#include <signal.h>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include <common/spread/spread_rc_mailbox.h>
#include <common/spread/mailbox_monitor.h>
#include <common/spread/msgsrc_mb_observer.h>

class CauvNode
{
	public:
		virtual ~CauvNode();
		
        void run();
		virtual void onRun();
    
        void join(std::string const& group);
        void addObserver(boost::shared_ptr<MessageObserver>);
        int send(boost::shared_ptr<const Message> message,
                 Spread::service serviceType = SAFE_MESS);


	protected:
		std::string m_name;
        boost::shared_ptr<ReconnectingSpreadMailbox> mailbox() const;

        CauvNode(const std::string& name, const char* host="16707@localhost");
    
    private:
        boost::shared_ptr<ReconnectingSpreadMailbox> m_mailbox;
        boost::shared_ptr<MailboxEventMonitor> m_event_monitor;
        boost::shared_ptr<MsgSrcMBMonitor> m_mailbox_monitor;
};

#endif//__CAUV_NODE_H__

