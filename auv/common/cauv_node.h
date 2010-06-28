#ifndef __CAUV_NODE_H__
#define __CAUV_NODE_H__

#include <string>
#include <signal.h>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include <common/spread/spread_rc_mailbox.h>
#include <common/spread/mailbox_monitor.h>
#include <common/spread/msgsrc_mb_observer.h>

namespace boost {
namespace program_options {
    class options_description;
    class variables_map;
};
};

class CauvNode
{
	public:
		virtual ~CauvNode();
		
        void run();

        int parseOptions(int argc, char** arg);

        void joinGroup(std::string const& group);
        void addMessageObserver(boost::shared_ptr<MessageObserver>);
        void removeMessageObserver(boost::shared_ptr<MessageObserver>);
        void clearMessageObservers();
   
        int send(boost::shared_ptr<const Message> message,
                 Spread::service serviceType = SAFE_MESS);


	protected:
		std::string m_name;
        boost::shared_ptr<ReconnectingSpreadMailbox> mailbox() const;

        CauvNode(const std::string& name);
        
		virtual void onRun();
        virtual void addOptions(boost::program_options::options_description& desc);
        virtual int useOptionsMap(boost::program_options::variables_map& vm, boost::program_options::options_description& desc);
    
    private:
        boost::shared_ptr<ReconnectingSpreadMailbox> m_mailbox;
        boost::shared_ptr<MailboxEventMonitor> m_event_monitor;
        boost::shared_ptr<MsgSrcMBMonitor> m_mailbox_monitor;
};

#endif//__CAUV_NODE_H__

