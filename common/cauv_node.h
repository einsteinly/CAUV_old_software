#ifndef __CAUV_NODE_H__
#define __CAUV_NODE_H__

#include <string>
#include <csignal>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include <ssrc/spread.h>


namespace boost {
namespace program_options {
    class options_description;
    class positional_options_description;
    class variables_map;
};
};

namespace cauv{

/* Forward Declarations */
class ReconnectingSpreadMailbox;
class MailboxEventMonitor;
class MsgSrcMBMonitor;
class MessageObserver;
class Message;

class CauvNode
{
	public:
        virtual ~CauvNode();
        virtual void stopNode();
		 
        void run(bool synchronous=false);

        int defaultOptions();
        int parseOptions(int argc, char** arg);

        void joinGroup(std::string const& group);
        void addMessageObserver(boost::shared_ptr<MessageObserver>);
        void removeMessageObserver(boost::shared_ptr<MessageObserver>);
        void clearMessageObservers();
   
        int send(boost::shared_ptr<const Message> message,
                 Spread::service serviceType = SAFE_MESS);
        
	protected:
		std::string m_name;
		std::string m_server;
		unsigned int m_port;
        boost::shared_ptr<ReconnectingSpreadMailbox> mailbox() const;

        CauvNode(const std::string& name);
            
		virtual void onRun();
        virtual void addOptions(boost::program_options::options_description& desc,
                                boost::program_options::positional_options_description& pos);
        virtual int useOptionsMap(boost::program_options::variables_map& vm,
                                  boost::program_options::options_description& desc);


    private:
        boost::shared_ptr<ReconnectingSpreadMailbox> m_mailbox;
        boost::shared_ptr<MailboxEventMonitor> m_event_monitor;
        boost::shared_ptr<MsgSrcMBMonitor> m_mailbox_monitor;
        volatile bool m_interrupted;
};

} // namespace cauv

#endif // ndef __CAUV_NODE_H__

