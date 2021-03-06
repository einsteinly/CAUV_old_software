/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_NODE_H__
#define __CAUV_NODE_H__

#include <string>
#include <csignal>

#include <boost/shared_ptr.hpp>
#include "mailbox.h"


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
class ZeroMQMailbox;
class MailboxEventMonitor;
class MessageObserver;
class Message;
class SubscribeObserver;

class CauvNode
{
    public:
        CauvNode(const std::string& name);
        virtual ~CauvNode();
        virtual void stopNode();

        /* syncronous refers to the processing of messages. If true, onRun
         * should not block or messages will not be delivered.
         * Else, onRun can block. In either case run() will not return until
         * the node has been stopped
         */
        void run(bool synchronous=true);
        bool isRunning();

        int defaultOptions();
        int parseOptions(int argc, char** arg);

        void joinGroup(const std::string& group);
        void subMessage(const Message &message);
        void unSubMessage(const Message &message);

        void addMessageObserver(boost::shared_ptr<MessageObserver>);
        void removeMessageObserver(boost::shared_ptr<MessageObserver>);
        void clearMessageObservers();

        void addSubscribeObserver(boost::shared_ptr<SubscribeObserver>);
        void removeSubscribeObserver(boost::shared_ptr<SubscribeObserver>);
        void clearSubscribeObservers();

        int send(boost::shared_ptr<const Message> message,
                 MessageReliability reliability = RELIABLE_MSG);

    protected:
        std::string m_name;
        std::string m_server;
        unsigned int m_port;
        boost::shared_ptr<Mailbox> mailbox() const;

        virtual void onRun();
        virtual void addOptions(boost::program_options::options_description& desc,
                                boost::program_options::positional_options_description& pos);
        virtual int useOptionsMap(boost::program_options::variables_map& vm,
                                  boost::program_options::options_description& desc);


    private:
        boost::shared_ptr<ReconnectingSpreadMailbox> m_spread_mailbox;
        boost::shared_ptr<ZeroMQMailbox> m_zeromq_mailbox;
        boost::shared_ptr<Mailbox> m_mailbox;
        boost::shared_ptr<MailboxEventMonitor> m_event_monitor;
        volatile bool m_running;
};

} // namespace cauv

#endif // ndef __CAUV_NODE_H__

