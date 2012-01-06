/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __CAUV_NODE_H__
#define __CAUV_NODE_H__

#include <string>
#include <csignal>
#include <iostream>

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
class Mailbox;
class ReconnectingSpreadMailbox;
class MailboxEventMonitor;
class MessageObserver;
class Message;

class CauvNode
{
    public:
        virtual ~CauvNode();
        virtual void stopNode();

        void run(bool synchronous=false);
        bool isRunning();

        int defaultOptions();
        int parseOptions(int argc, char** arg);

        void joinGroup(std::string const& group);
        void addMessageObserver(boost::shared_ptr<MessageObserver>);
        void removeMessageObserver(boost::shared_ptr<MessageObserver>);
        void clearMessageObservers();

        int send(boost::shared_ptr<const Message> message,
                 MessageReliability reliability = RELIABLE_MSG);

    protected:
        std::string m_name;
        std::string m_server;
        unsigned int m_port;
        boost::shared_ptr<Mailbox> mailbox() const;

        CauvNode(const std::string& name);

        virtual void onRun();
        virtual void addOptions(boost::program_options::options_description& desc,
                                boost::program_options::positional_options_description& pos);
        virtual int useOptionsMap(boost::program_options::variables_map& vm,
                                  boost::program_options::options_description& desc);


    private:
        boost::shared_ptr<ReconnectingSpreadMailbox> m_spread_mailbox;
        boost::shared_ptr<Mailbox> m_mailbox;
        boost::shared_ptr<MailboxEventMonitor> m_event_monitor;
        volatile bool m_running;
};

} // namespace cauv

#endif // ndef __CAUV_NODE_H__

