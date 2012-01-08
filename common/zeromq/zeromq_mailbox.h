#ifndef CAUV_ZMQ_MAILBOX_H
#define CAUV_ZMQ_MAILBOX_H

#include <common/mailbox.h>

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/thread/mutex.hpp>
#include <zmq.hpp>

#include <string>
#include <map>
#include <vector>

namespace cauv {

class ZeroMQGroup : boost::noncopyable {
    public:
    ZeroMQGroup(zmq::context_t &context, const std::string name);
    zmq::pollitem_t poll_item;
    zmq::socket_t push_skt;
    zmq::socket_t sub_skt;
    const std::string name;
};

class ZeroMQMailboxEventMonitor;

class ZeroMQMailbox : public Mailbox, boost::noncopyable {
    public:
    ZeroMQMailbox(void);
    /**
     * @return The number of bytes sent
     */
    virtual int sendMessage(boost::shared_ptr<const Message> message, MessageReliability);

    /**
     * @return The number of bytes sent
     */
    virtual int sendMessage(boost::shared_ptr<const Message> message, MessageReliability,
                    const std::string &destinationGroup);

    virtual void joinGroup(const std::string &groupName);
    virtual void leaveGroup(const std::string &groupName);

    friend class ZeroMQMailboxEventMonitor;

    private:
    int send_message_to_group(boost::shared_ptr<const Message>, boost::shared_ptr<ZeroMQGroup> group);
    std::vector <boost::shared_ptr<ZeroMQGroup> > get_groups(void);

    zmq::context_t zm_context;

    typedef std::map <std::string, boost::shared_ptr<ZeroMQGroup> > zm_group_map;
    zm_group_map zm_groups;
    int m_group_gen;
    boost::mutex m_event_monitor_mutex;
};

}

#endif //ndef CAUV_ZMQ_MAILBOX_H
