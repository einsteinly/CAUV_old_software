#ifndef CAUV_ZMQ_MAILBOX_H
#define CAUV_ZMQ_MAILBOX_H

#include <common/mailbox.h>
#include <common/mailbox_monitor.h>
#include <generated/message_observers.h>
#include <utility/threadsafe-observable.h>


#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "zmq.hpp"

#include <string>
#include <map>
#include <vector>
#include <set>

namespace cauv {

class ZeroMQMailbox : public Mailbox, public MailboxEventMonitor, public MessageSource, boost::noncopyable {
    public:
    ZeroMQMailbox(const std::string name = "unknown");
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
    virtual void subMessage(const Message &message);
    virtual void unSubMessage(const Message &message);

    virtual void startMonitoringAsync();
    virtual void startMonitoringSync();
    virtual void stopMonitoring();
    virtual bool isMonitoring();

    virtual void addMessageObserver(boost::shared_ptr<MessageObserver>);
    virtual void removeMessageObserver(boost::shared_ptr<MessageObserver>);
    virtual void clearMessageObservers();

    private:
    //most internal variables are *not* threadsafe and should only be accessed
    //by the thread calling doMonitoring
    const std::string name;         //node name
    std::string sub_bind;           //zmq connection string for this node's sub socket
    zmq::context_t zm_context;      //main zm_context
    zmq::socket_t pub;              //the main message publication socket
    zmq::socket_t sub;              //the main message subscription socket
    zmq::socket_t send_queue_push;  //inproc queue for messages (passed by pointer)
    zmq::socket_t send_queue_pull;
    zmq::socket_t sub_queue_push;   //inproc queue for subscriptions
    zmq::socket_t sub_queue_pull;
    zmq::socket_t daemon_control;   //used to communicate with vehicle_daemon

    boost::mutex m_send_mutex;      //guards send_queue_push socket
    boost::mutex m_sub_mutex;       //guards send_sub_push socket

    //message ids that have been subscribed to by other nodes
    typedef std::set<uint32_t> pubs_t;
    pubs_t publications;

    //message ids this node has subscribed to
    typedef std::map<uint32_t,unsigned int> subs_count_t;
    subs_count_t subscriptions;

    typedef std::set<uint32_t> pids_t;
    typedef std::set<std::string> connections_t;

    //scan for unix domain sockets in dir and connect to them if the node that
    //manages them is still alive
    pids_t scan_ipc_dir(std::string dir);
    //pids which we need to send connection strings to
    pids_t send_connect_pids;
    //zmq connections strings that this node has connected to already
    connections_t connections;

    //pushed over sub_queue_push for subscriptions
    struct SubscriptionMessage {
        bool subscribe; //subscribe or unsubscribe
        uint32_t msg_id;
    };

    void send_connect_message(uint32_t pid);
    void handle_pub_message(void);
    void handle_sub_message(void);
    void handle_send_message(void);
    void handle_subscription_message(void);
    void handle_daemon_message(void);
    void doMonitoring();

    bool m_monitoring;
    bool m_interrupted;
    bool daemon_connected;
    boost::thread m_thread;
    int m_group_gen;

};

}

#endif //ndef CAUV_ZMQ_MAILBOX_H
