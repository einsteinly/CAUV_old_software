#ifndef CAUV_ZMQ_MAILBOX_H
#define CAUV_ZMQ_MAILBOX_H

#include <string>
#include <map>
#include <vector>
#include <set>

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/uuid/uuid.hpp>

#include <xs.hpp>

#include <common/mailbox.h>
#include <common/mailbox_monitor.h>
#include <generated/types/message_type.h>
#include <generated/message_observers.h>
#include <utility/threadsafe-observable.h>

namespace cauv {

class ZeroMQMailbox : public Mailbox, public MailboxEventMonitor, public MessageSource, public Observable<SubscribeObserver>,  boost::noncopyable {
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

    virtual void addSubscribeObserver(boost::shared_ptr<SubscribeObserver>);
    virtual void removeSubscribeObserver(boost::shared_ptr<SubscribeObserver>);
    virtual void clearSubscribeObservers();

    private:
    //most internal variables are *not* threadsafe and should only be accessed
    //by the thread calling doMonitoring
    const std::string name;         //node name
    std::string sub_bind;           //xs connection string for this node's sub socket
    xs::context_t zm_context;       //main zm_context
    xs::socket_t pub;               //the main message publication socket
    xs::socket_t sub;               //the main message subscription socket
    xs::socket_t send_queue_push;   //inproc queue for messages (passed by pointer)
    xs::socket_t send_queue_pull;
    xs::socket_t sub_queue_push;    //inproc queue for subscriptions
    xs::socket_t sub_queue_pull;
    xs::socket_t daemon_control;    //used to communicate with vehicle_daemon

    boost::mutex m_send_mutex;      //guards send_queue_push socket
    boost::mutex m_sub_mutex;       //guards send_sub_push socket
    boost::mutex m_pub_map_mutex;   //guards publications map

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
    //xs connections strings that this node has connected to already
    connections_t connections;

    void send_connect_message(uint32_t pid);
    void send_subscribed_message (const boost::uuids::uuid& node_uuid, uint32_t type);
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
    boost::uuids::uuid id;

    bool starting_up;
};

}

#endif //ndef CAUV_ZMQ_MAILBOX_H
