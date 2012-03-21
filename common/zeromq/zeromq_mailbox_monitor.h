#ifndef CAUV_ZMQ_MAILBOX_MONITOR
#define CAUV_ZMQ_MAILBOX_MONITOR

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>

#include <exception>

#include <common/mailbox_monitor.h>
#include <generated/message_observers.h>
#include <utility/threadsafe-observable.h>

namespace cauv {

class ZeroMQMailbox;

class MultipleMonitorsForMailboxError : public std::runtime_error {
    public:
    MultipleMonitorsForMailboxError(void) :
        std::runtime_error("tried to create more than one ZeroMQMailboxEventMonitor for one ZeroMQMailbox") {};
};

class ZeroMQMailboxEventMonitor : public MailboxEventMonitor, public MessageSource, boost::noncopyable {
    public:
    ZeroMQMailboxEventMonitor(boost::shared_ptr<ZeroMQMailbox> mailbox);

    /**
     * Spawns a new thread listening for new messages on the associated mailbox.
     * Whenever a new message is received, all observers are informed.
     */
    virtual void startMonitoringAsync();
    virtual void startMonitoringSync();
    virtual void stopMonitoring();
    virtual bool isMonitoring();

    virtual void addMessageObserver(boost::shared_ptr<MessageObserver>);
    virtual void removeMessageObserver(boost::shared_ptr<MessageObserver>);
    virtual void clearMessageObservers();

    private:
    void doMonitoring();

    bool m_monitoring;
    bool m_interrupted;
    boost::shared_ptr<ZeroMQMailbox> m_mailbox;
    boost::unique_lock<boost::mutex> m_mailbox_lock;
    boost::thread m_thread;
    int m_group_gen;
};

}

#endif //ndef CAUV_ZMQ_MAILBOX_MONITOR
