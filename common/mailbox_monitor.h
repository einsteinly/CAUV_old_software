#ifndef CAUV_MAILBOX_MONITOR_H
#define CAUV_MAILBOX_MONITOR_H
#include <boost/shared_ptr.hpp>

namespace cauv {

class MessageObserver

class SubscribeObserver {
    public:
        virtual void onSubscribed(MessageType::e messageType) = 0;
};

class MailboxEventMonitor {
    public:

    /**
     * Spawns a new thread listening for new messages on the associated mailbox.
     * Whenever a new message is received, all observers are informed.
     */
    virtual void startMonitoringAsync() = 0;
    virtual void startMonitoringSync() = 0;
    virtual void stopMonitoring() = 0;
    virtual bool isMonitoring() = 0;

    virtual void addMessageObserver(boost::shared_ptr<MessageObserver>) = 0;
    virtual void removeMessageObserver(boost::shared_ptr<MessageObserver>) = 0;
    virtual void clearMessageObservers() = 0;

    virtual void addSubscribeObserver(boost::shared_ptr<SubscribeObserver>) = 0;
    virtual void removeSubscribeObserver(boost::shared_ptr<SubscribeObserver>) = 0;
    virtual void clearSubscribeObservers() = 0;
};

} //namespace cauv

#endif //ndef CAUV_MAILBOX_MONITOR
