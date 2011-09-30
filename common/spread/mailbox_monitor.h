#ifndef __CAUV_MAILBOX_MONITOR_H__
#define __CAUV_MAILBOX_MONITOR_H__

#include <list>
#include <iostream>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <utility/threadsafe-observable.h>

namespace cauv{

class ReconnectingSpreadMailbox;
class SpreadMessage;
class RegularMessage;
class MembershipMessage;

class MailboxObserver {
    public:
        virtual void regularMessageReceived(boost::shared_ptr<const RegularMessage> message) = 0;
        virtual void membershipMessageReceived(boost::shared_ptr<const MembershipMessage> message) = 0;
};
typedef boost::shared_ptr<MailboxObserver> mb_observer_ptr_t;

class TestMBObserver: public MailboxObserver{
    public:
        TestMBObserver();
        void regularMessageReceived(boost::shared_ptr<const RegularMessage>);
        void membershipMessageReceived(boost::shared_ptr<const MembershipMessage>);
};

class MailboxEventMonitor : public ThreadSafeObservable<MailboxObserver>, public boost::noncopyable {
    public:
        MailboxEventMonitor(boost::shared_ptr<ReconnectingSpreadMailbox> mailbox);

        /**
         * Spawns a new thread listening for new messages on the associated mailbox.
         * Whenever a new message is received, all observers are informed.
         */
        void startMonitoringAsync();

        void startMonitoringSync();

        void stopMonitoring();

        bool isMonitoring();

    private:
        void doMonitoring();

        boost::thread m_thread;
        boost::shared_ptr<ReconnectingSpreadMailbox> m_mailbox;
        volatile bool m_interupted;
        volatile bool m_monitoring;
        boost::thread::id m_sync_thread_id;
};

} // namespace cauv

#endif//__CAUV_MAILBOX_MONITOR_H__

