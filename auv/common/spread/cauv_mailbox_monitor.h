#ifndef __CAUV_MAILBOX_MONITOR_H__
#define __CAUV_MAILBOX_MONITOR_H__

#include <list>
#include <iostream>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>

#include <common/debug.h>

#include "cauv_spread_rc_mailbox.h"
#include "cauv_spread_messages.h"

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

class MailboxEventMonitor: boost::noncopyable {
    public:
        MailboxEventMonitor(boost::shared_ptr<ReconnectingSpreadMailbox> mailbox);

        void addObserver(mb_observer_ptr_t observer); 
        void removeObserver(mb_observer_ptr_t observer); 
        void clearObservers();

        /**
         * Spawns a new thread listening for new messages on the associated mailbox.
         * Whenever a new message is received, all observers are informed.
         */
        void startMonitoring(); 

        /**
         * Tells the spawned event loop to die.
         * TODO: should this return immediately, or wait?
         */
        void stopMonitoring();

    private:
        void doMonitoring();

        boost::recursive_mutex m_observers_lock;
        std::list<mb_observer_ptr_t> m_observers;

        boost::thread m_thread;
        boost::shared_ptr<ReconnectingSpreadMailbox> m_mailbox;
};
#endif//__CAUV_MAILBOX_MONITOR_H__

