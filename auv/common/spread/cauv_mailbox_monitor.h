#ifndef CAUV_MAILBOX_MONITOR_H_INCLUDED
#define CAUV_MAILBOX_MONITOR_H_INCLUDED

#include <set>
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
    TestMBObserver()
        : MailboxObserver(){
    }

    void regularMessageReceived(boost::shared_ptr<const RegularMessage>) {
        info() << "TestMBObserver: regular message received";
    }
    
    void membershipMessageReceived(boost::shared_ptr<const MembershipMessage>){
        info() << "TestMBObserver: membership message received";
    }
};

class MailboxEventMonitor: boost::noncopyable {
public:
    MailboxEventMonitor(boost::shared_ptr<ReconnectingSpreadMailbox> mailbox)
            : m_thread(), m_thread_callable(mailbox) {
    }

    void addObserver(mb_observer_ptr_t observer) {
        m_thread_callable.addObserver(observer);
    }

    void removeObserver(mb_observer_ptr_t observer) {
        m_thread_callable.removeObserver(observer);
    }

    void clearObservers() {
        m_thread_callable.clearObservers();
    }

    /**
     * Spawns a new thread listening for new messages on the associated mailbox.
     * Whenever a new message is received, all observers are informed.
     */
    void startMonitoring() {
        if(!m_thread){
            m_thread = boost::make_shared<boost::thread>( boost::ref(m_thread_callable) );

            struct sched_param param;
            param.sched_priority = -10;
            pthread_setschedparam( m_thread->native_handle(), SCHED_OTHER, &param);

        }else{
            error() << __func__ << ": already monitoring";
        }
    }

    /**
     * Tells the spawned event loop to die.
     * TODO: should this return immediately, or wait?
     */
    void stopMonitoring() {
        if(m_thread){
            m_thread_callable.stop();
            m_thread->join();
            m_thread.reset();
        }
    }

private:
    /**
     * Defines the event loop for the monitor thread, and a way to stop it
     */
    class MonitorThreadCallable {
    public:
        MonitorThreadCallable(boost::shared_ptr<ReconnectingSpreadMailbox> mailbox)
                : m_mailbox(mailbox), m_stop_lock(), m_stop(false),
                m_observers_lock(), m_observers() {
        }

        void stop() {
            boost::lock_guard<boost::recursive_mutex> l(m_stop_lock);
            m_stop = true;
        }

        void addObserver(mb_observer_ptr_t observer) {
            boost::lock_guard<boost::recursive_mutex> l(m_observers_lock);
            m_observers.insert(observer);
        }

        void removeObserver(mb_observer_ptr_t observer) {
            boost::lock_guard<boost::recursive_mutex> l(m_observers_lock);
            m_observers.erase(observer);
        }

        void clearObservers() {
            boost::lock_guard<boost::recursive_mutex> l(m_observers_lock);
            m_observers.clear();
        }

        void operator()() {
            while(true) {
                boost::unique_lock<boost::recursive_mutex> l(m_stop_lock);
                if (m_stop) {
                    m_stop = false;
                    return;
                }
                l.unlock();

                boost::shared_ptr<SpreadMessage> m( m_mailbox->receiveMessage(200) );
                if(m){ 
                    m_observers_lock.lock();
                    std::set<mb_observer_ptr_t>::iterator i;
                    if (m->getMessageFlavour() == SpreadMessage::REGULAR_MESSAGE) {
                        for(i = m_observers.begin(); i != m_observers.end(); i++)
                            (*i)->regularMessageReceived(boost::dynamic_pointer_cast<RegularMessage, SpreadMessage>(m));
                    } else if(m->getMessageFlavour() == SpreadMessage::MEMBERSHIP_MESSAGE){
                        for(i = m_observers.begin(); i != m_observers.end(); i++) 
                            (*i)->membershipMessageReceived(boost::dynamic_pointer_cast<MembershipMessage, SpreadMessage>(m));
                    } else {
                        error() << __func__ << "dropping unrecognised message type:"
                                << m->getMessageFlavour();
                    }
                    m_observers_lock.unlock();
                }
            }
        }

    private:
        boost::shared_ptr<ReconnectingSpreadMailbox> m_mailbox;

        boost::recursive_mutex m_stop_lock;
        bool m_stop;

        boost::recursive_mutex m_observers_lock;
        std::set<mb_observer_ptr_t> m_observers;
    };

    boost::shared_ptr<boost::thread> m_thread;
    MonitorThreadCallable m_thread_callable;
};
#endif // CAUV_MAILBOX_MONITOR_H_INCLUDED

