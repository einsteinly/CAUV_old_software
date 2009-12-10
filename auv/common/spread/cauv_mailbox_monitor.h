#ifndef CAUV_MAILBOX_MONITOR_H_INCLUDED
#define CAUV_MAILBOX_MONITOR_H_INCLUDED

#include <set>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include "cauv_spread_mailbox.h"
#include "cauv_spread_messages.h"

class MailboxObserver {
public:
    virtual void applicationMessageReceived(boost::shared_ptr<ApplicationMessage> message) = 0;
    virtual void membershipMessageReceived(boost::shared_ptr<MembershipMessaget> message) = 0;
}; 
typedef boost::shared_ptr<MailboxObserver> mb_observer_ptr_t;

class MailboxEventMonitor {
public:
    MailboxEventMonitor(SpreadMailbox &mailbox)
        : m_thread_callable(mailbox), m_thread(){
    }

    void addObserver(mb_observer_ptr_t observer){
        m_thread_callable.addObserver(observer);
    }

    void removeObserver(mb_observer_ptr_t observer){
        m_thread_callable.removeObserver(observer);
    }

    void clearObservers(){
        m_thread_callable.clearObservers();
    }

    /**
     * Spawns a new thread listening for new messages on the associated mailbox.
     * Whenever a new message is received, all observers are informed.
     */
    void startMonitoring(){
        if(!m_thread){
            m_thread = new boost::thread(boost::ref(m_thread_callable));
        }
    }

    /**
     * Tells the spawned event loop to die.
     * TODO: should this return immediately, or wait?
     */
    void stopMonitoring(){
        m_thread_callable.stop() ;
        m_thread->join();
    }
    
private:
    /**
     * Defines the event loop for the monitor thread, and a way to stop it
     */
    class MonitorThreadCallable{
        public:
            MonitorThreadCallable(SpreadMailbox &mailbox)
                : m_mailbox(mailbox), m_stop_lock(), m_stop(false),
                  m_observers_lock(), m_observers(){
            }

            void stop(){
                boost::lock_guard<boost::recursive_mutex> l(m_stop_lock);
                m_stop = true;
            }

            void addObserver(mb_observer_ptr_t observer){
                boost::lock_guard<boost::recursive_mutex> l(m_observers_lock); 
                m_observers.insert(observer);
            }

            void removeObserver(mb_observer_ptr_t observer){
                boost::lock_guard<boost::recursive_mutex> l(m_observers_lock); 
                m_observers.erase(observer);
            }

            void clearObservers(){
               boost::lock_guard<boost::recursive_mutex> l(m_observers_lock); 
               m_observers.clear();
            }

            void operator()(){
                for(;;){
                    boost::unique_lock<boost::recursive_mutex> l(m_stop_lock);
                    if(m_stop){
                        m_stop = false;
                        return;
                    }
                    l.unlock();
                    
                    boost::shared_ptr<SpreadMessage> m(m_mailbox.receiveMessage());

                    m_observers_lock.lock();
                    BOOST_FOREACH(mb_observer_ptr_t p, m_observers){
                        if(m->getMessageType() == SpreadMessage::REGULAR_MESSAGE){
                            p->applicationMessageReceived(m);
                        }else{
                            assert(m->getMessageType() == SpreadMessage::MEMBERSHIP_MESSAGE);
                            p->membershipMessageReceived(m);
                        }
                    }
                    m_observers_lock.unlock();
                }
            }

        private:
            SpreadMailbox& m_mailbox;

            boost::recursive_mutex m_stop_lock;
            bool m_stop;

            boost::recursive_mutex m_observers_lock;
            std::set<mb_observer_ptr_t> m_observers;
    };

    boost::thread *m_thread;
    MonitorThreadCallable m_thread_callable;
};
#endif // CAUV_MAILBOX_MONITOR_H_INCLUDED

