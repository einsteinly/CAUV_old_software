#include "cauv_mailbox_monitor.h"


TestMBObserver::TestMBObserver()
    : MailboxObserver(){
}

void TestMBObserver::regularMessageReceived(boost::shared_ptr<const RegularMessage>) {
    info() << "TestMBObserver: regular message received";
}

void TestMBObserver::membershipMessageReceived(boost::shared_ptr<const MembershipMessage>){
    info() << "TestMBObserver: membership message received";
}



MailboxEventMonitor::MailboxEventMonitor(boost::shared_ptr<ReconnectingSpreadMailbox> mailbox)
        : m_thread(), m_mailbox(mailbox) {
}

void MailboxEventMonitor::addObserver(mb_observer_ptr_t observer) {
    boost::lock_guard<boost::recursive_mutex> l(m_observers_lock);
    m_observers.push_back(observer);
}

void MailboxEventMonitor::removeObserver(mb_observer_ptr_t observer) {
    boost::lock_guard<boost::recursive_mutex> l(m_observers_lock);
    m_observers.remove(observer);
}

void MailboxEventMonitor::clearObservers() {
    boost::lock_guard<boost::recursive_mutex> l(m_observers_lock);
    m_observers.clear();
}

void MailboxEventMonitor::startMonitoring() {
    if(m_thread.get_id() == boost::thread::id()){
        m_thread = boost::thread( &MailboxEventMonitor::doMonitoring, this );

        struct sched_param param;
        param.sched_priority = -10;
        pthread_setschedparam( m_thread.native_handle(), SCHED_OTHER, &param);

    }else{
        error() << __func__ << ": already monitoring";
    }
}

void MailboxEventMonitor::stopMonitoring() {
    if (m_thread.get_id() != boost::thread::id())
    {
        debug() << "Interrupting monitor thread";
        m_thread.interrupt();
        m_thread.join();
    }
}

void MailboxEventMonitor::doMonitoring() {
    try {
        debug() << "Started monitor thread";
        while(true) {
            boost::shared_ptr<SpreadMessage> m( m_mailbox->receiveMessage() );

            boost::lock_guard<boost::recursive_mutex> l(m_observers_lock);

            if (m->getMessageFlavour() == SpreadMessage::REGULAR_MESSAGE)
            {
                foreach (mb_observer_ptr_t o, m_observers)
                    o->regularMessageReceived(boost::dynamic_pointer_cast<RegularMessage, SpreadMessage>(m));
            }
            else if(m->getMessageFlavour() == SpreadMessage::MEMBERSHIP_MESSAGE)
            {
                foreach (mb_observer_ptr_t o, m_observers)
                    o->membershipMessageReceived(boost::dynamic_pointer_cast<MembershipMessage, SpreadMessage>(m));
            }
            else
            {
                error() << __func__ << "dropping unrecognised message type:"
                    << m->getMessageFlavour();
            }
        }
    } catch (boost::thread_interrupted& e) {
        debug() << "Monitor thread interrupted";
    } catch (...) {
        debug() << "WTF";
    }
    debug() << "Ending monitor thread";
}

