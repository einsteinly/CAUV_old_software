#include "mailbox_monitor.h"
#include "spread_rc_mailbox.h"

#include <debug/cauv_debug.h>

using namespace cauv;

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

void MailboxEventMonitor::startMonitoringAsync() {
    if(m_thread.get_id() == boost::thread::id()){
        m_thread = boost::thread( &MailboxEventMonitor::doMonitoring, this );

        struct sched_param param;
        param.sched_priority = -10;
        pthread_setschedparam( m_thread.native_handle(), SCHED_OTHER, &param);

    }else{
        error() << __func__ << ": already monitoring";
    }
}

void MailboxEventMonitor::stopMonitoringAsync() {
    if (m_thread.get_id() != boost::thread::id())
    {
        debug() << "Interrupting monitor thread";
        m_thread.interrupt();
        m_thread.join();
    }
}

void MailboxEventMonitor::startMonitoringSync() {
    doMonitoring();
}

void MailboxEventMonitor::doMonitoring() {
    try {
        debug() << "Started monitoring";
        while(true) {
            boost::shared_ptr<SpreadMessage> m( m_mailbox->receiveMessage() );

            boost::lock_guard<boost::recursive_mutex> l(m_observers_lock);

            if (m->getMessageFlavour() == SpreadMessage::REGULAR_MESSAGE)
            {
                foreach (observer_ptr_t o, m_observers)
                    o->regularMessageReceived(boost::dynamic_pointer_cast<RegularMessage, SpreadMessage>(m));
            }
            else if(m->getMessageFlavour() == SpreadMessage::MEMBERSHIP_MESSAGE)
            {
                foreach (observer_ptr_t o, m_observers)
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
    } 
    debug() << "Ending monitoring";
}
