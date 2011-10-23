/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "mailbox_monitor.h"
#include "spread_rc_mailbox.h"

#include <boost/thread/thread.hpp>
#include <common/cauv_utils.h>

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
        : m_thread(), m_mailbox(mailbox), m_interupted(false),
          m_monitoring(false), m_sync_thread_id(){
}

void MailboxEventMonitor::startMonitoringAsync() {
    m_monitoring = true;

    if(m_thread.get_id() == boost::thread::id()){
        m_thread = boost::thread( &MailboxEventMonitor::doMonitoring, this );
        /* TODO: we can't nice down without sudoing, so nice up other things
         * instead
        struct sched_param param;
        param.sched_priority = -10;
        pthread_setschedparam( m_thread.native_handle(), SCHED_OTHER, &param);
        */
    }else{
        error() << __func__ << ": already monitoring";
    }
}

void MailboxEventMonitor::stopMonitoring() {
    // async
    if (m_thread.get_id() != boost::thread::id())
    {
        int c = 0;
        do{
            debug() << "Interrupting monitor thread";
            m_thread.interrupt();
            m_thread.timed_join(boost::posix_time::seconds(1));
            m_interupted = true;
        }while(m_thread.joinable() && c++ < 30);
    } else {
        // sync
        debug() << "No internal monitor thread to interrupt"
                << "(m_thread=" << m_thread.native_handle() << ")"
                << "External thread:" << *(int*)&m_sync_thread_id << "instead";
        m_interupted = true;
    }
}

void MailboxEventMonitor::startMonitoringSync() {
    m_sync_thread_id = boost::this_thread::get_id();
    doMonitoring();
}

void MailboxEventMonitor::doMonitoring() {
    try {
        debug() << "Started monitoring";

        m_monitoring = true;

        while(!m_interupted) {
            boost::this_thread::interruption_point();
            boost::shared_ptr<SpreadMessage> m( m_mailbox->receiveMessage() );

            boost::lock_guard<boost::recursive_mutex> l(m_observers_lock);

            if (m->getMessageFlavour() == SpreadMessage::REGULAR_MESSAGE)
            {
                debug(13) << "forwarding regular message";
                foreach (observer_ptr_t o, m_observers)
                    o->regularMessageReceived(boost::dynamic_pointer_cast<RegularMessage, SpreadMessage>(m));
            }
            else if(m->getMessageFlavour() == SpreadMessage::MEMBERSHIP_MESSAGE)
            {
                debug(13) << "forwarding membership message";
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

    m_monitoring = false;
}

bool MailboxEventMonitor::isMonitoring(){
    return m_monitoring;
}
