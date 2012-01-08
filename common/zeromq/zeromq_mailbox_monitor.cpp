#include "zeromq_mailbox_monitor.h"
#include "zeromq_mailbox.h"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>
#include <zmq.hpp>
#include <vector>

#include <debug/cauv_debug.h>

namespace cauv {

ZeroMQMailboxEventMonitor::ZeroMQMailboxEventMonitor(boost::shared_ptr<ZeroMQMailbox> mailbox) :
    m_monitoring(false), m_mailbox(mailbox), 
    m_mailbox_lock(mailbox->m_event_monitor_mutex, boost::try_to_lock), m_group_gen(0) {
    if (!m_mailbox_lock.owns_lock()) {
        throw MultipleMonitorsForMailboxError();
    }

}

void ZeroMQMailboxEventMonitor::startMonitoringAsync(void) {
    m_thread = boost::thread(&ZeroMQMailboxEventMonitor::doMonitoring,this);
}

void ZeroMQMailboxEventMonitor::startMonitoringSync(void) {
    doMonitoring();
}

void ZeroMQMailboxEventMonitor::stopMonitoring(void) {
    m_interrupted = true;
    if (m_thread.get_id() != boost::thread::id()) {
        m_thread.join();
    }
}

bool ZeroMQMailboxEventMonitor::isMonitoring(void) {
    return m_monitoring;
}

void ZeroMQMailboxEventMonitor::addMessageObserver(boost::shared_ptr<MessageObserver> observer) {
    addObserver(observer);
}

void ZeroMQMailboxEventMonitor::removeMessageObserver(boost::shared_ptr<MessageObserver> observer) {
    removeObserver(observer);
}

void ZeroMQMailboxEventMonitor::clearMessageObservers(void) {
    clearObservers();
}

void ZeroMQMailboxEventMonitor::doMonitoring(void) {
    info() << "started monitoring";
    m_monitoring = true;
    m_interrupted = false;
    zmq::message_t msg;
    std::vector <boost::shared_ptr <ZeroMQGroup> > zm_groups;
    std::vector <zmq::pollitem_t> zm_poll_items;

    while(!m_interrupted) {
        if (m_mailbox->m_group_gen != m_group_gen) {
            zm_poll_items.clear();
            zm_groups.clear();
            zm_groups = m_mailbox->get_groups();
            BOOST_FOREACH(boost::shared_ptr <ZeroMQGroup> group,zm_groups) {
                zm_poll_items.push_back(group->poll_item);
            }
        }
        if (zmq::poll(&zm_poll_items[0],zm_poll_items.size(),1000000)) {
            for (unsigned int i = 0; i < zm_poll_items.size(); i++) {
                if (zm_poll_items[i].revents == ZMQ_POLLIN) {
                    debug() << "received message from group" << zm_groups[i]->name;
                    zm_groups[i]->sub_skt.recv(&msg,ZMQ_NOBLOCK);
                    //!!! seems like the copy involved here could be avoided
                    //somehow...
                    notifyObservers(boost::make_shared<const svec_t>((byte*)msg.data(),(byte*)msg.data() + msg.size()));
                }
            }
        }
    }
}

}
