#ifndef CAUV_MAILBOX_MONITOR_H_INCLUDED
#define CAUV_MAILBOX_MONITOR_H_INCLUDED

#include "cauv_spread_mailbox.h"
#include "cauv_spread_messages.h"

class MailboxObserver {
public:
    virtual void applicationMessageReceived(const ApplicationMessage &message) {}
    virtual void membershipMessageReceived(const MembershipMessage &message) {}
};

class MailboxEventMonitor {
public:
    MailboxEventMonitor(SpreadMailbox &mailbox);
    void addObserver(MailboxObserver observer);
    void removeObserver(MailboxObserver observer);
    void clearObservers();

    /**
     * Spawns a new thread listening for new messages on the associated mailbox.
     * Whenever a new message is received, all observers are informed.
     */
    void startMonitoring();

    /**
     * Tells the spawned event loop to die.
     */
    void stopMonitoring();
};
#endif // CAUV_MAILBOX_MONITOR_H_INCLUDED
