#ifndef __CAUV_MSGSRC_MAILBOX_H__
#define __CAUV_MSGSRC_MAILBOX_H__

#include <generated/messages.h>

#include "spread_messages.h"
#include "mailbox_monitor.h"

namespace cauv{

class MsgSrcMBMonitor: public MessageSource, public MailboxObserver{
    public:
        virtual void regularMessageReceived(boost::shared_ptr<const RegularMessage> msg){
            notifyObservers(msg->getMessage());
        }
        
        /* for now we don't do anything with these */
        virtual void membershipMessageReceived(boost::shared_ptr<const MembershipMessage>) { }
};

} // namespace cauv

#endif // ndef __CAUV_MSGSRC_MAILBOX_H__
