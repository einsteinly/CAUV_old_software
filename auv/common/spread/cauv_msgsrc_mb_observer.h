#ifndef __CAUV_MSGSRC_MAILBOX_H__
#define __CAUV_MSGSRC_MAILBOX_H__

#include <common/messages.h>
#include "cauv_mailbox_monitor.h"

class MsgSrcMBMonitor: public MessageSource, public MailboxObserver{
    public:
        virtual void regularMessageReceived(boost::shared_ptr<RegularMessage> msg){
            notifyObservers(msg->getMessage());
        }
        
        /* for now we don't do anything with these */
        virtual void membershipMessageReceived() { }
};


#endif // ndef __CAUV_MSGSRC_MAILBOX_H__
