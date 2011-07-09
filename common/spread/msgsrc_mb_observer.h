#ifndef __CAUV_MSGSRC_MAILBOX_H__
#define __CAUV_MSGSRC_MAILBOX_H__

#include <boost/make_shared.hpp>

#include <common/cauv_utils.h>
#include <generated/message_observers.h>

#include "spread_messages.h"
#include "mailbox_monitor.h"

namespace cauv{

class MsgSrcMBMonitor: public MessageSource, public MailboxObserver{
    public:
        virtual void regularMessageReceived(boost::shared_ptr<const RegularMessage> msg){
            notifyObservers(msg->getMessage());
        }
        
        /* HACK - Hardcoding a generated message use in messsage code
         *        This membership message handling could/should be built into the message generation scripts,
         *        but if we're going to replace Spread soon I'm not going to spend a lot of time on it now.
         *        Group memberships are a feature of spread and might not be a feature of a new messaging system
         *
         * For now we only notify there has been a memebership message and what group it relates to
         * @TODO: support for all memebership message types
         */
        virtual void membershipMessageReceived(boost::shared_ptr<const MembershipMessage> msg) {
            foreach(observer_ptr_t o, m_observers)
                o->onMembershipChangedMessage(boost::make_shared<MembershipChangedMessage>(msg->getAffectedGroupName()));
        }
};

} // namespace cauv

#endif // ndef __CAUV_MSGSRC_MAILBOX_H__
