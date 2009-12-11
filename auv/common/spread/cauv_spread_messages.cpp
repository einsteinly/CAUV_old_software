#include "cauv_spread_messages.h"

SpreadMessage::MessageFlavour RegularMessage::getMessageFlavour() const {
    return REGULAR_MESSAGE;
}

SpreadMessage::MessageFlavour MembershipMessage::getMessageFlavour() const {
    return MEMBERSHIP_MESSAGE;
}
