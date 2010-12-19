#include "spread_messages.h"

#include <boost/make_shared.hpp>

using namespace cauv;

SpreadMessage::SpreadMessage( const std::string &sender ) : m_sender(sender) {}


RegularMessage::RegularMessage(
            const std::string &senderName,
            const Spread::service serviceType,
            const StringVectorPtr groups,
            const int messageType,
            const svec_t &bytes ) :
        SpreadMessage(senderName),
        m_serviceType(serviceType),
        m_groups(groups),
        m_messageType(messageType),
        m_messageContents(boost::make_shared<svec_t>(&bytes[0], &bytes[0] + bytes.size()) )
{
}
RegularMessage::RegularMessage( const std::string &senderName, const Spread::service serviceType,
                    const StringVectorPtr groups, const int messageType,
                    const char * const bytes, const int byteCount ) :
        SpreadMessage(senderName),
        m_serviceType(serviceType),
        m_groups(groups),
        m_messageType(messageType),
        m_messageContents(boost::make_shared<svec_t>(bytes, bytes+byteCount) )
{
}

SpreadMessage::MessageFlavour RegularMessage::getMessageFlavour() const {
    return REGULAR_MESSAGE;
}

const std::string &RegularMessage::getSenderName() const
{
    return m_sender;
}
const Spread::service& RegularMessage::getServiceType() const
{
    return m_serviceType;
}
int RegularMessage::getAppMessageType() const
{
    return m_messageType;
}
const StringVectorPtr RegularMessage::getReceivingGroupNames() const
{
    return m_groups;
}
const_svec_ptr RegularMessage::getMessage() const
{
    return m_messageContents;
}

MembershipMessage::MembershipMessage( const std::string &sender ) : SpreadMessage(sender) {}
const std::string &MembershipMessage::getAffectedGroupName() const
{
    return m_sender;
}
SpreadMessage::MessageFlavour MembershipMessage::getMessageFlavour() const {
    return MEMBERSHIP_MESSAGE;
}


TransitionMembershipMessage::TransitionMembershipMessage( const std::string &sender) : MembershipMessage(sender) {}


RegularMembershipMessage::RegularMembershipMessage(
            const std::string &sender, const MessageCause cause,
            const StringVectorPtr remaining,
            const StringVectorPtr changedMembers ) : 
        MembershipMessage(sender),
        m_remaining(remaining),
        m_changedMembers(changedMembers),
        m_cause(cause)
{
}
const StringVectorPtr RegularMembershipMessage::getRemainingGroupMembers() const
{
    return m_remaining;
}
RegularMembershipMessage::MessageCause RegularMembershipMessage::getCause() const
{
    return m_cause;
}
const StringVectorPtr RegularMembershipMessage::getChangedMemberNames() const
{
    return m_changedMembers;
}

SelfLeaveMessage::SelfLeaveMessage( const std::string &sender) : MembershipMessage(sender) {}
