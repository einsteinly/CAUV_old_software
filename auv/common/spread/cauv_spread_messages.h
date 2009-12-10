#ifndef CAUV_SPREAD_MESSAGES_H_INCLUDED
#define CAUV_SPREAD_MESSAGES_H_INCLUDED

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include "../cauv_application_message.h"

/**
 * SpreadMessage is a top-level abstract class representing any kind of message received
 * by a Spread mailbox from the Spread daemon.
 */
class SpreadMessage {
    friend class SpreadMailbox;
public:
    enum MessageType { REGULAR_MESSAGE, MEMBERSHIP_MESSAGE };
    virtual MessageType getMessageType() = 0;

protected:
    /**
     * Protected constructor for use only by derived classes. The meanings of these parameters is as specified
     * in the Spread docs, so they vary depending on what the message type is (see derived classes).
     */
    SpreadMessage( const std::string &sender, const Spread::service serviceType,
                   const boost::shared_ptr< std::vector<std::string> >groups, const int messageType );
    Spread::service serviceType;
};


/**
 * A RegularMessage is a message from the Spread daemon wrapping an application data
 * message along with a bunch of metadata. Constructors are not documented because they
 * are only meant to be used by SpreadMailbox.
 */
class RegularMessage : public SpreadMessage {
    friend class SpreadMailbox;

protected:
    RegularMessage( const std::string &senderName, const Spread::service serviceType,
                    const boost::shared_ptr< std::vector<std::string> >groups, const int messageType,
                    const MessageByteBuffer &bytes );
    RegularMessage( const std::string &senderName, const Spread::service serviceType,
                    const boost::shared_ptr< std::vector<std::string> >groups, const int messageType,
                    const char * const bytes, const int byteCount );
public:
    /**
     * @return The private group name of the sending connection.
     */
    const std::string &getSenderName() const;

    /**
     * @return A value representing the service level (mess, ordered, etc.)
     * and Spread message type (membership, regular) of the message.
     */
    const Spread::service getServiceType() const;

    /**
     * @return The type of the application message as indicated by the sender.
     */
    const int getMessageType() const;

    /**
     * @return A list of all groups that received this message.
     */
    const std::vector<const std::string> &getReceivingGroupNames() const;

    /**
     * @return The actual application message data.
     */
    const ApplicationMessage &getMessage() const;

    virtual MessageType getMessageType();
};


/**
 * A MembershipMessage indicates some kind of change in group membership in one of the groups
 * to whose messages a mailbox is subscribed. This is an abstract class.
 */
class MembershipMessage : public SpreadMessage {
    friend class SpreadMailbox;

public:
    MembershipMessage( const std::string &sender, const Spread::service serviceType,
                   const boost::shared_ptr< std::vector<std::string> >groups, const int messageType )
        : SpreadMessage(sender, serviceType, groups, messageType) {}
    virtual MessageType getMessageType();
public:
    /**
     * @return The name of a group affected by this membership change (precise meaning varies by message type).
     */
    virtual const std::string &getAffectedGroupName() const = 0;
};


/**
 * From the Spread API docs:
 *   "The importance of a TransitionalMembershipMessage is that it tells the application that all
 *   messages received after it and before the RegularMembershipMessage for the same group
 *   are 'clean up' messages to put the messages in a consistant state before actually
 *   changing memberships."
 */
class TransitionMembershipMessage : public MembershipMessage {
    friend class SpreadMailbox;

protected:
    TransitionMembershipMessage( const std::string &sender, const Spread::service serviceType,
                   const boost::shared_ptr< std::vector<std::string> >groups, const int messageType )
        : MembershipMessage(sender, serviceType, groups, messageType) {}
public:
    /**
     * @return The group whose membership is going to change.
     */
    virtual const std::string &getAffectedGroupName() const;
};


enum MembershipMessageCause {
    JOIN = CAUSED_BY_JOIN, LEAVE = CAUSED_BY_LEAVE, DISCONNECT = CAUSED_BY_DISCONNECT, NETWORK = CAUSED_BY_NETWORK
};

/**
 * A RegularMembershipMessage is issued when a group's membership changes. It is sent
 * to all group members who have indicated to the daemon that they wish to receive
 * membership change messages.
 */
class RegularMembershipMessage : public MembershipMessage {
    friend class SpreadMailbox;

protected:
    RegularMembershipMessage( const std::string &sender, const Spread::service serviceType,
                   const boost::shared_ptr< std::vector<std::string> >groups, const int messageType )
        : MembershipMessage(sender, serviceType, groups, messageType) {}
public:
    /**
     * @return The name of the group for which the membership change is occuring.
     */
    virtual const std::string &getAffectedGroupName() const;

    /**
     * @return A list of members in the affected group after the change has taken place.
     */
    const std::vector<const std::string> &getRemainingGroupMembers() const;

    /**
     * @return The type of membership change event that caused this message.
     */
    const MembershipMessageCause getCause() const;

    /**
     * @return A list of all the private group names of those mailboxes which came
     * with the receiving mailbox from the old group membership into this
     * new membership.
     */
    const std::vector<const std::string> &getTransitioningMemberNames() const;
};


/**
 * This is a specialization of a MembershipMessage that confirms to a mailbox that a
 * "leave group" operation has succeeded.
 */
class SelfLeaveMessage : public MembershipMessage {
    friend class SpreadMailbox;

protected:
    SelfLeaveMessage( const std::string &sender, const Spread::service serviceType,
                   const boost::shared_ptr< std::vector<std::string> >groups, const int messageType )
        : MembershipMessage(sender, serviceType, groups, messageType) {}
public:
    /**
     * @return The name of the group which this mailbox has left.
     */
    virtual const std::string &getAffectedGroupName() const;
};

#endif // CAUV_SPREAD_MESSAGES_H_INCLUDED
