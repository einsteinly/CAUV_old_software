#ifndef CAUV_SPREAD_MESSAGES_H_INCLUDED
#define CAUV_SPREAD_MESSAGES_H_INCLUDED

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <ssrc/spread/Mailbox.h>
#include "../cauv_application_message.h"

typedef boost::shared_ptr< std::vector<std::string> > StringVectorPtr;

/**
 * SpreadMessage is a top-level abstract class representing any kind of message received
 * by a Spread mailbox from the Spread daemon.
 */
class SpreadMessage {
    friend class SpreadMailbox;
public:
    enum MessageFlavour { REGULAR_MESSAGE, MEMBERSHIP_MESSAGE };

    /**
     * @return The broad class of message to which this message belongs
     */
    virtual MessageFlavour getMessageFlavour() const = 0;

protected:
    SpreadMessage( const std::string &sender ) : m_sender(sender) {}

    /**
     * The meaning of this member depends on the message type.
     */
    const std::string m_sender;
};


/**
 * A RegularMessage is a message from the Spread daemon wrapping an application data
 * message along with a bunch of metadata. Constructors are not documented because they
 * are only meant to be used by SpreadMailbox.
 */
class RegularMessage : public SpreadMessage {
    friend class SpreadMailbox;

protected:
    Spread::service m_serviceType;
    StringVectorPtr m_groups;
    int m_messageType;
    boost::shared_ptr<ApplicationMessage> m_messageContents;

    RegularMessage( const std::string &senderName, const Spread::service serviceType,
                    const StringVectorPtr groups, const int messageType,
                    const MessageByteBuffer &bytes )
            // TODO: Is there some way of just passing all the right parameters to the other constructor?
            // TODO: Yes, try using magic, it normally works. Don't,
            // however, use varargs; it breaks constructors.
            : SpreadMessage(senderName), m_serviceType(serviceType), m_groups(groups), m_messageType(messageType),
              m_messageContents( ApplicationMessage::deserialise( &bytes[0], bytes.size() ) ) {}
    RegularMessage( const std::string &senderName, const Spread::service serviceType,
                    const StringVectorPtr groups, const int messageType,
                    const char * const bytes, const int byteCount )
            : SpreadMessage(senderName), m_serviceType(serviceType), m_groups(groups), m_messageType(messageType),
              m_messageContents( ApplicationMessage::deserialise( bytes, byteCount) ) {}

public:
    /**
     * @return The private group name of the sending connection.
     */
    const std::string &getSenderName() const {
        return m_sender;
    }

    /**
     * @return A value representing the service level (mess, ordered, etc.)
     * and Spread message type (membership, regular) of the message.
     */
    const Spread::service getServiceType() const {
        return m_serviceType;
    }

    /**
     * @return The type of the application message as indicated by the sender.
     */
    const int getAppMessageType() const {
        return m_messageType;
    }

    /**
     * @return A list of all groups that received this message.
     */
    const StringVectorPtr getReceivingGroupNames() const {
        return m_groups;
    }

    /**
     * @return The actual application message data.
     */
    const boost::shared_ptr<ApplicationMessage> getMessage() const {
        return m_messageContents;
    }

    virtual MessageFlavour getMessageFlavour() const;
};


/**
 * A MembershipMessage indicates some kind of change in group membership in one of the groups
 * to whose messages a mailbox is subscribed. This is an abstract class.
 */
class MembershipMessage : public SpreadMessage {
    friend class SpreadMailbox;

protected:
    MembershipMessage( const std::string &sender )
            : SpreadMessage(sender) {}

public:
    virtual MessageFlavour getMessageFlavour() const;

    /**
     * @return The name of a group affected by this membership change (precise meaning varies by message type).
     */
    const std::string &getAffectedGroupName() const  {
        return m_sender;
    }
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
    TransitionMembershipMessage( const std::string &sender) : MembershipMessage(sender) {}
public:
    /**
     * @return The group whose membership is going to change.
     */
    const std::string &getAffectedGroupName() const;
};



/**
 * A RegularMembershipMessage is issued when a group's membership changes. It is sent
 * to all group members who have indicated to the daemon that they wish to receive
 * membership change messages.
 */
class RegularMembershipMessage : public MembershipMessage {
    friend class SpreadMailbox;

public:
    enum MessageCause {
        JOIN = CAUSED_BY_JOIN, LEAVE = CAUSED_BY_LEAVE, DISCONNECT = CAUSED_BY_DISCONNECT,
        NETWORK = CAUSED_BY_NETWORK
    };


    /**
     * @return The name of the group for which the membership change is occuring.
     */
    const std::string &getAffectedGroupName() const;

    /**
     * @return A list of members in the affected group after the change has taken place.
     */
    const StringVectorPtr getRemainingGroupMembers() const {
        return m_remaining;
    }

    /**
     * @return The type of membership change event that caused this message.
     */
    const MessageCause getCause() const {
        return m_cause;
    }

    /**
     * Returns a list of group members affected by this membership change.
     * For a join, leave, or disconnect, the list contains just the name of the
     * member who joined, left, or disconnected.
     * For a network partition message, it contains a list of the private group names in
     * the local network segment that remained in the group after the partition-induced
     * change in membership (this list includes the changed member itself).
     * @return A list of affected group members.
     */
    const StringVectorPtr getChangedMemberNames() const {
        return m_changedMembers;
    }

protected:
    RegularMembershipMessage( const std::string &sender, const MessageCause cause,
                              const StringVectorPtr remaining,
                              const StringVectorPtr changedMembers )
            : MembershipMessage(sender), m_remaining(remaining), m_changedMembers(changedMembers), m_cause(cause) {}

    StringVectorPtr m_remaining;
    StringVectorPtr m_changedMembers;
    MessageCause m_cause;
};


/**
 * This is a specialization of a MembershipMessage that confirms to a mailbox that a
 * "leave group" operation has succeeded.
 */
class SelfLeaveMessage : public MembershipMessage {
    friend class SpreadMailbox;

protected:
    SelfLeaveMessage( const std::string &sender)
            : MembershipMessage(sender) {}
public:
    /**
     * @return The name of the group which this mailbox has left.
     */
    const std::string &getAffectedGroupName() const;
};

#endif // CAUV_SPREAD_MESSAGES_H_INCLUDED
