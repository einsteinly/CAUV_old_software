#include <iostream>
#include <vector>
#include <ssrc/spread/Mailbox.h>
#include "cauv_spread_mailbox.h"
#include <boost/shared_ptr.hpp>

using namespace std;
using namespace boost;
using ssrc::spread::Mailbox;
using ssrc::spread::Error;
using ssrc::spread::GroupList;

const ConnectionTimeout SpreadMailbox::ZERO_TIMEOUT;


SpreadMailbox::SpreadMailbox(const string &portAndHost, const string &internalConnectionName,
                  const bool shouldReceiveMembershipMessages, const ConnectionTimeout &timeout,
                  const MailboxPriority priority) throw(ConnectionError) {
    try {
        m_ssrcMailbox = shared_ptr<Mailbox>(
            new Mailbox(portAndHost, internalConnectionName, shouldReceiveMembershipMessages,
                        timeout, (Mailbox::Priority)priority) );
    } catch(Error e) {
        const char *errMsg;
        bool critical = false;
        
        switch( e.error() ) {
        case ILLEGAL_SPREAD:
            errMsg = "Spread daemon port & host were not formatted correctly";
            critical = true;
            break;
        case CONNECTION_CLOSED:
            errMsg = "Connection to Spread daemon was interrupted";
            break;
        case REJECT_VERSION:
            errMsg = "This client is the wrong version for the specified Spread daemon";
            critical = true;
            break;
        case REJECT_NO_NAME:
            errMsg = "No private connection name provided";
            critical = true;
            break;
        case REJECT_ILLEGAL_NAME:
            errMsg = "Illegal private connection name";
            critical = true;
            break;
        case REJECT_NOT_UNIQUE:
            errMsg = "Private connection name already in use";
            break;
        case COULD_NOT_CONNECT:
        default:
            errMsg = "Could not connect to Spread daemon";
            break;
        }

        throw ConnectionError(errMsg, critical);
    }
}


void SpreadMailbox::disconnect() throw(InvalidSessionError) {
    m_ssrcMailbox->kill();
}


void SpreadMailbox::joinGroup(const string &groupName)
        throw(ConnectionError, InvalidSessionError, IllegalGroupError) {
    try {
        m_ssrcMailbox->join(groupName);
    }
    catch(Error e) {
        switch( e.error() ) {
        case ILLEGAL_GROUP:
            throw IllegalGroupError();
            break;
        case ILLEGAL_SESSION:
            throw InvalidSessionError();
            break;
        default:
            throw ConnectionError("Connection error occurred during join");
            break;
        }
    }
}


void SpreadMailbox::leaveGroup(const string &groupName)
        throw(ConnectionError, InvalidSessionError, IllegalGroupError) {
    try {
        m_ssrcMailbox->leave(groupName);
    }
    catch(Error e) {
        switch( e.error() ) {
        case ILLEGAL_GROUP:
            throw IllegalGroupError();
            break;
        case ILLEGAL_SESSION:
            throw InvalidSessionError();
            break;
        default:
            throw ConnectionError("Connection error occurred during leave");
            break;
        }
    }
}

// Helper functions for converting between SSRC GroupLists and vectors
shared_ptr<GroupList> vectorToGroupList(const vector<string> &groups) {
    shared_ptr<GroupList> list = shared_ptr<GroupList>( new GroupList( groups.size() ) );
    for( vector<string>::const_iterator i = groups.begin(); i != groups.end(); i++ ) {
        list->add( *i );
    }
    return list;
}
shared_ptr<GroupList> stringToGroupList(const string & group) {
    shared_ptr<GroupList> list = shared_ptr<GroupList>( new GroupList( 1 ) );
    list->add(group);
    return list;
}
StringVectorPtr groupListToVector(const GroupList &groups) {
    StringVectorPtr v ( new vector<string>( groups.size() ) );
    for( unsigned i = 0; i < groups.size(); i++ ) {
        v->push_back( groups.group(i) );
    }
    return v;
}

int SpreadMailbox::doSendMessage( Message &message, Spread::service serviceType,
        const shared_ptr<GroupList> groupNames ) {
    vector<char> bytes = message.toBytes();
    ssrc::spread::ScatterMessage spreadMsg;
    spreadMsg.add( &bytes[0], bytes.size() ); // Grab the address of the first element of the byte array in memory
    spreadMsg.set_service(serviceType);

    try {
        return m_ssrcMailbox->send(spreadMsg, *groupNames);
    }
    catch(Error e) {
        switch( e.error() ) {
        case ILLEGAL_SESSION:
            throw InvalidSessionError();
            break;
        case ILLEGAL_MESSAGE:
            throw IllegalMessageError();
            break;
        default:
            throw ConnectionError("Connection error occurred during send");
            break;
        }
    }
}

int SpreadMailbox::sendMessage(Message &message, Spread::service serviceType,
        const string &groupName) throw(InvalidSessionError, ConnectionError, IllegalMessageError) {
    return doSendMessage( message, serviceType, stringToGroupList(groupName) );
}


int SpreadMailbox::sendMultigroupMessage(Message &message, Spread::service serviceType,
        const vector<string> &groupNames) throw(InvalidSessionError, ConnectionError, IllegalMessageError) {
    return doSendMessage( message, serviceType, vectorToGroupList(groupNames) );
}


RegularMembershipMessage::MessageCause causeFromServiceType( Spread::service serviceType ) {
    if( Is_caused_join_mess(serviceType) ) {
        return RegularMembershipMessage::JOIN;
    }
    if( Is_caused_disconnect_mess(serviceType) ) {
        return RegularMembershipMessage::DISCONNECT;
    }
    if( Is_caused_leave_mess(serviceType) ) {
        return RegularMembershipMessage::LEAVE;
    }
    return RegularMembershipMessage::NETWORK;
}

shared_ptr<SpreadMessage> SpreadMailbox::receiveMessage() throw(InvalidSessionError, ConnectionError, IllegalMessageError) {
    ssrc::spread::Message ssrcMsg;    // We don't expect to have to deal with ScatterMessages on this end
    GroupList groups;
    m_ssrcMailbox->receive(ssrcMsg, groups);
    shared_ptr< vector<string> > groupVector = groupListToVector(groups);
    ssrc::spread::BaseMessage::service_type sType = ssrcMsg.service();

    if( Is_regular_mess(sType) ) {
        return shared_ptr<RegularMessage>( new RegularMessage( ssrcMsg.sender(), sType,
                                           groupVector, ssrcMsg.type(), &ssrcMsg[0],
                                           ssrcMsg.size() ) );   // [] operator returns ref to underlying array
    }
    else {  // Now we know it's a membership message
        if( Is_transition_mess(sType) ) {
            return shared_ptr<TransitionMembershipMessage>(
                new TransitionMembershipMessage( ssrcMsg.sender() ) );
        }
        else if( Is_reg_memb_mess(sType) ) {
            ssrc::spread::MembershipInfo info;
            ssrcMsg.get_membership_info(info);
            GroupList allMembers;
            info.get_all_members(allMembers);

            StringVectorPtr changedMembers;
            if( info.caused_by_network() ) {
                // For a network partition change, those who entered the new membership
                // with the changed member are all those now in the same network segment
                // as the changed member.
                GroupList localMembers;
                info.get_local_members(localMembers);
                changedMembers = groupListToVector(localMembers);
            }
            else {
                changedMembers = StringVectorPtr( new vector<string>(1) );
                changedMembers->push_back( info.changed_member() );
            }

            return shared_ptr<RegularMembershipMessage>(
                new RegularMembershipMessage( ssrcMsg.sender(), causeFromServiceType(sType),
                                              groupListToVector(allMembers), changedMembers ) );
        }
        else { // Is_self_leave(sType)
            return shared_ptr<SelfLeaveMessage>(
                new SelfLeaveMessage( getPrivateGroupName() ) );
        }
    }
}
