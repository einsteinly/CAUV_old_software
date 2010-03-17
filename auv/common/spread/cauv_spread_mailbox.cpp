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
    // ssrc spread doesn't validate this!
    if(internalConnectionName.size() > MAX_PRIVATE_NAME){
        throw(ConnectionError("Private connection name too long", true));
    }

    try {
        m_ssrcMailbox = shared_ptr<Mailbox>(
            new Mailbox(portAndHost, internalConnectionName, shouldReceiveMembershipMessages,
                        timeout, (Mailbox::Priority)priority) );
        std::cout << "Successfully created spread mailbox: "
                  << portAndHost << ": "
                  << internalConnectionName << std::endl;
    } catch(Error e) {
        string errMsg;
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
            errMsg = string("Private connection name already in use (") + internalConnectionName + string(")");
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

int SpreadMailbox::doSendMessage( Message const& message, Spread::service serviceType,
        const shared_ptr<GroupList> groupNames ) {
    byte_vec_t bytes = message.toBytes();
    ssrc::spread::ScatterMessage spreadMsg;
    spreadMsg.add(bytes.data(), bytes.size());
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
    return 0;
}

int SpreadMailbox::sendMessage(Message const& message, Spread::service serviceType,
        const string &groupName) throw(InvalidSessionError, ConnectionError, IllegalMessageError) {
    return doSendMessage( message, serviceType, stringToGroupList(groupName) );
}


int SpreadMailbox::sendMultigroupMessage(Message const& message, Spread::service serviceType,
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

shared_ptr<SpreadMessage> SpreadMailbox::receiveMessage(int timeout) throw(InvalidSessionError, ConnectionError, IllegalMessageError) {
    ssrc::spread::Message ssrcMsg;    // We don't expect to have to deal with ScatterMessages on this end
    GroupList groups;

    // Select with timeout on descriptor
    {
        Spread::mailbox fd = m_ssrcMailbox->descriptor();
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        
        struct timeval towait;
	    towait.tv_sec = timeout / 1000;
	    towait.tv_usec = (timeout % 1000) * 1000;
	    
        int ret = select(fd+1, &fds, NULL, &fds, &towait);
        if(ret < 0)
        {
            throw ConnectionError("Error in select");	        
        }
        else if(ret == 0)
        {
            return shared_ptr<SpreadMessage>();
        }
    }

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
