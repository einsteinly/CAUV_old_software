#include "spread_mailbox.h"

#include <iostream>
#include <vector>
#include <cstring>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ssrc/spread/Mailbox.h>

#include <utility/string.h>
#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>

using namespace std;
using namespace boost;
using namespace cauv;
using ssrc::spread::Mailbox;
using ssrc::spread::Error;
using ssrc::spread::GroupList;

const ConnectionTimeout SpreadMailbox::ZERO_TIMEOUT;


void SpreadMailbox::connect(const string &portAndHost, const string &internalConnectionName,
                  const bool shouldReceiveMembershipMessages, const ConnectionTimeout &timeout,
                  const MailboxPriority priority) {
    // ssrc spread doesn't validate this!
    if(internalConnectionName.size() > MAX_PRIVATE_NAME){
        throw ConnectionError("Private connection name too long", true);
    }

    try {
        debug() << "Attempting to connect to"
               << portAndHost << "as"
               << internalConnectionName;
        m_ssrcMailbox = shared_ptr<Mailbox>(
            new Mailbox(portAndHost, internalConnectionName, shouldReceiveMembershipMessages,
                        timeout, (Mailbox::Priority)priority) );
        info() << "Successfully created spread mailbox. Connected to"
               << portAndHost << "as"
               << internalConnectionName;
    } catch(Error& e) {
        handleSpreadError(e);
    }
}


void SpreadMailbox::disconnect() {
    try {
        m_ssrcMailbox->kill();
    } catch(Error& e){
        // NB: can't use handleSpreadError here!
        throw ConnectionError(e.error());
    }
}


void SpreadMailbox::joinGroup(const string &groupName) {
    try {
        m_ssrcMailbox->join(groupName);
    }
    catch(Error& e) {
        handleSpreadError(e);
    }
}


void SpreadMailbox::leaveGroup(const string &groupName) {
    try {
        m_ssrcMailbox->leave(groupName);
    }
    catch(Error& e) {
        handleSpreadError(e);
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

int SpreadMailbox::doSendMessage(const boost::shared_ptr<const Message> message,
                                 Spread::service serviceType,
                                 const shared_ptr<GroupList> groupNames) {
    const_svec_ptr bytes = message->toBytes();

#ifdef CAUV_DEBUG_MESSAGES
    std::stringstream ss;
    for (unsigned i = 0; i < bytes->size(); i++)
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)(*bytes)[i] << " ";
    debug(9) << "->Spread: Sending message" << *message;
    debug(10) << "->Data:\n\t" << ss.str();
#endif

    ssrc::spread::ScatterMessage spreadMsg;
    spreadMsg.add(bytes->data(), bytes->size());
    spreadMsg.set_service(serviceType);

    try {
        return m_ssrcMailbox->send(spreadMsg, *groupNames);
    }
    catch(Error& e) {
        handleSpreadError(e);
    }
    return 0;
}

int SpreadMailbox::sendMessage(const boost::shared_ptr<const Message> message, Spread::service serviceType) {
    return sendMessage( message, serviceType, message->group() );
}
int SpreadMailbox::sendMessage(const boost::shared_ptr<const Message> message, Spread::service serviceType,
        const string &groupName) {
    return doSendMessage( message, serviceType, stringToGroupList(groupName) );
}


int SpreadMailbox::sendMultigroupMessage(const boost::shared_ptr<const Message> message, Spread::service serviceType,
        const vector<string> &groupNames) {
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

shared_ptr<SpreadMessage> SpreadMailbox::receiveMessage() {
    ssrc::spread::Message ssrcMsg;    // We don't expect to have to deal with ScatterMessages on this end
    GroupList groups;

    // Select with timeout on descriptor
    {
        Spread::mailbox fd = m_ssrcMailbox->descriptor();
        fd_set fds;
        FD_ZERO(&fds);
        struct timeval towait;
	    
        int ret = 0;
        while (true) {
            FD_SET(fd, &fds);
            towait.tv_sec = 2;
            towait.tv_usec = 0;
            ret = select(fd+1, &fds, NULL, NULL, &towait);
            if (ret != 0) {
                break;
            }
            boost::this_thread::interruption_point();
        }
        if(ret < 0)
        {
            throw ConnectionError(MakeString() << "Error in select (" << errno << ": " << strerror(errno) << ")", true);	        
        }
    }

    try {
        m_ssrcMailbox->receive(ssrcMsg, groups);
    }
    catch(Error& e) {
        handleSpreadError(e);
    }

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

/* re-throw, disconnect etc as required */
void SpreadMailbox::handleSpreadError(Error& e) {
    if(needsReconnect(e.error())){
        try{
            disconnect();
        }catch(ConnectionError& e2){
            throw ConnectionError(MakeString() << "Error whilst "
                << "dissconnecting following exception, original error:\n\t"
                << getErrorString(e.error()) << "\nSubsequent error:\n\t"
                << e2.what(), true, false);
        }
    }
    throw ConnectionError(e.error()); 
}

