#include <vector>
#include <ssrc/spread/Mailbox.h>
#include "cauv_spread_mailbox.h"
#include <boost/shared_ptr.hpp>

using namespace std;
using namespace ssrc::spread;
using namespace boost;

const ConnectionTimeout SpreadMailbox::ZERO_TIMEOUT;


SpreadMailbox::SpreadMailbox(const string &portAndHost, const string &internalConnectionName,
                  const bool shouldReceiveMembershipMessages, const ConnectionTimeout &timeout,
                  const MailboxPriority priority) throw(ConnectionError) {
    try {
        m_ssrcMailbox = new Mailbox(portAndHost, internalConnectionName, shouldReceiveMembershipMessages,
                                    timeout, (Mailbox::Priority)priority);
    } catch(Error e) {
        const char *errMsg;

        switch( e.error() ) {
        case ILLEGAL_SPREAD:
            errMsg = "Spread daemon port & host were not formatted correctly";
            break;
        case CONNECTION_CLOSED:
            errMsg = "Connection to Spread daemon was interrupted";
            break;
        case REJECT_VERSION:
            errMsg = "This client is the wrong version for the specified Spread daemon";
            break;
        case REJECT_NO_NAME:
            errMsg = "No private connection name provided";
            break;
        case REJECT_ILLEGAL_NAME:
            errMsg = "Illegal private connection name";
            break;
        case REJECT_NOT_UNIQUE:
            errMsg = "Private connection name already in use";
            break;
        case COULD_NOT_CONNECT:
        default:
            errMsg = "Could not connect to Spread daemon";
            break;
        }

        throw ConnectionError(errMsg);
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

// Helper functions for converting between GroupLists and vectors (caller is responsible for deleting)
GroupList *vectorToGroupList(const vector<string> &groups) {
    GroupList *list = new GroupList( groups.size() );
    for( vector<string>::const_iterator i = groups.begin(); i != groups.end(); i++ ) {
        list->add( *i );
    }
    return list;
}
GroupList *vectorToGroupList(const string & group) {
    GroupList *list = new GroupList( 1 );
    list->add(group);
    return list;
}
shared_ptr< vector<string> > groupListToVector(const GroupList &groups) {
    shared_ptr< vector<string> > v ( new vector<string>( groups.size() ) );
    for( unsigned i = 0; i < groups.size(); i++ ) {
        v->push_back( groups.group(i) );
    }
    return v;
}

int SpreadMailbox::doSendMessage( ApplicationMessage &message, Spread::service serviceType,
        GroupList *const groupNames ) {
    MessageByteBuffer bytes = message.getBytes();
    ScatterMessage spreadMsg;
    spreadMsg.add( &bytes.front(), bytes.size() ); // Grab the address of the first element of the byte array in memory
    spreadMsg.set_service(serviceType);

    int sentBytes;
    try {
        sentBytes = m_ssrcMailbox->send(spreadMsg, *groupNames);
        delete groupNames;
    }
    catch(Error e) {
        delete groupNames;

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

    return sentBytes;
}

int SpreadMailbox::sendMessage(ApplicationMessage &message, Spread::service serviceType,
        const string &groupName) throw(InvalidSessionError, ConnectionError, IllegalMessageError) {
    return doSendMessage( message, serviceType, vectorToGroupList(groupName) );
}


int SpreadMailbox::sendMultigroupMessage(ApplicationMessage &message, Spread::service serviceType,
        const vector<string> &groupNames) throw(InvalidSessionError, ConnectionError, IllegalMessageError) {
    return doSendMessage( message, serviceType, vectorToGroupList(groupNames) );
}


shared_ptr<SpreadMessage> SpreadMailbox::receiveMessage() throw(InvalidSessionError, ConnectionError, IllegalMessageError) {
    Message ssrcMsg;    // We don't expect to have to deal with ScatterMessages on this end
    GroupList groups;
    m_ssrcMailbox->receive(ssrcMsg, groups);
    shared_ptr< vector<string> > groupVector = groupListToVector(groups);
    BaseMessage::service_type sType = ssrcMsg.service();

    // TODO: Make sure each these is being initialised with the appropriate values for its message type
    if( Is_regular_mess(sType) ) {
        return shared_ptr<RegularMessage>( new RegularMessage( ssrcMsg.sender(), sType,
                                           groupVector, ssrcMsg.type(), &ssrcMsg[0],
                                           ssrcMsg.size() ) );   // [] operator returns ref to underlying array
    }
    else {  // Now we know it's a membership message
        if( Is_transition_mess(sType) ) {
            return shared_ptr<TransitionMembershipMessage>(
                new TransitionMembershipMessage( ssrcMsg.sender(), sType, groupVector, ssrcMsg.type() ) );
        }
        else if( Is_reg_memb_mess(sType) ) {
            return shared_ptr<RegularMembershipMessage>(
                new RegularMembershipMessage( ssrcMsg.sender(), sType, groupVector, ssrcMsg.type() ) );
        }
        else { // Is_self_leave(sType)
            return shared_ptr<SelfLeaveMessage>(
                new SelfLeaveMessage( ssrcMsg.sender(), sType, groupVector, ssrcMsg.type() ) );
        }
    }
}


shared_ptr<SpreadMessage> SpreadMailbox::receiveScatterMessage() throw(InvalidSessionError, ConnectionError, IllegalMessageError) {
    throw runtime_error("Not implemented");
}


SpreadMailbox::~SpreadMailbox() {
    delete m_ssrcMailbox;   // Disconnects if not previously disconnected
}
