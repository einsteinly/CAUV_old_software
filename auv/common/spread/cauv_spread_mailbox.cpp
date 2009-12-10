#include <vector>
#include <ssrc/spread/Mailbox.h>
#include "cauv_spread_mailbox.h"

using namespace std;
using namespace ssrc::spread;

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

// Helper functions for creating GroupLists from vectors (caller is responsible for deleting)
GroupList *makeGroupList(const vector<string> &groups) {
    GroupList *list = new GroupList( groups.size() );
    for( vector<string>::const_iterator i = groups.begin(); i != groups.end(); i++ ) {
        list->add( *i );
    }
    return list;
}
GroupList *makeGroupList(const string & group) {
    GroupList *list = new GroupList( 1 );
    list->add(group);
    return list;
}

int SpreadMailbox::doSendMessage( ApplicationMessage &message, Spread::service serviceType,
        GroupList *const groupNames ) {
    MessageByteBuffer bytes = message.getBytes();
    ScatterMessage spreadMsg;
    spreadMsg.add( &bytes.front(), bytes.size() ); // Grab the address of the first element of the byte array in memory
    spreadMsg.set_service(serviceType);

    int sentBytes;
    try {
        sentBytes = m_ssrcMailbox->send(spreadMsg);
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
    return doSendMessage( message, serviceType, makeGroupList(groupName) );
}


int SpreadMailbox::sendMultigroupMessage(ApplicationMessage &message, Spread::service serviceType,
        const vector<string> &groupNames) throw(InvalidSessionError, ConnectionError, IllegalMessageError) {
    return doSendMessage( message, serviceType, makeGroupList(groupNames) );
}


SpreadMessage SpreadMailbox::receiveMessage() throw(InvalidSessionError, ConnectionError, IllegalMessageError) {
}
SpreadMessage SpreadMailbox::receiveScatterMessage() throw(InvalidSessionError, ConnectionError, IllegalMessageError) {
}


SpreadMailbox::~SpreadMailbox() {
    delete m_ssrcMailbox;   // Disconnects if not previously disconnected
}
