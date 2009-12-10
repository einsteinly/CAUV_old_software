#include "cauv_spread_mailbox.h"
#include <ssrc/spread/Mailbox.h>
using namespace std;

const Timeout SpreadMailbox::ZERO_TIMEOUT;

SpreadMailbox::SpreadMailbox(const string &portAndHost, const string &internalConnectionName,
                  const bool shouldReceiveMembershipMessages, const Timeout &timeout,
                  const MailboxPriority priority) throw(ConnectionError) {
    try {
        m_ssrcMailbox = new NS_SSRCSPREAD::Mailbox(portAndHost, internalConnectionName, shouldReceiveMembershipMessages,
                                    timeout, (NS_SSRCSPREAD::Mailbox::Priority)priority);
    } catch(NS_SSRCSPREAD::Error e) {
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

void SpreadMailbox::joinGroup(const string &groupName) throw(ConnectionError, InvalidSessionError, IllegalGroupError) {
    try {
        m_ssrcMailbox->join(groupName);
    }
    catch(NS_SSRCSPREAD::Error e) {
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

void SpreadMailbox::leaveGroup(const string &groupName) throw(ConnectionError, InvalidSessionError, IllegalGroupError) {
    try {
        m_ssrcMailbox->leave(groupName);
    }
    catch(NS_SSRCSPREAD::Error e) {
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

SpreadMailbox::~SpreadMailbox() {
    delete m_ssrcMailbox;   // Disconnects if not previously disconnected
}
