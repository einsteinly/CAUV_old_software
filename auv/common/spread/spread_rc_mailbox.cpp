#include "spread_rc_mailbox.h"

#include <iostream>
#include <string>
#include <set>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/utility.hpp>

#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>

#include "spread_mailbox.h"

class ErrOnExit: boost::noncopyable{
    public:
        ErrOnExit(std::string const& msg)
            : m_print_err(true), m_msg(msg){
        }

        ~ErrOnExit(){
            if(m_print_err)
                error() << m_msg;
        }

        ErrOnExit& operator+=(std::string const& msg){
            m_msg += msg;
            return *this;
        }

        void no(){
            m_print_err = false;
        }

    private:
        bool m_print_err;
        std::string m_msg;
};


ReconnectingSpreadMailbox::ReconnectingSpreadMailbox() :
    m_connection_state_lock(),
    m_connection_state(DISCONNECTED),
    m_mailbox(),
    m_keep_trying(true),
    m_no_threads(false),
    m_thread(),
    m_groups_lock(),
    m_groups()
{
}

ReconnectingSpreadMailbox::~ReconnectingSpreadMailbox() {
    m_keep_trying = false;
    if(m_thread.joinable()) m_thread.join();
}

void ReconnectingSpreadMailbox::connect(
    std::string const& portAndHost,
    std::string const& privConnectionName,
    bool recvMembershipMessages,
    ConnectionTimeout const& timeout,
    SpreadMailbox::MailboxPriority priority
)
{
    _disconnect();
    m_ci = ConnectionInfo(portAndHost, privConnectionName, recvMembershipMessages, timeout, priority);
    _asyncConnect();
}

void ReconnectingSpreadMailbox::run() {
    operator()();
}

/**
 * @return The internal connection identifier assigned to this mailbox.
 */
std::string ReconnectingSpreadMailbox::getInternalName() {
    if(m_mailbox)
        return m_mailbox->getInternalName();
    else
        return "";
}

/**
 * @return The unique group name assigned to this mailbox.
 */
std::string ReconnectingSpreadMailbox::getPrivateGroupName() {
    if(m_mailbox)
        return m_mailbox->getPrivateGroupName();
    else
        return "";
}

void ReconnectingSpreadMailbox::joinGroup(const std::string &groupName) {
    lock_t l(m_groups_lock);
    m_groups.insert(groupName);
    
    if(m_mailbox && isConnected()){
        _doJoinGroup(groupName);
    }
}

void ReconnectingSpreadMailbox::leaveGroup(const std::string &groupName) {
    lock_t l(m_groups_lock);
    string_set_t::iterator i = m_groups.find(groupName);
    if(i != m_groups.end()){
        m_groups.erase(i);
        _doLeaveGroup(groupName);
    }
}

/**
 * @return The number of bytes sent
 */
int ReconnectingSpreadMailbox::sendMessage(boost::shared_ptr<const Message> message,
                                           Spread::service serviceType) {
    return sendMessage(message, serviceType, message->group());
}

void ReconnectingSpreadMailbox::handleConnectionError(ConnectionError& e){
    if(!e.critical())
        _asyncConnect();
    else
        _disconnect();
}
/**
 * @return The number of bytes sent
 */
int ReconnectingSpreadMailbox::sendMessage(boost::shared_ptr<const Message> message,
                                           Spread::service serviceType,
                                           const std::string &destinationGroup) {
    ErrOnExit err("Failed to send message "); 
    int r = 0;
    if(_waitConnected(500)){
        try{
            if(m_mailbox){
                r = m_mailbox->sendMessage(message, serviceType, destinationGroup);
                err.no();
            }
        }catch(ConnectionError& e){
            err += "to " + destinationGroup + ", " + e.what();
            handleConnectionError(e);
        }
    }else{
        _asyncConnect();
    }
    return r;
}

/**
 * @return The number of bytes sent
 */
int ReconnectingSpreadMailbox::sendMultigroupMessage(boost::shared_ptr<const Message> message,
                                                     Spread::service serviceType,
                                                     const std::vector<std::string> &groupNames) {
    ErrOnExit err("Failed to send multigroup message "); 
    int r = 0;
    if(_waitConnected(500)){
        try{
            if(m_mailbox){
                r = m_mailbox->sendMultigroupMessage(message, serviceType, groupNames);
                err.no();
            }
        }catch(ConnectionError& e){
            err += "to " + to_string(groupNames.size()) + " groups, " + e.what();
            handleConnectionError(e);
        }
    }else{
        _asyncConnect();
    }
    return r;
}


/**
 * Blocks until a message comes in from the Spread daemon. The received messsage may
 * be anything in the RegularMessage or MembershipMessage hierarchies.
 * @return An object containing the received message and associated metadata.
 */
boost::shared_ptr<SpreadMessage> ReconnectingSpreadMailbox::receiveMessage() {
    while(true)
    {
        ErrOnExit err("Failed to receive message ");
        if(_waitConnected(1000)){
            try{
                if(m_mailbox){
                    boost::shared_ptr<SpreadMessage> r = m_mailbox->receiveMessage();
                    err.no();
                    return r;
                }
            }catch(ConnectionError& e){
                err += e.what();
                handleConnectionError(e);
            }
        }else{
            _asyncConnect();
        }
    }
}

int ReconnectingSpreadMailbox::waitingMessageByteCount() {
    error() << __func__ << __FILE__ << __LINE__ << "unimplemented";
    return 0;
/* waitingMessageByteCount is not implemented on SpreadMailbox...
    while(true)
    {
        ErrOnExit err(std::string(__func__) + " error"); 
        if(_waitConnected(100)){
            try{
                if(m_mailbox){
                    int r = m_mailbox->waitingMessageByteCount();
                    err.no();
                    return r;
                }
            }catch(ConnectionError& e){
                err += e.what();
                handleConnectionError(e);
            }
        }else{
            _asyncConnect();
        }
    }
*/
}

bool ReconnectingSpreadMailbox::isConnected() {
    return _checkConnected() == CONNECTED;
}

/* called by in a new thread */
void ReconnectingSpreadMailbox::operator()(){
    // TODO: move these random constants somewhere nice and configurable
    const int min_retry_msecs = 10;
    const int max_retry_msecs = 500;
    const float retry_inc = 1.2;

    int retry_msecs = min_retry_msecs;
    for(;m_keep_trying;){ 
        try {
            m_mailbox = boost::make_shared<SpreadMailbox>();
            m_mailbox->connect(m_ci.portAndHost,
                               m_ci.privConnectionName,
                               m_ci.recvMembershipMessages,
                               m_ci.timeout,
                               m_ci.priority);
            // yay, finally
            _doOnConnected();
            return;
        } catch(ConnectionError& e) {
            if (e.critical()){
                warning() << "critical error:" << e.what() << "(will re-throw)";
                _disconnect();
                throw e;
            }else{
                error() << e.what() << ", trying to reconnect...";
            }
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(retry_msecs)); 
        retry_msecs *= retry_inc;
        if(retry_msecs > max_retry_msecs)
            retry_msecs = max_retry_msecs;
    }
}

ReconnectingSpreadMailbox::ConnectionState ReconnectingSpreadMailbox::_checkConnected(){
    // TODO: m_mailbox->connected() might return false even when we have
    // m_connected_state = CONNECTED
    lock_t l(m_connection_state_lock);
    return m_connection_state;
}

/**
 * Return true as soon as possible, or false if timeout is reached.
 */
bool ReconnectingSpreadMailbox::_waitConnected(unsigned msecs, unsigned attempts){
    // TODO: timed_wait, or similar
    const unsigned dt = msecs / attempts;
    unsigned i = 0;
    for(; _checkConnected() != CONNECTED && i < attempts; i++)
        boost::this_thread::sleep(boost::posix_time::milliseconds(dt)); 
    return i < attempts; 
}

void ReconnectingSpreadMailbox::_doOnConnected() {
    info() << "mailbox connected";
    lock_t l2(m_connection_state_lock);
    m_connection_state = CONNECTED;
    _synchroniseGroups();
}

void ReconnectingSpreadMailbox::_doJoinGroup(std::string const& g) {
    ErrOnExit err("Failed to join group ");
    try{
        if(m_mailbox){
            m_mailbox->joinGroup(g);
            err.no();
        }
    }catch(ConnectionError& e){
        err += g + ", " + e.what();
        if(!e.critical()){
            _asyncConnect();
        }
    }
}

void ReconnectingSpreadMailbox::_doLeaveGroup(std::string const& g) {
    ErrOnExit err("Failed to leave group ");
    try{
        if(m_mailbox){
            m_mailbox->leaveGroup(g);
            err.no(); 
        }
    }catch(ConnectionError& e){
        err += g + ", " + e.what(); 
        if(!e.critical()){
            _asyncConnect();
        }
    }
}

void ReconnectingSpreadMailbox::_synchroniseGroups() {
    lock_t l(m_groups_lock);
    string_set_t::const_iterator i;
    for(i = m_groups.begin(); i != m_groups.end(); i++)
        _doJoinGroup(*i);
}

void ReconnectingSpreadMailbox::_asyncConnect(){
    if(!m_no_threads){
        // I appreciate that lowlevel functions which print things are really
        // annoying, however this saves a signficiant number of lines of code,
        // since this function is almost always called after an error, in which
        // case printing that we are going to try to reconnect is what we want
        // to do anyway.
        
        boost::unique_lock<mutex_t> l(m_connection_state_lock);
        if (m_connection_state != DISCONNECTED) return;
        m_connection_state = CONNECTING;
        l.unlock();

        info() << "reconnecting..."; 
        m_thread = boost::thread(boost::ref(*this));
    }
}

void ReconnectingSpreadMailbox::_disconnect() {
    boost::unique_lock<mutex_t> l(m_connection_state_lock);
    if (m_connection_state == DISCONNECTED) return;
    m_connection_state = DISCONNECTED; 
    info() << "disconnecting...";
    m_mailbox.reset();
    m_keep_trying = false;
    if(m_thread.joinable())
        m_thread.join();
    m_keep_trying = true;
}

