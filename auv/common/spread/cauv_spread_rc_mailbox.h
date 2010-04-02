#ifndef CAUV_SPREAD_RC_MAILBOX_H_INCLUDED
#define CAUV_SPREAD_RC_MAILBOX_H_INCLUDED

#include <iostream>
#include <string>
#include <set>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/utility.hpp>

#include <common/cauv_utils.h>
#include <common/debug.h>

#include "cauv_spread_mailbox.h"

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

/**
 * A ReconnectingSpreadMailbox automatically handles disconnection problems by trying
 * to reconnect, retrying whatever operation failed due to disconnection.
 * TODO: doesn't retry anything other than connect at the moment. This may be
 * the behaviour we want.
 */

class ReconnectingSpreadMailbox{
    typedef boost::recursive_mutex mutex_t;
    typedef boost::lock_guard<mutex_t> lock_t;
public:
    ReconnectingSpreadMailbox(std::string const& portAndHost,
                              std::string const& privConnectionName = "",
                              bool recvMembershipMessages = true,
                              ConnectionTimeout const& timeout = SpreadMailbox::ZERO_TIMEOUT,
                              SpreadMailbox::MailboxPriority priority = SpreadMailbox::MEDIUM) throw()
        : m_ci(portAndHost, privConnectionName, recvMembershipMessages, timeout, priority),
          m_connection_state_lock(), m_connection_state(NC), m_mailbox(),
          m_keep_trying(true), m_thread(), m_groups_lock(), m_groups(){
        _asyncConnect();
    }

    ~ReconnectingSpreadMailbox() {
        m_keep_trying = false;
        if(m_thread.joinable()) m_thread.join();
    }
    
    /**
     * @return The internal connection identifier assigned to this mailbox.
     */
    std::string getInternalName() {
        if(m_mailbox)
            return m_mailbox->getInternalName();
        else
            return "";
    }
    
    /**
     * @return The unique group name assigned to this mailbox.
     */
    std::string getPrivateGroupName() {
        if(m_mailbox)
            return m_mailbox->getPrivateGroupName();
        else
            return "";
    }
    
    virtual void joinGroup(const std::string &groupName) throw() {
        lock_t l(m_groups_lock);
        m_groups.insert(groupName);
        
        if(m_mailbox && isConnected()){
            _doJoinGroup(groupName);
        }
    }
    
    virtual void leaveGroup(const std::string &groupName) throw() {
        lock_t l(m_groups_lock);
        string_set_t::iterator i;
        if((i = m_groups.find(groupName)) != m_groups.end()){
            m_groups.erase(i);
            _doLeaveGroup(groupName);
        }
    }
    
    /**
     * @return The number of bytes sent
     */
    int sendMessage(boost::shared_ptr<const Message> message, Spread::service serviceType) {
        return sendMessage(message, serviceType, message->group());
    }
    /**
     * @return The number of bytes sent
     */
    int sendMessage(boost::shared_ptr<const Message> message, Spread::service serviceType,
                     const std::string &destinationGroup) {
        ErrOnExit err("Failed to send message "); 
        int r = 0;
        if(_waitConnected(100)){
            try{
                if(m_mailbox){
                    r = m_mailbox->sendMessage(message, serviceType, destinationGroup);
                    err.no();
                }
            }catch(ConnectionError& e){
                err += "to " + destinationGroup + ", " + e.what();
                if(!e.critical()){
                    _asyncConnect();
                }
            }
        }else{
            _asyncConnect();
        }
        return r;
    }
    
    /**
     * @return The number of bytes sent
     */
    virtual int sendMultigroupMessage(boost::shared_ptr<const Message> message,
                                      Spread::service serviceType,
                                      const std::vector<std::string> &groupNames) {
        ErrOnExit err("Failed to send multigroup message "); 
        int r = 0;
        if(_waitConnected(100)){
            try{
                if(m_mailbox){
                    r = m_mailbox->sendMultigroupMessage(message, serviceType, groupNames);
                    err.no();
                }
            }catch(ConnectionError& e){
                err += "to " + to_string(groupNames.size()) + " groups, " + e.what();
                if(!e.critical()){
                    _asyncConnect();
                }
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
    virtual boost::shared_ptr<SpreadMessage> receiveMessage(int timeout) throw() {
        ErrOnExit err("Failed to receive message ");
        boost::shared_ptr<SpreadMessage> r;
        if(_waitConnected(500)){
            try{
                if(m_mailbox){
                    r = m_mailbox->receiveMessage(timeout);
                    err.no();
                }
            }catch(ConnectionError& e){
                err += e.what();
                if(!e.critical()){
                    _asyncConnect();
                }
            }
        }else{
            _asyncConnect();
        }
        return r;
    }

    int waitingMessageByteCount() throw() {
        ErrOnExit err(std::string(__func__) + " error"); 
        int r = 0;
        if(_waitConnected(100)){
            try{
                if(m_mailbox){
                    r = m_mailbox->waitingMessageByteCount();
                    err.no();
                }
            }catch(ConnectionError& e){
                err += e.what();
                if(!e.critical()){
                    _asyncConnect();
                }
            }
        }else{
            _asyncConnect();
        }
        return r;
    }

    bool isConnected() {
        return _checkConnected() == CONNECTED;
    }

    /* called by in a new thread */
    void operator()(){
        // TODO: move these random constants somewhere nice and configurable
        const int min_retry_msecs = 10;
        const int max_retry_msecs = 500;
        const float retry_inc = 1.2;

        int retry_msecs = min_retry_msecs;
        for(;m_keep_trying;){ 
            try {
                m_mailbox = boost::shared_ptr<SpreadMailbox>(
                     new SpreadMailbox(m_ci.portAndHost,
                                       m_ci.privConnectionName,
                                       m_ci.recvMembershipMessages,
                                       m_ci.timeout,
                                       m_ci.priority)
                );
                // yay, finally
                _doOnConnected();
                return;
            } catch(ConnectionError& e) {
                if (e.critical()){
                    throw(e);
                }else{
                    std::cerr << e.what() << ", trying to reconnect..." << std::endl;
                }
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(retry_msecs)); 
            retry_msecs *= retry_inc;
            if(retry_msecs > max_retry_msecs)
                retry_msecs = max_retry_msecs;
        }
    }
    
private:
    enum ConnectionState {NC=0, CONNECTING=1, CONNECTED=2};

    struct ConnectionInfo{ 
        ConnectionInfo(std::string const& ph,
                       std::string const& pcn,
                       bool recvmm,
                       ConnectionTimeout const& t,
                       SpreadMailbox::MailboxPriority p) throw()
            : portAndHost(ph), privConnectionName(pcn), recvMembershipMessages(recvmm), timeout(t), priority(p){
        }

        std::string portAndHost;
        std::string privConnectionName;
        bool recvMembershipMessages;
        ConnectionTimeout timeout;
        SpreadMailbox::MailboxPriority priority;
    };

    ConnectionState _checkConnected() throw(){
        // TODO: m_mailbox->connected() might return false even when we have
        // m_connected_state = CONNECTED
        lock_t l(m_connection_state_lock);
        return m_connection_state;
    }

    /**
     * Return true as soon as possible, or false if timeout is reached.
     */
    bool _waitConnected(unsigned msecs, unsigned attempts = 5) throw(){
        // TODO: timed_wait, or similar
        const unsigned dt = msecs / attempts;
        unsigned i = 0;
        for(; _checkConnected() != CONNECTED && i < attempts; i++)
            boost::this_thread::sleep(boost::posix_time::milliseconds(dt)); 
        return i < attempts; 
    }

    void _doOnConnected() throw(){
        info() << "mailbox connected";
        lock_t l2(m_connection_state_lock);
        m_connection_state = CONNECTED;
        _synchroniseGroups();
    }

    void _doJoinGroup(std::string const& g) throw(){
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

    void _doLeaveGroup(std::string const& g) throw(){
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

    void _synchroniseGroups() throw(){
        lock_t l(m_groups_lock);
        string_set_t::const_iterator i;
        for(i = m_groups.begin(); i != m_groups.end(); i++)
            _doJoinGroup(*i);
    }

    void _asyncConnect() throw(){
        // I appreciate that lowlevel functions which print things are really
        // annoying, however this saves a signficiant number of lines of code,
        // since this function is almost always called after an error, in which
        // case printing that we are going to try to reconnect is what we want
        // to do anyway.
        
        boost::unique_lock<mutex_t> l(m_connection_state_lock);
        if (m_connection_state != NC) return;
        m_connection_state = CONNECTING;
        l.unlock();

        info() << "reconnecting..."; 
        m_thread = boost::thread(boost::ref(*this));
    }
    
    ConnectionInfo m_ci;
    
    mutex_t m_connection_state_lock;
    ConnectionState m_connection_state;
    
    // spread is thread-safe, ssrc spread is probably not thread safe in
    // general, however, it is probably thread safe provided two threads don't
    // try to send messages at the same time, or receive messages at the same
    // time.
    // TODO: mutual exclusion at this level is not practical, so if ssrcspread
    // causes problems, the easiest solution is to duplicate mailboxes
    // per-thread. In practise this probably means two mailboxes, one for the
    // single receing thread, and one mailbox with mutual exclusion for sending
    // messages (since sending messages will probably only happen from one
    // thread anyway)
    boost::shared_ptr<SpreadMailbox> m_mailbox;

    volatile bool m_keep_trying;

    boost::thread m_thread;
    
    /* Groups joined, and groups that we have been asked to join but haven't
     * necessarily done so yet.
     */
    mutex_t m_groups_lock;
    typedef std::set<std::string> string_set_t;
    string_set_t m_groups;
};

#endif // ndef CAUV_SPREAD_RC_MAILBOX_H_INCLUDED
