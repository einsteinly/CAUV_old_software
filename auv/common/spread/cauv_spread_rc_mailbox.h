#ifndef CAUV_SPREAD_RC_MAILBOX_H_INCLUDED
#define CAUV_SPREAD_RC_MAILBOX_H_INCLUDED

#include <iostream>
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/utility.hpp>

#include <common/cauv_utils.h>

#include "cauv_spread_mailbox.h"

class ErrOnExit: boost::noncopyable{
    public:
        ErrOnExit(std::string const& msg)
            : m_print_err(true), m_msg(msg){
        }

        ~ErrOnExit(){
            if(m_print_err)
                std::cerr << m_msg << std::endl;
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
    typedef boost::lock_guard<boost::recursive_mutex> lock;
public:
    ReconnectingSpreadMailbox(std::string const& portAndHost,
                              std::string const& privConnectionName = "",
                              bool recvMembershipMessages = true,
                              ConnectionTimeout const& timeout = SpreadMailbox::ZERO_TIMEOUT,
                              SpreadMailbox::MailboxPriority priority = SpreadMailbox::MEDIUM) throw()
        : m_ci(portAndHost, privConnectionName, recvMembershipMessages, timeout, priority),
          m_connection_state(NC), m_keep_trying(true){
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
        ErrOnExit err("Failed to join group ");
        if(_waitConnected(500)){
            try{
                if(m_mailbox){
                    m_mailbox->joinGroup(groupName);
                    err.no();
                }
            }catch(ConnectionError& e){
                err += groupName + ", " + e.what();
                if(!e.critical()){
                    _asyncConnect();
                }
            }catch(std::exception& e){
                err += groupName + ", " + e.what(); 
            }
        }else{
            _asyncConnect();
        }
    }
    
    virtual void leaveGroup(const std::string &groupName) throw() {
        ErrOnExit err("Failed to leave group ");
        if(_waitConnected(500)){
            try{
                if(m_mailbox){
                    m_mailbox->leaveGroup(groupName);
                    err.no();
                }
            }catch(ConnectionError& e){
                err += groupName + ", " + e.what(); 
                if(!e.critical()){
                    _asyncConnect();
                }
            }catch(std::exception& e){
                err += groupName + ", " + e.what(); 
            }
        }else{
            _asyncConnect();
        }
    }
    
    /**
     * @return The number of bytes sent
     */
    int sendMessage(Message const& message, Spread::service serviceType) {
        return sendMessage(message, serviceType, message.group());
    }
    /**
     * @return The number of bytes sent
     */
    int sendMessage(Message const& message, Spread::service serviceType,
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
            }catch(std::exception& e){
                err += "to " + destinationGroup + ", " + e.what(); 
            }
        }else{
            _asyncConnect();
        }
        return r;
    }
    
    /**
     * @return The number of bytes sent
     */
    virtual int sendMultigroupMessage(Message &message,
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
            }catch(std::exception& e){
                err += "to " + to_string(groupNames.size()) + " groups, " + e.what(); 
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
            }catch(std::exception& e){
                err += e.what(); 
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
            }catch(std::exception& e){
                err += e.what();
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
                lock l2(m_connection_state_lock);
                // yay, finally
                m_connection_state = CONNECTED;
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
        lock l(m_connection_state_lock);
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

    void _asyncConnect() throw(){
        // I appreciate that lowlevel functions which print things are really
        // annoying, however this saves a signficiant number of lines of code,
        // since this function is almost always called after an error, in which
        // case printing that we are going to try to reconnect is what we want
        // to do anyway.
        
        boost::unique_lock<boost::recursive_mutex> l(m_connection_state_lock);
        if (m_connection_state != NC) return;
        m_connection_state = CONNECTING;
        l.unlock();

        std::cerr << "reconnecting..." << std::flush; 
        m_thread = boost::thread(boost::ref(*this));
    }
    
    ConnectionInfo m_ci;
    
    boost::recursive_mutex m_connection_state_lock;
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
};

#endif // ndef CAUV_SPREAD_RC_MAILBOX_H_INCLUDED
