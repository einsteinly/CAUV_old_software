#ifndef CAUV_SPREAD_RC_MAILBOX_H_INCLUDED
#define CAUV_SPREAD_RC_MAILBOX_H_INCLUDED

#include <iostream>
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>

#include "cauv_spread_mailbox.h"

/**
 * A ReconnectingSpreadMailbox automatically handles disconnection problems by trying
 * to reconnect, retrying whatever operation failed due to disconnection.
 * TODO: doesn't retry anything other than connect at the moment. This may be
 * the behaviour we want.
 */

class ReconnectingSpreadMailbox{
public:
    ReconnectingSpreadMailbox(std::string const& portAndHost,
                              std::string const& privConnectionName = "",
                              bool recvMembershipMessages = true,
                              ConnectionTimeout const& timeout = SpreadMailbox::ZERO_TIMEOUT,
                              SpreadMailbox::MailboxPriority priority = SpreadMailbox::MEDIUM) throw()
        : m_ci(portAndHost, privConnectionName, recvMembershipMessages, timeout, priority),
          m_connection_state(NC){
        _asyncConnect();
    }

    ~ReconnectingSpreadMailbox() {
        if(m_thread.joinable()) m_thread.join();
    }
    
    /**
     * @return The internal connection identifier assigned to this mailbox.
     */
    std::string getInternalName() {
        boost::lock_guard<boost::recursive_mutex> l(m_mailbox_lock);
        if(m_mailbox)
            return m_mailbox->getInternalName();
        else
            return "";
    }
    
    /**
     * @return The unique group name assigned to this mailbox.
     */
    std::string getPrivateGroupName() {
        boost::lock_guard<boost::recursive_mutex> l(m_mailbox_lock);
        if(m_mailbox)
            return m_mailbox->getPrivateGroupName();
        else
            return "";
    }
    
    virtual void joinGroup(const std::string &groupName) throw() {
        const char* errmsg = "Failed to join group ";
        if(_waitConnected(100)){
            try{
                boost::lock_guard<boost::recursive_mutex> l(m_mailbox_lock);
                if(m_mailbox)
                    m_mailbox->joinGroup(groupName);
            }catch(ConnectionError& e){
                std::cerr << errmsg << groupName << ", " << e.what() << std::endl;
                if(!e.critical()){
                    _asyncConnect();
                }
            }catch(std::exception& e){
                std::cerr << errmsg << groupName << ", " << e.what() << std::endl;
            }
        }else{
            _asyncConnect();
        }
    }
    
    virtual void leaveGroup(const std::string &groupName) throw() {
        const char* errmsg = "Failed to leave group ";
        if(_waitConnected(100)){
            try{
                boost::lock_guard<boost::recursive_mutex> l(m_mailbox_lock);
                if(m_mailbox)
                    m_mailbox->leaveGroup(groupName);
            }catch(ConnectionError& e){
                std::cerr << errmsg << groupName << ", " << e.what() << std::endl;
                if(!e.critical()){
                    _asyncConnect();
                }
            }catch(std::exception& e){
                std::cerr << errmsg << groupName << ", " << e.what() << std::endl;
            }
        }else{
            _asyncConnect();
        }
    }
    
    /**
     * @return The number of bytes sent
     */
    int sendMessage(Message &message, Spread::service serviceType) {
        return sendMessage(message, serviceType, message.group());
    }
    /**
     * @return The number of bytes sent
     */
    int sendMessage(Message &message, Spread::service serviceType,
                     const std::string &destinationGroup) {
        const char* errmsg = "Failed to send message "; 
        int r = 0;
        if(_waitConnected(100)){
            try{
                boost::lock_guard<boost::recursive_mutex> l(m_mailbox_lock);
                if(m_mailbox)
                    r = m_mailbox->sendMessage(message, serviceType, destinationGroup);
            }catch(ConnectionError& e){
                std::cerr << errmsg << /*message <<*/ " msg to " << destinationGroup << ", " << e.what() << std::endl;
                if(!e.critical()){
                    _asyncConnect();
                }
            }catch(std::exception& e){
                std::cerr << errmsg << /*message <<*/ " msg to " << destinationGroup << ", " << e.what() << std::endl;
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
        const char* errmsg = "Failed to send multigroup message "; 
        int r = 0;
        if(_waitConnected(100)){
            try{
                boost::lock_guard<boost::recursive_mutex> l(m_mailbox_lock);
                if(m_mailbox)
                    r = m_mailbox->sendMultigroupMessage(message, serviceType, groupNames);
            }catch(ConnectionError& e){
                std::cerr << errmsg << /*message <<*/ " msg to " << groupNames.size() << " groups, " << e.what() << std::endl;
                if(!e.critical()){
                    _asyncConnect();
                }
            }catch(std::exception& e){
                std::cerr << errmsg << /*message <<*/ " msg to " << groupNames.size() << " groups, " << e.what() << std::endl;
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
    virtual boost::shared_ptr<SpreadMessage> receiveMessage() throw() {
        const char* errmsg = "Failed to receive message "; 
        boost::shared_ptr<SpreadMessage> r;
        if(_waitConnected(500)){
            try{
                boost::lock_guard<boost::recursive_mutex> l(m_mailbox_lock);
                if(m_mailbox)
                    r = m_mailbox->receiveMessage();
            }catch(ConnectionError& e){
                std::cerr << errmsg << ", " << e.what() << std::endl;
                if(!e.critical()){
                    _asyncConnect();
                }
            }catch(std::exception& e){
                std::cerr << errmsg << ", " << e.what() << std::endl;
            }
        }else{
            _asyncConnect();
        }
        return r;
    }

    int waitingMessageByteCount() throw() {
        std::string errmsg = std::string(__func__) + " error"; 
        int r = 0;
        if(_waitConnected(100)){
            try{
                boost::lock_guard<boost::recursive_mutex> l(m_mailbox_lock);
                if(m_mailbox)
                    r = m_mailbox->waitingMessageByteCount();
            }catch(ConnectionError& e){
                std::cerr << errmsg << ", " << e.what() << std::endl;
                if(!e.critical()){
                    _asyncConnect();
                }
            }catch(std::exception& e){
                std::cerr << errmsg << ", " << e.what() << std::endl;
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
        for(;;){
            try {
                boost::lock_guard<boost::recursive_mutex> l(m_mailbox_lock);
                m_mailbox = boost::shared_ptr<SpreadMailbox>(
                     new SpreadMailbox(m_ci.portAndHost,
                                       m_ci.privConnectionName,
                                       m_ci.recvMembershipMessages,
                                       m_ci.timeout,
                                       m_ci.priority)
                );
                boost::lock_guard<boost::recursive_mutex> l2(m_connection_state_lock);
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
        boost::lock_guard<boost::recursive_mutex> l(m_connection_state_lock);
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
        std::cerr << "reconnecting..." << std::endl;
        
        boost::unique_lock<boost::recursive_mutex> l(m_connection_state_lock);
        if (m_connection_state != NC) return;
        m_connection_state = CONNECTING;
        l.unlock();
        
        m_thread = boost::thread(boost::ref(*this));
    }
    
    ConnectionInfo m_ci;
    
    boost::recursive_mutex m_connection_state_lock;
    ConnectionState m_connection_state;
    
    boost::recursive_mutex m_mailbox_lock;
    boost::shared_ptr<SpreadMailbox> m_mailbox; 
    boost::thread m_thread;
};

#endif // ndef CAUV_SPREAD_RC_MAILBOX_H_INCLUDED
