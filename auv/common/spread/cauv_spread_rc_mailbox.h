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
 */

class ReconnectingSpreadMailbox{
public:
    ReconnectingSpreadMailbox(std::string const& portAndHost,
                              std::string const& privConnectionName = "",
                              bool recvMembershipMessages = true,
                              ConnectionTimeout const& timeout = SpreadMailbox::ZERO_TIMEOUT,
                              SpreadMailbox::MailboxPriority priority = SpreadMailbox::MEDIUM) throw()
        : m_ci(portAndHost, privConnectionName, recvMembershipMessages, timeout, priority){
        _asyncConnect();
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
    int sendMessage(ApplicationMessage &message, Spread::service serviceType,
                     const std::string &destinationGroup) {
        const char* errmsg = "Failed to send message "; 
        int r = 0;
        if(_waitConnected(20)){
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
    virtual int sendMultigroupMessage(ApplicationMessage &message,
                                      Spread::service serviceType,
                                      const std::vector<std::string> &groupNames) {
        const char* errmsg = "Failed to send multigroup message "; 
        int r = 0;
        if(_waitConnected(20)){
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

    /* called by in a new thread */
    void operator()(){
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
        boost::lock_guard<boost::recursive_mutex> l(m_connection_state_lock);
        return m_connection_state;
    }
    
    /**
     * Return true as soon as possible, or false if timeout is reached.
     */
    bool _waitConnected(unsigned msecs, unsigned attempts = 5) throw(){
        // TODO: timed_wait, or similar
        unsigned i = 0;
        boost::xtime xt;
        for(; _checkConnected() != CONNECTED && i < attempts; i++){ 
            boost::xtime_get(&xt, boost::TIME_UTC);
            xt.sec += msecs / attempts;
            boost::thread::sleep(xt);
        }
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
        
        boost::thread t = boost::thread(boost::ref(*this));
        
    }
    
    ConnectionInfo m_ci;
    
    boost::recursive_mutex m_connection_state_lock;
    ConnectionState m_connection_state;
    
    boost::recursive_mutex m_mailbox_lock;
    boost::shared_ptr<SpreadMailbox> m_mailbox; 
};

#endif // ndef CAUV_SPREAD_RC_MAILBOX_H_INCLUDED
