#ifndef CAUV_MAILBOX_H
#define CAUV_MAILBOX_H
#include <string>
#include <boost/shared_ptr.hpp>

namespace cauv {

class Message;

enum MessageReliability {
    RELIABLE_MSG, 
    UNRELIABLE_MSG
};

class Mailbox {
    public:

    /**
     * @return The number of bytes sent
     */
    virtual int sendMessage(boost::shared_ptr<const Message> message, MessageReliability) = 0;

    /**
     * @return The number of bytes sent
     */
    virtual int sendMessage(boost::shared_ptr<const Message> message, MessageReliability,
                    const std::string &destinationGroup) = 0;

    virtual void joinGroup(const std::string &groupName) = 0;
    virtual void leaveGroup(const std::string &groupName) = 0;
};

} //namespace cauv

#endif //ndef CAUV_MAILBOX_H
