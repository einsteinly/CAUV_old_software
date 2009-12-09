#ifndef CAUV_APPLICATION_MESSAGE_H_INCLUDED
#define CAUV_APPLICATION_MESSAGE_H_INCLUDED

#include <vector>

class ApplicationMessage {
    virtual std::vector<char> getBytes() = 0;
    virtual ApplicationMessage *deserialise(const std::vector<char> bytes) = 0;
};

/**
 * This class is not currently implemented.
 */
class ScatterMessage : public ApplicationMessage {
    virtual void addMessagePart(const ApplicationMessage &messagePart);
    virtual void addMessagePart(const std::vector<char> &data);
    virtual void addMessagePart(const void *const data, const int length);
    short getPartsCount() const;
};

#endif // CAUV_APPLICATION_MESSAGE_H_INCLUDED
