#ifndef CAUV_APPLICATION_MESSAGE_H_INCLUDED
#define CAUV_APPLICATION_MESSAGE_H_INCLUDED

#include <vector>

typedef std::vector<char> MessageByteBuffer;

class ApplicationMessage {
public:
    virtual MessageByteBuffer getBytes() = 0;
    virtual ApplicationMessage *deserialise(const MessageByteBuffer &bytes) = 0;
};

/*
class ScatterMessage : public ApplicationMessage {
public:
    virtual void addMessagePart(const ApplicationMessage &messagePart);
    virtual void addMessagePart(const std::vector<char> &data);
    virtual void addMessagePart(const void *const data, const int length);
    short getPartsCount() const;
};
*/

#endif // CAUV_APPLICATION_MESSAGE_H_INCLUDED
