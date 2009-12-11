#ifndef CAUV_APPLICATION_MESSAGE_H_INCLUDED
#define CAUV_APPLICATION_MESSAGE_H_INCLUDED

#include <vector>
#include <boost/shared_ptr.hpp>

typedef std::vector<char> MessageByteBuffer;

class ApplicationMessage {
public:
    virtual MessageByteBuffer getBytes() = 0;
    static boost::shared_ptr<ApplicationMessage> deserialise(const char *const bytes, const int byteCount) {
        // TODO: Replace this with something functional
        return boost::shared_ptr<ApplicationMessage>( (ApplicationMessage *)0 );
    }
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
