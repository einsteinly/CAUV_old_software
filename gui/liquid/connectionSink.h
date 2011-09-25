#ifndef __LIQUID_CONNECTION_SINK_H__
#define __LIQUID_CONNECTION_SINK_H__

namespace liquid {

class ConnectionSink{
    public:
        // called whilst a drag operation is in progress to test & highlight things
        virtual bool willAcceptConnection(void* from_source) = 0;

        // called when the connection is dropped, may return:
        enum ConnectionStatus{Rejected, Accepted, Pending};
        virtual ConnectionStatus doAcceptConnection(void* from_source) = 0;
};

} // namespace liquid

#endif //  __LIQUID_CONNECTION_SINK_H__
