/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __LIQUID_CONNECTION_SINK_H__
#define __LIQUID_CONNECTION_SINK_H__

#include "forward.h"

#include <QtGlobal>

namespace liquid {

class ArcSourceDelegate;

class ConnectionSink{
    public:
        virtual ~ConnectionSink(){ }

        // called whilst a drag operation is in progress to test & highlight things
        virtual bool willAcceptConnection(ArcSourceDelegate* from_source, AbstractArcSink* to_sink) = 0;

        // called when the connection is dropped, may return:
        enum ConnectionStatus{Rejected, Accepted, Pending};
        virtual ConnectionStatus doAcceptConnection(ArcSourceDelegate* from_source, AbstractArcSink* to_sink) = 0;
};

// convenience class
class RejectingConnectionSink : public ConnectionSink {
    public:
        RejectingConnectionSink() : ConnectionSink() {
        }

        virtual bool willAcceptConnection(ArcSourceDelegate* from_source, AbstractArcSink* to_sink) {
            Q_UNUSED(from_source);
            Q_UNUSED(to_sink);
            return false;
        }

        virtual ConnectionStatus doAcceptConnection(ArcSourceDelegate* from_source, AbstractArcSink* to_sink) {
            Q_UNUSED(from_source);
            Q_UNUSED(to_sink);
            return Rejected;
        }
};

} // namespace liquid

#endif //  __LIQUID_CONNECTION_SINK_H__
