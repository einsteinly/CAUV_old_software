/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
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
