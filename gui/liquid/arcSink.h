#ifndef __LIQUID_ARC_SINK_H__
#define __LIQUID_ARC_SINK_H__

#include <QGraphisItem>

#include "connectionSink.h"

namespace liquid {

struct ArcStyle;

// Note that you could subclass QGraphicsItem and ConnectionSink separately,
// and do completely custom drawing of the connect-y bit; this provides a class
// that does standard drawing, using an ArcStyle, and delegates the connection
// logic to another class
class ArcSink: public QGraphicsItem, public ConnectionSink{
    public:
        // or style could be delegated to something else...
        ArcSink(ArcStlye const& of_style,
                ConnectionSink *connectionDelegate);

        // called whilst a drag operation is in progress to test & highlight
        // things, delegates the question to connectionDelgate
        virtual bool willAcceptConnection(void* from_source);

        // called when the connection is dropped,  delegates the question to
        // connectionDelgate
        virtual ConnectionStatus doAcceptConnection(void* from_source);
        
    private:
        void m_connectionDelegate;
};

} // namespace liquid

#endif //  __LIQUID_ARC_SINK_H__
