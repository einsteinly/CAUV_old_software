#ifndef __LIQUID_ARC_SINK_H__
#define __LIQUID_ARC_SINK_H__

#include <QGraphicsObject>
#include <QGraphicsLayoutItem>

#include "connectionSink.h"
#include "requiresCutout.h"

namespace liquid {

struct ArcStyle;
struct CutoutStyle;

// Note that you could subclass QGraphicsItem and ConnectionSink separately,
// and do completely custom drawing of the connect-y bit; this provides a class
// that does standard drawing
class ArcSink: public QGraphicsObject,
               public QGraphicsLayoutItem,
               public ConnectionSink,
               public RequiresCutout{
    public:
        // or style could be delegated to something else...
        ArcSink(ArcStyle const& of_style,
                CutoutStyle const& with_cutout,
                ConnectionSink *connectionDelegate);

        // called whilst a drag operation is in progress to test & highlight
        // things, delegates the question to connectionDelgate
        virtual bool willAcceptConnection(void* from_source);
        
        // called whilst a during a drag operation to indicate that this is
        // something that might accept a connection (after willacceptConnection
        // has returned true)
        // intensity = 0: no highlight
        // intensity = 1: maximum highlight
        virtual void doPresentHighlight(qreal intensity);

        // called when the connection is dropped,  delegates the question to
        // connectionDelgate
        virtual ConnectionStatus doAcceptConnection(void* from_source);
    
        // RequiresCutout:
        virtual QList<CutoutStyle> cutoutGeometry() const;
        
        // QGraphicsLayoutItem:
        virtual void setGeometry(QRectF const& rect);

    protected:
        virtual QSizeF sizeHint(Qt::SizeHint which,
                                const QSizeF&
                                constraint=QSizeF()) const;
        virtual QRectF boundingRect() const;
        virtual void paint(QPainter *painter,
                           const QStyleOptionGraphicsItem *option,
                           QWidget *widget = 0);

    private:
        ArcStyle const& m_arc_style; // actually unused
        CutoutStyle const& m_cutout_style;

        ConnectionSink *m_connectionDelegate;
        QGraphicsEllipseItem *m_highlight;

        QRect m_rect;
};

} // namespace liquid

#endif //  __LIQUID_ARC_SINK_H__
