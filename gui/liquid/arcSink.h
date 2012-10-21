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

#ifndef __LIQUID_ARC_SINK_H__
#define __LIQUID_ARC_SINK_H__

#include <QGraphicsObject>
#include <QGraphicsLayoutItem>

#include "connectionSink.h"
#include "requiresCutout.h"
#include "forward.h"
#include "layout.h"

namespace liquid {

class AbstractArcSink: public QGraphicsObject,
                       public ConnectionSink,
                       public LayoutItem,
                       protected LayoutItems{
    Q_OBJECT
    public:
        AbstractArcSink(QGraphicsItem * parent = 0);
        virtual ~AbstractArcSink();

        virtual bool willAcceptConnection(ArcSourceDelegate* from_source, AbstractArcSink* to_sink) = 0;
        virtual void doPresentHighlight(qreal intensity) = 0;
        virtual ConnectionStatus doAcceptConnection(ArcSourceDelegate* from_source, AbstractArcSink* to_sink) = 0;

        // !!! base QGraphicsItem::setParentItem isn't virtual, so this is
        // probably a bad idea!
        virtual void setParentItem(QGraphicsItem* item);
        
        /* Return the top-level QGraphicsItem of which this is a child: used
         * for arc-directed layout.
         */
        virtual QGraphicsItem* ultimateParent();

    Q_SIGNALS:
        void geometryChanged();
        void disconnected(AbstractArcSink*);

    private:
        void disconnectParentSignals(QGraphicsItem* parent);
        void    connectParentSignals(QGraphicsItem* parent);
};

class ArcSink: public AbstractArcSink,
               public QGraphicsLayoutItem,
               public RequiresCutout{
    Q_OBJECT
    // No idea why this is required....
    Q_INTERFACES(QGraphicsLayoutItem)
    public:
        // Note that in the current implementation the reference to each style
        // is stored. The styles ARE NOT COPIED. This is a sensible because in
        // practise styles are constant static global things, if we start
        // generating them dyamically then we either need to pass them around
        // by shared pointer, or by value (undesirable)
        //
        // or style could be delegated to something else... 
        ArcSink(ArcStyle const& of_style,
                CutoutStyle const& with_cutout,
                ConnectionSink *connectionDelegate);
        virtual ~ArcSink();

        // called whilst a drag operation is in progress to test & highlight
        // things, delegates the question to connectionDelgate
        virtual bool willAcceptConnection(ArcSourceDelegate* from_source, AbstractArcSink* to_sink);
        
        // called whilst a during a drag operation to indicate that this is
        // something that might accept a connection (after willacceptConnection
        // has returned true)
        // intensity = 0: no highlight
        // intensity = 1: maximum highlight
        virtual void doPresentHighlight(qreal intensity);

        // called when the connection is dropped,  delegates the question to
        // connectionDelgate
        virtual ConnectionStatus doAcceptConnection(ArcSourceDelegate* from_source, AbstractArcSink* to_sink);

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
        QGraphicsPathItem *m_highlight;
        QGraphicsPathItem *m_back;

        QRect m_rect;
};

} // namespace liquid

#endif //  __LIQUID_ARC_SINK_H__
