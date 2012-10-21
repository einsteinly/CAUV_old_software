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

#ifndef __LIQUID_ARC_H__
#define __LIQUID_ARC_H__

#include <set>

#include "arcSource.h"
#include "layout.h"

namespace liquid {

struct ArcStyle;
class AbstractArcSink;
class EphemeralArcEnd;

class Arc: public _::AbstractArcSourceInternal,
           protected LayoutItems{
        Q_OBJECT
    private:
        typedef std::set<AbstractArcSink*> arcsink_set_t;
        typedef std::map<AbstractArcSink*, EphemeralArcEnd*> sink_end_map_t;
    public:
        Arc(ArcStyle const& of_style,
            AbstractArcSource *from=NULL,
            AbstractArcSink *to=NULL);
        virtual ~Arc();

        ArcStyle const& style() const;
        AbstractArcSource *source();
        std::set<AbstractArcSink *> sinks(); 

        // ArcSource
        // this is overloaded so that arcs do not participate in layout
        virtual QGraphicsItem* ultimateParent(); // !!! should be private 

        // QGraphicsItem:
        // overloaded for performance
        virtual QPainterPath shape() const;
        // TODO: also override contains?

        virtual QRectF boundingRect() const;
        virtual void paint(QPainter *painter,
                           const QStyleOptionGraphicsItem *option,
                           QWidget *widget = 0);
    protected:
        // itemChanged is used to hook the notification when this arc is
        // actually added to a scene, at which point magic-foo is performed to
        // draw arcs underneath everything else
        virtual QVariant itemChange(GraphicsItemChange change, QVariant const& value);

    public Q_SLOTS:
        void updateLayout();    
        
        void setFrom(AbstractArcSource *from);
        void addTo(AbstractArcSink *to);
        void addPending(AbstractArcSink *to);
        void removeTo(AbstractArcSink *to);
        void promotePending(AbstractArcSink *to);

    protected:
        ArcStyle const& m_style;
        AbstractArcSource *m_source;
        arcsink_set_t m_sinks;
        arcsink_set_t m_pending_sinks;
        sink_end_map_t m_ends;
        QGraphicsPathItem *m_back;
        QGraphicsPathItem *m_front;
        QGraphicsPathItem *m_pending_back;
        QGraphicsPathItem *m_pending_front;
        EphemeralArcEnd *m_ephemeral_end;

        // caches:
        mutable bool m_cached_shape_invalid;
        mutable QPainterPath m_cached_shape;
};

} // namespace liquid

#endif // __LIQUID_ARC_H__

