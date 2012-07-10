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

#ifndef __LIQUID_EPHEMERAL_ARC_END__
#define __LIQUID_EPHEMERAL_ARC_END__

#include "arcSink.h"

namespace liquid{

struct ArcStyle;

class EphemeralArcEnd: public AbstractArcSink{
    Q_OBJECT
    public:
        EphemeralArcEnd(QGraphicsItem* parent, ArcStyle const& of_style, bool cosmetic=false);
        virtual ~EphemeralArcEnd();
    
        // these are forwarded from AbstractArcSink: the event object is
        // in the *parent* coordinate system
        void mousePressEvent(QGraphicsSceneMouseEvent *e); 
        void mouseMoveEvent(QGraphicsSceneMouseEvent *e); 
        void mouseReleaseEvent(QGraphicsSceneMouseEvent *e); 

        QRectF boundingRect() const; 
        void paint(QPainter *painter,
                   const QStyleOptionGraphicsItem *opt,
                   QWidget *widget);
        virtual bool contains(QPointF const& point) const;
        
        virtual bool willAcceptConnection(ArcSourceDelegate*, AbstractArcSink*){return false;}
        virtual void doPresentHighlight(qreal){}
        virtual ConnectionStatus doAcceptConnection(ArcSourceDelegate*, AbstractArcSink*){return Rejected;}

    Q_SIGNALS:
        void disconnected(AbstractArcSink*);

    protected Q_SLOTS:
        void removeFromScene();

    protected:
        void setFill(bool pressed);
        
        QGraphicsPolygonItem *m_back_poly;
        QGraphicsPolygonItem *m_front_poly;
        
        ArcStyle const& m_style;
};

} // namespace liquid

#endif // ndef __LIQUID_EPHEMERAL_ARC_END__
