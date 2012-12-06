/* Copyright 2012 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Steven Ogborne  steve@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "interactionArea.h"

using namespace liquid;

InteractionArea::InteractionArea(QGraphicsItem* delegate, float add_border)
    : m_outline(), m_add_border(add_border), m_delegate(delegate){
    freezeDelegateShape();
}

void InteractionArea::freezeDelegateShape(){
    if(m_delegate){
        prepareGeometryChange();

        // !!! FIXME should use a proper outset operation!
        QPainterPath p = m_delegate->shape()
                            .united(m_delegate->shape().translated( m_add_border, 0))
                            .united(m_delegate->shape().translated(-m_add_border, 0));
        m_outline = p.united(p.translated(0,  m_add_border));
        m_outline = m_outline.united(p.translated(0, -m_add_border));
        
        setAcceptsHoverEvents(m_delegate->acceptsHoverEvents());
    }
}

void InteractionArea::mousePressEvent(QGraphicsSceneMouseEvent *e){
    //m_delegate->mousePressEvent(e);
}

void InteractionArea::mouseMoveEvent(QGraphicsSceneMouseEvent *e){
    //m_delegate->mouseMoveEvent(e);
}

void InteractionArea::mouseReleaseEvent(QGraphicsSceneMouseEvent *e){
    //m_delegate->mouseReleaseEvent(e);
}

QPainterPath InteractionArea::shape() const{
    return m_outline;
}

QRectF InteractionArea::boundingRect() const{
    return m_outline.boundingRect();
}

void InteractionArea::paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget *){
    // paint nothing!
}

bool InteractionArea::contains(QPointF const& point) const{
    return m_outline.contains(point);
}

