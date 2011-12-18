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

#include "ephemeralArcEnd.h"

#include <QPropertyAnimation>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>

#include <debug/cauv_debug.h>

#include "style.h"

using namespace liquid;

static QGraphicsPolygonItem* endArrow(liquid::ArcStyle::SingleArcStyle const& s, QGraphicsItem* parent){
    QPolygonF shape;
    shape << QPointF(0, -s.thickness/2);
    shape << QPointF(s.end_base_offset, -s.thickness/2);
    shape << QPointF(s.end_base_offset, -s.end_base_thickness/2);    
    shape << QPointF(s.end_length,      -s.end_tip_thickness /2);
    shape << QPointF(s.end_length,       s.end_tip_thickness /2);
    shape << QPointF(s.end_base_offset,  s.end_base_thickness/2);
    shape << QPointF(s.end_base_offset,  s.thickness/2);
    shape << QPointF(0, s.thickness/2);
    return new QGraphicsPolygonItem(shape, parent);
}

EphemeralArcEnd::EphemeralArcEnd(QGraphicsItem* parent, ArcStyle const& of_style, bool cosmetic)
    : AbstractArcSink(parent),
      m_back_poly(endArrow(of_style.back, this)),
      m_front_poly(endArrow(of_style.front, this)),
      m_style(of_style){
    setFill(false);
    setFlag(ItemIsMovable);
    if(cosmetic)
        setAcceptedMouseButtons(Qt::NoButton);
}

EphemeralArcEnd::~EphemeralArcEnd(){
    debug() << "~EphemeralArcEnd()";
}

void EphemeralArcEnd::mousePressEvent(QGraphicsSceneMouseEvent *e){
    setFill(true);
    mouseMoveEvent(e);
}

void EphemeralArcEnd::mouseMoveEvent(QGraphicsSceneMouseEvent *e){
    // !!! relying that e is in parent coordinates (ie forwarded by
    // AbstractArcSouce)
    setPos(e->pos());
}

void EphemeralArcEnd::mouseReleaseEvent(QGraphicsSceneMouseEvent *e){
    Q_UNUSED(e);    
    debug() << "EphemeralArcEnd::mouseReleaseEvent";
    QPropertyAnimation *fadeOut = new QPropertyAnimation(this, "opacity");
    m_back_poly->setOpacity(0.5);
    fadeOut->setEndValue(0);
    fadeOut->setDuration(100);
    connect(fadeOut, SIGNAL(finished()), this, SLOT(removeFromScene()));
    fadeOut->start();
}

QRectF EphemeralArcEnd::boundingRect() const{
    return QRectF(
        0, -m_style.back.end_base_thickness/2,
        m_style.back.end_length, m_style.back.end_base_thickness
    );
}

void EphemeralArcEnd::paint(QPainter *painter,
           const QStyleOptionGraphicsItem *opt,
           QWidget *widget){
    Q_UNUSED(painter);
    Q_UNUSED(opt);
    Q_UNUSED(widget);
}

void EphemeralArcEnd::removeFromScene(){
    hide();
    debug() << "EphemeralArcEnd::removeFromScene:" << this;
    Q_EMIT(disconnected(this));
    scene()->removeItem(this);
    deleteLater();
}

void EphemeralArcEnd::setFill(bool pressed){
    // !!! back doesn't change
    m_back_poly->setBrush(m_style.back.col);

    // front does:
    if(pressed){
        QLinearGradient gradient(
            QPointF(0,0), QPointF(m_style.front.end_length, 0)
        );
        gradient.setColorAt(0, m_style.front.col);
        gradient.setColorAt(1, m_style.front.tip_hl_col);
        m_front_poly->setBrush(QBrush(gradient));
    }else{
        m_front_poly->setBrush(m_style.front.col);
    }

    m_back_poly->setPen(QPen(Qt::NoPen));
    m_front_poly->setPen(QPen(Qt::NoPen));
}
