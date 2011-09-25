#include "ephemeralArcEnd.h"

#include <QPropertyAnimation>
#include <QGraphicsScene>

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

EphemeralArcEnd::EphemeralArcEnd(ArcStyle const& of_style)
    : AbstractArcSink(),
      m_back_poly(endArrow(m_style.back, this)),
      m_front_poly(endArrow(m_style.front, this)),
      m_style(of_style){
    setFill(false);
    setFlag(ItemIsMovable);
    connect(this, SIGNAL(xChanged()), this, SIGNAL(geometryChanged));
    connect(this, SIGNAL(yChanged()), this, SIGNAL(geometryChanged));
}

void EphemeralArcEnd::mousePressEvent(QGraphicsSceneMouseEvent *event){
    setFill(true);
    QGraphicsObject::mousePressEvent(event);
}

void EphemeralArcEnd::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    QPropertyAnimation *fadeOut = new QPropertyAnimation(this, "opacity");
    m_back_poly->setOpacity(0.5);
    fadeOut->setEndValue(0);
    fadeOut->setDuration(100);
    connect(fadeOut, SIGNAL(finished()), this, SLOT(removeFromScene()));
    fadeOut->start();
    QGraphicsObject::mouseReleaseEvent(event);
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
    Q_EMIT(disconnected());
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
