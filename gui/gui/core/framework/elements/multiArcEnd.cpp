#include "multiArcEnd.h"

#include <cassert>

#include <QPolygonF>
#include <QPropertyAnimation>
#include <QGraphicsScene>

#include "multiArc.h"
#include "style.h"

using namespace cauv;
using namespace gui;

static QGraphicsPolygonItem* endArrow(ArcStyle::SingleArcStyle const& s, QGraphicsItem* parent){
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

MultiArcEnd::MultiArcEnd(MultiArc* arc, bool ephemeral)
    : m_arc(arc),
      m_back_poly(),
      m_front_poly(),
      m_ephemeral(ephemeral),
      m_style(arc->style()){
    
    m_back_poly = endArrow(m_style.back, this);
    m_front_poly = endArrow(m_style.front, this);
    
    setFill(ephemeral);
    setFlag(ItemIsMovable);
    
    // set parent and add only after setting up geometry
    setParentItem(arc);
    arc->addTo(this);

    connect(this, SIGNAL(xChanged()), this, SIGNAL(boundriesChanged()));
    connect(this, SIGNAL(yChanged()), this, SIGNAL(boundriesChanged()));
    Q_EMIT boundriesChanged();
}


void MultiArcEnd::mousePressEvent(QGraphicsSceneMouseEvent *event){
    setFill(true);
    QGraphicsObject::mousePressEvent(event);
}

void MultiArcEnd::mouseMoveEvent(QGraphicsSceneMouseEvent *event){
    QGraphicsObject::mouseMoveEvent(event);
}

void MultiArcEnd::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    if(m_ephemeral){
        QPropertyAnimation *fadeOut = new QPropertyAnimation(this, "opacity");
        m_back_poly->setOpacity(0.5);
        fadeOut->setEndValue(0);
        fadeOut->setDuration(100);
        connect(fadeOut, SIGNAL(finished()), this, SLOT(removeFromScene()));
        fadeOut->start();
    }else{
        setFill(false);
    }
    QGraphicsObject::mouseReleaseEvent(event);
}

// QGraphicsItem required:
QRectF MultiArcEnd::boundingRect() const{
    return QRectF(
        0, -m_style.back.end_base_thickness/2,
        m_style.back.end_length, m_style.back.end_base_thickness
    );
}

void MultiArcEnd::paint(QPainter *painter,
                        const QStyleOptionGraphicsItem *opt,
                        QWidget *widget){
    Q_UNUSED(painter);
    Q_UNUSED(opt);
    Q_UNUSED(widget);
}


// ConnectableInterface:
QGraphicsObject* MultiArcEnd::asQGraphicsObject(){
    return this;
}

QPointF MultiArcEnd::connectionPoint(){
    return QPointF(0, 0);
}

void MultiArcEnd::removeFromScene(){
    // time to die
    hide();
    m_arc->removeTo(this);
    scene()->removeItem(this);
    deleteLater();
}

void MultiArcEnd::setFill(bool pressed){
    // !!! back doesn't change
    m_back_poly->setBrush(m_style.back.col);

    // front does:
    if(pressed){
        QLinearGradient gradient(
            connectionPoint(), QPointF(m_style.front.end_length, 0)
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

