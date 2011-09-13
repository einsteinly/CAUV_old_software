#include "multiArcEnd.h"

#include <cassert>

#include <QPolygonF>
#include <QPropertyAnimation>
#include <QGraphicsScene>

#include "multiArc.h"

using namespace cauv;
using namespace gui;

const qreal MultiArcEnd::Length = 14;
const qreal MultiArcEnd::Base_Thickness = 11;
const qreal MultiArcEnd::Tip_Thickness = 3;
const QColor MultiArcEnd::Tip_Colour = QColor(80, 235, 200, 255);
const QColor MultiArcEnd::Pressed_Colour = QColor(255, 255, 255, 200);


MultiArcEnd::MultiArcEnd(MultiArc* arc, bool ephemeral)
    : m_arc(arc), m_poly(), m_ephemeral(ephemeral){
    assert(arc);
    setParentItem(arc);
    arc->addTo(this);

    QPolygonF shape;
    shape << QPointF(0, 0);
    shape << QPointF(Length, (Base_Thickness - Tip_Thickness) / 2);
    shape << QPointF(Length, (Base_Thickness + Tip_Thickness) / 2);
    shape << QPointF(0, Base_Thickness);
    m_poly = new QGraphicsPolygonItem(shape, this);
    
    if(ephemeral)
        setPolyGradient(Pressed_Colour);
    else
        setPolyGradient(Tip_Colour);
    
    setFlag(ItemIsMovable);
    connect(this, SIGNAL(xChanged()), this, SIGNAL(boundriesChanged()));
    connect(this, SIGNAL(yChanged()), this, SIGNAL(boundriesChanged()));
    Q_EMIT boundriesChanged();
}


void MultiArcEnd::mousePressEvent(QGraphicsSceneMouseEvent *event){
    Q_UNUSED(event);
    setPolyGradient(Pressed_Colour);
    QGraphicsObject::mousePressEvent(event);
}

void MultiArcEnd::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    Q_UNUSED(event);
    if(m_ephemeral){
        QPropertyAnimation *fadeOut = new QPropertyAnimation(this, "opacity");
        fadeOut->setEndValue(0);
        fadeOut->setDuration(150);
        connect(fadeOut, SIGNAL(finished()), this, SLOT(removeFromScene()));
        fadeOut->start();
    }else{
        setPolyGradient(Tip_Colour);
    }
    QGraphicsObject::mouseReleaseEvent(event);
}

// QGraphicsItem required:
QRectF MultiArcEnd::boundingRect() const{
    return QRectF(0,0,Length,std::max(Base_Thickness, Tip_Thickness));
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
    return QPointF(0, Base_Thickness/2);
}

void MultiArcEnd::removeFromScene(){
    // time to die
    hide();
    m_arc->removeTo(this);
    scene()->removeItem(this);
    deleteLater();
}

void MultiArcEnd::setPolyGradient(QColor tip_colour){
    QLinearGradient gradient(
        connectionPoint(),
        QPointF(Length,std::max(Base_Thickness, Tip_Thickness)/2)
    );

    gradient.setColorAt(0, m_arc->endColour());
    gradient.setColorAt(1, tip_colour);
    gradient.setColorAt(1, tip_colour);

    QBrush brush(gradient);

    m_poly->setBrush(brush);
    m_poly->setPen(QPen(Qt::NoPen));
}

