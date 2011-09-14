#include "multiArcStart.h"

using namespace cauv;
using namespace gui;

const qreal MultiArcStart::Length = 12;
const qreal MultiArcStart::Thickness = 4;
const QColor MultiArcStart::Lead_In_Colour = QColor(134, 217, 241, 0);

MultiArcStart::MultiArcStart()
    : m_arc(), m_line(){
    m_arc = new MultiArc(this, NULL);
    m_line = new QGraphicsLineItem(0, Thickness/2, Length, Thickness/2, this);
    
    QLinearGradient line_gradient(QPointF(0,Thickness/2), connectionPoint());
    line_gradient.setColorAt(0, Lead_In_Colour);
    line_gradient.setColorAt(1, m_arc->startColour());
    QBrush brush(line_gradient);
    
    m_line->setPen(QPen(brush, Thickness));
    
    setFlag(ItemIsMovable);
    connect(this, SIGNAL(xChanged()), this, SIGNAL(boundriesChanged()));
    connect(this, SIGNAL(yChanged()), this, SIGNAL(boundriesChanged()));
}


MultiArc* MultiArcStart::arc(){
    // !!! do not delete 'this' while m_arc (a child of this) is being
    // used
    return m_arc;
}

// QGraphicsItem required:
QRectF MultiArcStart::boundingRect() const{
    return QRectF(0, 0, Length, Thickness) | childrenBoundingRect();
}
void MultiArcStart::paint(QPainter *painter,
                          const QStyleOptionGraphicsItem *opt,
                          QWidget *widget){
    Q_UNUSED(opt);
    Q_UNUSED(widget);
    Q_UNUSED(painter);
    // drawing is all done by children
}

// ConnectableInterface:
QGraphicsObject* MultiArcStart::asQGraphicsObject(){
    return this;
}
QPointF MultiArcStart::connectionPoint(){
    return QPointF(Length, Thickness/2);
}
