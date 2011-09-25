#include "multiArcStart.h"
#include "style.h"

using namespace cauv;
using namespace gui;

MultiArcStart::MultiArcStart(ManagedElement const& m, liquid::ArcStyle const& style)
    : QGraphicsObject(),
      ConnectableInterface(),
      ManagedElement(m),
      m_arc(), m_front_line(), m_back_line(), m_style(style){

    setFlag(ItemHasNoContents);
    setFlag(ItemIsMovable);    

    m_arc = new MultiArc(*this, style, this, NULL);
    m_back_line = new QGraphicsLineItem(-m_style.back.start_length,0,0,0,this);
    m_front_line = new QGraphicsLineItem(-m_style.front.start_length,0,0,0,this);
    
    QLinearGradient back_gradient(QPointF(-m_style.back.start_length,0), connectionPoint());
    back_gradient.setColorAt(0, m_style.back.start_col);
    back_gradient.setColorAt(1, m_style.back.col);
    m_back_line->setPen(QPen(
        QBrush(back_gradient), m_style.back.thickness, Qt::SolidLine, Qt::FlatCap
    ));
    
    QLinearGradient front_gradient(QPointF(-m_style.front.start_length,0), connectionPoint());
    front_gradient.setColorAt(0, m_style.front.start_col);
    front_gradient.setColorAt(1, m_style.front.col);
    m_front_line->setPen(QPen(
        QBrush(front_gradient),m_style.front.thickness, Qt::SolidLine, Qt::FlatCap
    ));
    
    connect(this, SIGNAL(xChanged()), this, SIGNAL(boundriesChanged()));
    connect(this, SIGNAL(yChanged()), this, SIGNAL(boundriesChanged()));
}


MultiArc* MultiArcStart::arc() const{
    // !!! do not delete 'this' while m_arc (a child of this) is being
    // used
    return m_arc;
}

liquid::ArcStyle const& MultiArcStart::style() const{
    return m_style;
}

// QGraphicsItem required:
QRectF MultiArcStart::boundingRect() const{
    // otherwise we can't receive mouse events and be movable
    return m_back_line->boundingRect();
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
    return QPointF(0, 0);
}
