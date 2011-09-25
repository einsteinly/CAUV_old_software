#include "arc.h"

#include <cmath>

#include "arcSink.h"
#include "style.h"

using namespace liquid;

Arc::Arc(ArcStyle const& of_style,
         AbstractArcSource *from,
         AbstractArcSink *to)
    : AbstractArcSource(of_style, from, this),
      m_style(of_style),
      m_source(from),
      m_sinks(),
      m_back(new QGraphicsPathItem(this)),
      m_front(new QGraphicsPathItem(this)){
    
    setParentItem(from);
    setFlag(ItemHasNoContents);

    connect(from, SIGNAL(geometryChanged()), this, SLOT(updateLayout()));

    if(to)
        addTo(to);

    updateLayout();
}

ArcStyle const& Arc::style() const{
    return m_style;
}

void *Arc::source(){
    return m_source;
}

std::set<AbstractArcSink *> Arc::sinks(){
    return m_sinks;
}

void Arc::addTo(AbstractArcSink *to){
    m_sinks.insert(to);
    // !!! TODO: also connect signal to act on disconnection
    connect(to, SIGNAL(geometryChanged()), this, SLOT(updateLayout()));
    updateLayout();
}

void Arc::removeTo(AbstractArcSink *to){
    // !!! TODO
    m_sinks.erase(to);
}

QRectF Arc::boundingRect() const{
    return m_back->boundingRect();
}

void Arc::updateLayout(){
    prepareGeometryChange();
    
    // we draw in m_source's coordinate system (it is this's parent)
    QPointF start_point = m_source->pos();
    QPointF split_point = start_point + QPointF(8,0);
    
    // back style:
    m_back->setPen(QPen(
        QBrush(m_style.back.col), m_style.back.thickness, Qt::SolidLine, Qt::FlatCap
    ));

    // front style:
    m_front->setPen(QPen(
        QBrush(m_style.front.col), m_style.front.thickness, Qt::SolidLine, Qt::FlatCap
    ));

    QPainterPath path(start_point);
    path.lineTo(split_point);

    if(m_sinks.size()){
        foreach(AbstractArcSink* ci, m_sinks){
            path.moveTo(split_point);
            // mapToItem maps connection point in 'to' object into m_source's
            // coordinates 
            QPointF end_point(ci->mapToItem(m_source, ci->pos()));
            QPointF c1(split_point + QPointF(15+std::fabs(end_point.x() - split_point.x())/2, 0));
            QPointF c2(end_point   - QPointF(15+std::fabs(end_point.x() - split_point.x())/2, 0));
            path.cubicTo(c1, c2, end_point);
            //debug() << "f=" << start_point << "s=" << split_point << "t=" << end_point;
            //path.lineTo(end_point);
        }
    }else{
        path.lineTo(split_point + QPointF(4,0));
    }

    m_back->setPath(path);
    m_front->setPath(path);
}

