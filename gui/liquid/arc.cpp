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

#include "arc.h"

#include <cmath>

#include <QPainter>

#include <debug/cauv_debug.h>

#include "arcSink.h"
#include "style.h"
#include "ephemeralArcEnd.h"

using namespace liquid;

Arc::Arc(ArcStyle const& of_style,
         AbstractArcSource *from,
         AbstractArcSink *to)
    : AbstractArcSource(of_style, from, this),
      m_style(of_style),
      m_source(from),
      m_sinks(),
      m_back(new QGraphicsPathItem(this)),
      m_front(new QGraphicsPathItem(this)),
      m_ephemeral_end(new EphemeralArcEnd(of_style)){

    debug() << "Arc()" << this;

    setFlag(ItemHasNoContents);
    
    // this is just cosmetic: should probably do drawing locally...
    m_ephemeral_end->setParentItem(this);
    m_ephemeral_end->setAcceptedMouseButtons(Qt::NoButton);
    
    if(from)
        setFrom(from);
    if(to)
        addTo(to);

    updateLayout();
}

Arc::~Arc(){
    debug() << "~Arc()" << this;
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

void Arc::setFrom(AbstractArcSource *from){
    debug() << "arc::setFrom:" << from;
    if (m_source)
        disconnect(m_source, SIGNAL(geometryChanged()), this, SLOT(updateLayout()));
    m_source = from;
    setParentItem(from);
    connect(from, SIGNAL(geometryChanged()), this, SLOT(updateLayout()));
    updateLayout();
}

void Arc::addTo(AbstractArcSink *to){
    debug() << "Arc::addTo:" << to;
    m_sinks.insert(to);
    connect(to, SIGNAL(geometryChanged()), this, SLOT(updateLayout()));
    connect(to, SIGNAL(disconnected(AbstractArcSink*)),
            this, SLOT(removeTo(AbstractArcSink*)));
    updateLayout();
}

void Arc::removeTo(AbstractArcSink *to){
    debug() << "Arc::removeTo:" << to;
    m_sinks.erase(to);
    updateLayout();
}

QRectF Arc::boundingRect() const{
    return m_back->boundingRect() | m_ephemeral_end->boundingRect();
}

void Arc::paint(QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget){
    Q_UNUSED(painter);
    Q_UNUSED(option);
    Q_UNUSED(widget);
}

void Arc::updateLayout(){
    if(!m_source){
        warning() << "no source!";
        return;
    }

    debug() << "Updating arc layout" << this;

    prepareGeometryChange();

    QPointF start_point = mapFromScene(m_source->scenePos());
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
        m_ephemeral_end->hide();
        foreach(AbstractArcSink* ci, m_sinks){
            path.moveTo(split_point);
            QPointF end_point(mapFromScene(ci->scenePos()));
            QPointF c1(split_point + QPointF(15+std::fabs(end_point.x() - split_point.x())/2, 0));
            QPointF c2(end_point   - QPointF(15+std::fabs(end_point.x() - split_point.x())/2, 0));
            path.cubicTo(c1, c2, end_point);
            //debug() << "f=" << start_point << "s=" << split_point << "t=" << end_point;
            //path.lineTo(end_point);
        }
    }else{
        //path.lineTo(split_point + QPointF(4,0));
        m_ephemeral_end->show();
    }

    m_back->setPath(path);
    m_front->setPath(path);
}

