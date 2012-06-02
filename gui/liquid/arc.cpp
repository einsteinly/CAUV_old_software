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
    : AbstractArcSourceInternal(of_style, from, this),
      LayoutItems(this),
      m_style(of_style),
      m_source(from),
      m_sinks(),
      m_pending_sinks(),
      m_ends(),
      m_back(new QGraphicsPathItem(this)),
      m_front(new QGraphicsPathItem(this)),
      m_pending_back(new QGraphicsPathItem(this)),
      m_pending_front(new QGraphicsPathItem(this)),
      m_ephemeral_end(new EphemeralArcEnd(this, of_style, true)),
      m_cached_shape_invalid(true),
      m_cached_shape(){

    debug(7) << "Arc()" << this;
    
    #ifndef CAUV_DEBUG_DRAW_LAYOUT
    setFlag(ItemHasNoContents);
    #endif
    setCacheMode(DeviceCoordinateCache);
    setBoundingRegionGranularity(0.1); // 10x10px regions
    
    if(from)
        setFrom(from);
    if(to)
        addTo(to);
    
    if(from)
        updateLayout();
    
    // styles
    m_back->setPen(QPen(
        QBrush(m_style.back.col), m_style.back.thickness, Qt::SolidLine, Qt::FlatCap
    ));
    m_front->setPen(QPen(
        QBrush(m_style.front.col), m_style.front.thickness, Qt::SolidLine, Qt::FlatCap
    ));
    m_pending_back->setPen(QPen(
        QBrush(m_style.back.col, Qt::Dense4Pattern), m_style.back.thickness, Qt::SolidLine, Qt::FlatCap
    ));
    m_pending_front->setPen(QPen(
        QBrush(m_style.front.col, Qt::Dense4Pattern), m_style.front.thickness, Qt::SolidLine, Qt::FlatCap
    ));
}

Arc::~Arc(){
    debug(7) << "~Arc()" << this;
    LayoutItems::unRegisterConnection(this);
}

ArcStyle const& Arc::style() const{
    return m_style;
}

AbstractArcSource *Arc::source(){
    return m_source;
}

std::set<AbstractArcSink *> Arc::sinks(){
    return m_sinks;
}

void Arc::setFrom(AbstractArcSource *from){
    debug(7) << "arc::setFrom:" << from;
    if (m_source)
        disconnect(m_source, SIGNAL(geometryChanged()), this, SLOT(updateLayout()));
    m_source = from;
    setSourceDelegate(from->sourceDelegate());
    setParentItem(from);
    connect(from, SIGNAL(geometryChanged()), this, SLOT(updateLayout()));
    updateLayout();
}

void Arc::addTo(AbstractArcSink *to){
    debug(3) << "Arc::addTo:" << to;
    if(m_pending_sinks.count(to)){
        promotePending(to);
    }else if(!m_sinks.count(to)){
        m_sinks.insert(to);
        m_ends[to] = new EphemeralArcEnd(this, m_style, true);
        connect(to, SIGNAL(geometryChanged()), this, SLOT(updateLayout()));
        connect(to, SIGNAL(disconnected(AbstractArcSink*)),
                this, SLOT(removeTo(AbstractArcSink*)));
        updateLayout();
    }else{
        warning() << "will not connect to same sink twice";
    }
}

void Arc::addPending(AbstractArcSink *to){
    debug(3) << "Arc::addPending:" << to;
    if(!m_pending_sinks.count(to) && !m_sinks.count(to)){
        m_pending_sinks.insert(to);
        m_ends[to] = new EphemeralArcEnd(this, m_style, true);
        connect(to, SIGNAL(geometryChanged()), this, SLOT(updateLayout()));
        connect(to, SIGNAL(disconnected(AbstractArcSink*)),
                this, SLOT(removeTo(AbstractArcSink*)));
        updateLayout();
    }else{
        warning() << "will not connect to same pending sink twice";
    }
}

void Arc::removeTo(AbstractArcSink *to){
    debug(3) << "Arc::removeTo:" << to;
    disconnect(to, SIGNAL(geometryChanged()), this, SLOT(updateLayout()));
    disconnect(to, SIGNAL(disconnected(AbstractArcSink*)),
               this, SLOT(removeTo(AbstractArcSink*)));
    m_sinks.erase(to);
    m_pending_sinks.erase(to);
    m_ends[to]->deleteLater();
    m_ends.erase(to);
    updateLayout();
}

void Arc::promotePending(AbstractArcSink *to){
    debug(3) << "Arc::promotePending:" << to;
    m_sinks.insert(to);
    m_pending_sinks.erase(to);
    updateLayout();
}

QPainterPath Arc::shape() const{
    if(m_cached_shape_invalid){
        debug(5) << "cached shape invalid, updating...";
        m_cached_shape = m_back->shape() | m_pending_back->shape();
        for(sink_end_map_t::const_iterator i = m_ends.begin(); i != m_ends.end(); i++)
            m_cached_shape |= i->second->shape().translated(i->second->pos());
        m_cached_shape_invalid = false;
    }
    return m_cached_shape;
}

QRectF Arc::boundingRect() const{
    /*QRectF r = m_back->boundingRect() |
               m_pending_back->boundingRect() |
               m_ephemeral_end->boundingRect();
    sink_end_map_t::const_iterator i;
    for(i = m_ends.begin(); i != m_ends.end(); i++)
        r |= i->second->boundingRect().translated(i->second->pos());
    return r;*/

    // !!! Qt bug workaround: the boudning rectangles of QPainterPaths with
    // cubic curves in are over-estimates (by 50%), at least in our usage: this
    // returns a much tighter bounding rect, which is better for performance
    return shape().boundingRect();
}

void Arc::paint(QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget){
    Q_UNUSED(painter);
    Q_UNUSED(option);
    Q_UNUSED(widget);
    /*painter->setBrush(QBrush(QColor(200,20,20,24)));
    painter->setPen(QPen(Qt::NoPen));
    painter->drawPath(shape());
    m_back->setFlag(ItemStacksBehindParent);
    m_front->setFlag(ItemStacksBehindParent);
    sink_end_map_t::const_iterator i;    
    for(i = m_ends.begin(); i != m_ends.end(); i++)
        i->second->setFlag(ItemStacksBehindParent);
    */
    #ifdef CAUV_DEBUG_DRAW_LAYOUT    
    painter->setPen(QPen(QColor(200,20,20,24)));
    painter->setBrush(Qt::NoBrush);
    painter->drawRect(boundingRect());
    painter->setPen(QPen(QColor(20,120,20,24)));
    painter->drawRect(m_back->boundingRect());
    painter->setPen(QPen(QColor(20,20,220,24)));
    painter->drawRect(m_ephemeral_end->boundingRect());
    painter->setPen(QPen(QColor(120,20,220,24)));
    sink_end_map_t::const_iterator i;
    for(i = m_ends.begin(); i != m_ends.end(); i++)
       painter->drawRect(i->second->boundingRect().translated(i->second->pos()));
    #endif // def CAUV_DEBUG_DRAW_LAYOUT
}

// !!! TODO: the updateLayout slot should only set a dirty flag on the layout,
// which causes re-layout the next time paint() contains() shape() or something
// gets called - at the moment updateLayout() can be called many times for a
// single paint, and it's a pretty big performance bottleneck
void Arc::updateLayout(){
    if(!m_source){
        warning() << "no source!";
        return;
    }

    debug(7) << "Updating arc layout" << this;
    m_cached_shape_invalid = true;

    prepareGeometryChange();

    QPointF start_point = mapFromScene(m_source->scenePos());
    QPointF split_point = start_point + QPointF(8,0);

    QPainterPath path(start_point);
    QPainterPath ppth(split_point);
    path.lineTo(split_point);

    if(m_sinks.size() || m_pending_sinks.size()){
        m_ephemeral_end->hide();
        foreach(AbstractArcSink* ci, m_sinks){
            path.moveTo(split_point);
            QPointF end_point(mapFromScene(ci->scenePos()).toPoint());
            QPointF c1(split_point + QPointF(15+std::fabs(end_point.x() - split_point.x())/2, 0));
            QPointF c2(end_point   - QPointF(15+std::fabs(end_point.x() - split_point.x())/2, 0));
            path.cubicTo(c1, c2, end_point);
            // return along the same path: non-closed paths draw okay, but have
            // broken shape() functions
            path.cubicTo(c2, c1, split_point);
        }
        foreach(AbstractArcSink* ci, m_pending_sinks){
            ppth.moveTo(split_point);
            QPointF end_point(mapFromScene(ci->scenePos()).toPoint());
            QPointF c1(split_point + QPointF(15+std::fabs(end_point.x() - split_point.x())/2, 0));
            QPointF c2(end_point   - QPointF(15+std::fabs(end_point.x() - split_point.x())/2, 0));
            ppth.cubicTo(c1, c2, end_point);
            ppth.cubicTo(c2, c1, split_point);
        }
    }else{
        //path.lineTo(split_point + QPointF(4,0));
        m_ephemeral_end->show();
    }

    path.lineTo(start_point);
    
    prepareGeometryChange();
    m_back->setPath(path);
    m_front->setPath(path);
    m_pending_back->setPath(ppth);
    m_pending_front->setPath(ppth);

    foreach(AbstractArcSink* ci, m_sinks)
        m_ends[ci]->setPos(mapFromScene(ci->scenePos()));
    foreach(AbstractArcSink* ci, m_pending_sinks)
        m_ends[ci]->setPos(mapFromScene(ci->scenePos()));
}

