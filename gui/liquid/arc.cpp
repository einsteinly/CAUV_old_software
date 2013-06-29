/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "arc.h"

#include <cmath>
#include <cassert>

#include <debug/cauv_debug.h>

#include <QPainter>

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
    
    #ifndef CAUV_DEBUG_DRAW_LAYOUT
    setFlag(ItemHasNoContents);
    #endif

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

    setBoundingRegionGranularity(0.1); // 10x10px regions

    m_back->setBoundingRegionGranularity(0.1);
    m_front->setBoundingRegionGranularity(0.1);
    m_pending_back->setBoundingRegionGranularity(0.1);
    m_pending_front->setBoundingRegionGranularity(0.1);

    m_ephemeral_end->setPos(-1,0);

    setZValue(-100);
    
    // Arcs are big, and caching them is about the same speed
    setCacheMode(NoCache);
    m_back->setCacheMode(NoCache);
    m_front->setCacheMode(NoCache);
    m_pending_back->setCacheMode(NoCache);
    m_pending_front->setCacheMode(NoCache);

#ifdef QT_PROFILE_GRAPHICSSCENE
    setProfileName("liquid::Arc");
    m_back->setProfileName("liquid::Arc::back");
    m_front->setProfileName("liquid::Arc::front");
    m_pending_back->setProfileName("liquid::Arc::pending_back");
    m_pending_front->setProfileName("liquid::Arc::pending_front");
    m_ephemeral_end->setProfileName("liquid::Arc::ephemeral_end");
#endif // def QT_PROFILE_GRAPHICSSCENE
}

Arc::~Arc(){
    //debug() << "arc" << this << "destroyed. (" << m_source << "->" << m_sinks.size() << "sinks)";
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

QGraphicsItem* Arc::ultimateParent(){
    return nullptr;
}

void Arc::setFrom(AbstractArcSource *from){
    assert(from);
    if (m_source)
        disconnect(m_source, SIGNAL(geometryChanged()), this, SLOT(updateLayout()));
    m_source = from;
    
    if(!scene()){
        // temporarily use from as the parent item: when from is added to a
        // scene, we can re-parent the arc to a top-level item, but at this
        // point a scene might not even exist (!)
        // it's necessary to do this in setFrom not in the constructor, because
        // passing 0 for `from` in the constructor is allowed (to ease
        // construction order-dependence of arcs and the things arcs connect
        // to)
        setParentItem(from);
    }

    setSourceDelegate(from->sourceDelegate());
    connect(from, SIGNAL(geometryChanged()), this, SLOT(updateLayout()));
    updateLayout();
}

void Arc::addTo(AbstractArcSink *to){
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
    disconnect(to, SIGNAL(geometryChanged()), this, SLOT(updateLayout()));
    disconnect(to, SIGNAL(disconnected(AbstractArcSink*)),
               this, SLOT(removeTo(AbstractArcSink*)));
    m_sinks.erase(to);
    m_pending_sinks.erase(to);
    m_ends[to]->deleteLater();
    m_ends.erase(to);
    if(scene())
        updateLayout();
}

void Arc::promotePending(AbstractArcSink *to){
    m_sinks.insert(to);
    m_pending_sinks.erase(to);
    updateLayout();
}

QPainterPath Arc::shape() const{
    if(m_cached_shape_invalid){
        m_cached_shape = m_back->shape() | m_pending_back->shape();
        for(auto const & end : m_ends)
            m_cached_shape |= end.second->shape().translated(end.second->pos());
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

QVariant Arc::itemChange(GraphicsItemChange change, QVariant const& value){
    if(change == ItemSceneHasChanged && scene())
        setParentItem(nullptr);
    return AbstractArcSourceInternal::itemChange(change, value);
}

void Arc::updateLayout(){
    if(!m_source){
        warning() << "no source!";
        return;
    }else if(!scene()){
        warning() << "no scene!";
        return;
    }
    m_cached_shape_invalid = true;
    
    prepareGeometryChange();

    setPos(m_source->scenePos());

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

