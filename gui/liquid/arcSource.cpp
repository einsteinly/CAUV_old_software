/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "arcSource.h"

#include <cmath>
#include <limits>

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsScene>
#include <QApplication>

#include <utility/qt_streamops.h>

#include "arc.h"
#include "arcSink.h"

#define CAUV_DEBUG_COMPAT
#include <debug/cauv_debug.h>

using namespace liquid;
using namespace liquid::_;

// - AbstractArcSourceInternal
AbstractArcSourceInternal::AbstractArcSourceInternal(ArcStyle const& of_style,
                                     ArcSourceDelegate* sourceDelegate,
                                     Arc* arc)
    : QGraphicsObject(),
      ArcSourceDelegate(),
      m_style(of_style),
      m_arc(arc),
      m_sourceDelegate(sourceDelegate),
      m_ephemeral_sink(nullptr){
    debug(7) << "AbstractArcSourceInternal(delegate="<<sourceDelegate<<"): " << this;
    // !!! TODO: instead of signals we can use ItemScenePositionHasChanged notifications
    connect(this, SIGNAL(xChanged()), this, SIGNAL(geometryChanged()));
    connect(this, SIGNAL(yChanged()), this, SIGNAL(geometryChanged()));
}
AbstractArcSourceInternal::~AbstractArcSourceInternal(){
    debug(7) << "~AbstractArcSourceInternal()" << this;
}

Arc* AbstractArcSourceInternal::arc() const{
    return m_arc;
}

ArcStyle const& AbstractArcSourceInternal::style() const{
    return m_style;
}

void AbstractArcSourceInternal::setSourceDelegate(ArcSourceDelegate *sourceDelegate){
    m_sourceDelegate = sourceDelegate;
}
ArcSourceDelegate* AbstractArcSourceInternal::sourceDelegate() const{
    return m_sourceDelegate;
}


void AbstractArcSourceInternal::setParentItem(QGraphicsItem* item){
    disconnectParentSignals(parentItem());
    connectParentSignals(item);
    QGraphicsObject::setParentItem(item);
}

QGraphicsItem* AbstractArcSourceInternal::ultimateParent(){
    QGraphicsItem* last_parent = this;
    QGraphicsItem* parent = nullptr;
    LayoutItem* as_layoutitem = nullptr;
    while((parent = last_parent->parentItem())){
        as_layoutitem = dynamic_cast<LayoutItem*>(parent);
        if(as_layoutitem)
            return as_layoutitem->ultimateParent();
        else
            last_parent = parent;
    }
    return last_parent;
}

ConnectionSink::ConnectionStatus AbstractArcSourceInternal::connectTo(AbstractArcSink* sink){
    const ConnectionSink::ConnectionStatus status = sink->doAcceptConnection(m_sourceDelegate, sink);
    if(status == ConnectionSink::Accepted){
        m_arc->addTo(sink);
    }else if(status == ConnectionSink::Pending){
        m_arc->addPending(sink);
    }
    return status;
}


void AbstractArcSourceInternal::mousePressEvent(QGraphicsSceneMouseEvent *e){
    debug(5) << "AbstractArcSourceInternal::mousePressEvent";
    if(e->button() & Qt::LeftButton){
        if(m_ephemeral_sink)
            error() << "unmatched mousePressEvent";
        m_ephemeral_sink = newArcEnd();
        m_ephemeral_sink->setParentItem(this);

        m_arc->addTo(m_ephemeral_sink);
        // in item (this) coordinates (this is now parent of new end):
        scene()->sendEvent(m_ephemeral_sink, e);
        // !!! TODO: check this
        QGraphicsObject::mousePressEvent(e); 
        e->accept();
    }else{
        e->ignore();
    }
}

void AbstractArcSourceInternal::mouseMoveEvent(QGraphicsSceneMouseEvent *e){
    debug(5) << "AbstractArcSourceInternal::mouseMoveEvent";
    if(m_ephemeral_sink){
        checkAndHighlightSinks(e->scenePos());
        scene()->sendEvent(m_ephemeral_sink, e);
    }else{
        e->ignore();
    }
}

void AbstractArcSourceInternal::mouseReleaseEvent(QGraphicsSceneMouseEvent *e){
    debug(5) << "AbstractArcSourceInternal::mouseReleaseEvent";
    if(e->button() & Qt::LeftButton){
        removeHighlights();
        QPointF ds(8,8);
        QRectF near_field(e->scenePos() - ds, e->scenePos() + ds);
        QList<QGraphicsItem *> items_at_drop = scene()->items(near_field, Qt::IntersectsItemShape);
        AbstractArcSink* sink;
        // only drop onto the closest possible sink (by manhattan length)
        float closest_manhattan_length = std::numeric_limits<float>::max();
        AbstractArcSink *closest = nullptr;
        foreach(QGraphicsItem* item, items_at_drop){
            if((sink = dynamic_cast<AbstractArcSink*>(item)) && sink->willAcceptConnection(m_sourceDelegate, sink)){
                const float ml = (sink->scenePos() - e->scenePos()).manhattanLength();
                if(ml < closest_manhattan_length){
                    closest = sink;
                    closest_manhattan_length = ml;
                }
            }
        }
        if(closest)
            checkDoAcceptConnection(closest);
        if(m_ephemeral_sink){
            // sink arranges its own deletion
            scene()->sendEvent(m_ephemeral_sink, e);
            m_ephemeral_sink = nullptr;
        }
        // accepted -> forward to base explicitly
        QGraphicsObject::mouseReleaseEvent(e);
    }else{
        e->ignore();
    }
}


#include "ephemeralArcEnd.h"
AbstractArcSink* AbstractArcSourceInternal::newArcEnd(){
    debug(5) << "newArcEnd:: this:" << this << "&style:" << &m_style;
    return new EphemeralArcEnd(nullptr, m_style);
}

void AbstractArcSourceInternal::removeHighlights(){
    foreach(AbstractArcSink* k, m_highlighted_items){
        disconnect(k, SIGNAL(disconnected(AbstractArcSink*)),
                   this, SLOT(highlightedItemDisconnected(AbstractArcSink*)));
        k->doPresentHighlight(0);
    }
    m_highlighted_items.clear();
}

void AbstractArcSourceInternal::checkAndHighlightSinks(QPointF scene_pos){
    QPointF ds(120,120);
    QRectF near_field(scene_pos - ds, scene_pos + ds);
    QGraphicsScene *s = scene();
    if(!s){
        error() << "no scene!";
        return;
    }
    AbstractArcSink *k;
    QList<QGraphicsItem *> near_items = s->items(near_field, Qt::ContainsItemBoundingRect);
    //
    QSet<AbstractArcSink*> near_set;
    debug(5) << near_items.size() << "nearby items";    
    foreach(QGraphicsItem* g, near_items)
        if((k = dynamic_cast<AbstractArcSink*>(g)) &&
            k->willAcceptConnection(m_sourceDelegate, k)){
            near_set << k;
            QPointF d = k->scenePos() - scene_pos;
            qreal dl = d.x()*d.x() +d.y()*d.y();
            k->doPresentHighlight(1.0/(1.0 + 0.001*dl));
        }
    debug(5) << "now highlighting" << near_set.size() << "items";
    // for each of the no longer highlighted items:
    foreach(AbstractArcSink* k, m_highlighted_items - near_set){
        k->doPresentHighlight(0);
        connect(k, SIGNAL(disconnected(AbstractArcSink*)),
                this, SLOT(highlightedItemDisconnected(AbstractArcSink*)));
    }
    m_highlighted_items = near_set;
}

void AbstractArcSourceInternal::highlightedItemDisconnected(AbstractArcSink* sink){
    debug() << "highlightedItemDisconnected (async remove while highlighted?)" << sink;
    // sink has probably been deleted by this point
    m_highlighted_items.remove(sink);
}

void AbstractArcSourceInternal::checkDoAcceptConnection(AbstractArcSink* item){
    ConnectionSink::ConnectionStatus status = connectTo(item);
    if(status == ConnectionSink::Rejected){
        debug() << "connection rejected:" << m_sourceDelegate << "-->" << item;
    }
}



void AbstractArcSourceInternal::disconnectParentSignals(QGraphicsItem* p){
    QGraphicsObject* parent = dynamic_cast<QGraphicsObject*>(p);
    if(parent){
        disconnect(parent, SIGNAL(xChanged()), this, SIGNAL(geometryChanged()));
        disconnect(parent, SIGNAL(yChanged()), this, SIGNAL(geometryChanged()));
        disconnect(parent, SIGNAL(parentChanged()), this, SIGNAL(geometryChanged()));
    }
}

void AbstractArcSourceInternal::connectParentSignals(QGraphicsItem* p){
    QGraphicsObject* parent = dynamic_cast<QGraphicsObject*>(p);
    if(parent){
        connect(parent, SIGNAL(xChanged()), this, SIGNAL(geometryChanged()));
        connect(parent, SIGNAL(yChanged()), this, SIGNAL(geometryChanged()));
        connect(parent, SIGNAL(parentChanged()), this, SIGNAL(geometryChanged()));
    }
}

// - AbstractArcSource
AbstractArcSource::AbstractArcSource(ArcStyle const& of_style,
                                     ArcSourceDelegate *sourceDelegate,
                                     Arc* arc)
    : _::AbstractArcSourceInternal(of_style, sourceDelegate, arc),
      LayoutItems(this){
}

AbstractArcSource::~AbstractArcSource(){
    LayoutItems::unRegisterSourceItem(this);
    // manage the lifetime of m_arc even though the arc isn't a child - there's
    // still a 1:1 relationship. This is managed in AbstractArcSource not in
    // AbstractArcSourceInternal because arcs themselves derive from
    // AbstractArcSourceInternal
    //debug() << "~AbstractArcSource" << this << "arc scene" << m_arc->scene() << "m_arc:" << m_arc;
    m_arc->hide();
    if(m_arc->scene())
        m_arc->scene()->removeItem(m_arc);
    m_arc->deleteLater();
}

// - ArcSource
ArcSource::ArcSource(ArcSourceDelegate* sourceDelegate,
                     Arc* arc)
    : AbstractArcSource(arc->style(), sourceDelegate, arc),
      m_front_line(nullptr),
      m_back_line(nullptr){

    setOwnedByLayout(true);

    arc->setFrom(this);

    setFlag(ItemHasNoContents);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    m_back_line = new QGraphicsLineItem(-m_style.back.start_length,0,0,0,this);
    m_front_line = new QGraphicsLineItem(-m_style.front.start_length,0,0,0,this);

    QLinearGradient back_gradient(
        QPointF(-m_style.back.start_length,0),
        QPointF(0,0)
    );
    back_gradient.setColorAt(0, m_style.back.start_col);
    back_gradient.setColorAt(1, m_style.back.col);
    m_back_line->setPen(QPen(
        QBrush(back_gradient), m_style.back.thickness, Qt::SolidLine, Qt::FlatCap
    ));
    
    QLinearGradient front_gradient(
        QPointF(-m_style.front.start_length,0),
        QPointF(0,0)
    );
    front_gradient.setColorAt(0, m_style.front.start_col);
    front_gradient.setColorAt(1, m_style.front.col);
    m_front_line->setPen(QPen(
        QBrush(front_gradient),m_style.front.thickness, Qt::SolidLine, Qt::FlatCap
    ));
    
    setCacheMode(DeviceCoordinateCache);
    m_front_line->setCacheMode(DeviceCoordinateCache);
    m_back_line->setCacheMode(DeviceCoordinateCache);

#ifdef QT_PROFILE_GRAPHICSSCENE
    setProfileName("liquid::ArcSource");
    m_front_line->setProfileName("liquid::ArcSource::front_line");
    m_back_line->setProfileName("liquid::ArcSource::back_line");
#endif // def QT_PROFILE_GRAPHICSSCENE
}

QSizeF ArcSource::sizeHint(Qt::SizeHint which, const QSizeF &constraint) const{
    Q_UNUSED(which);
    Q_UNUSED(constraint);
    return boundingRect().size();// + QSizeF(0,6);
}

void ArcSource::setGeometry(QRectF const& rect){
    prepareGeometryChange();
    // sets geometry()
    QGraphicsLayoutItem::setGeometry(rect);
    debug(7) << "ArcSource::setGeometry" << rect << "(pos=" << pos() << ")";
    setPos(QPointF(2, 0) + rect.topLeft() - boundingRect().topLeft());
}



QRectF ArcSource::boundingRect() const{
    return m_back_line->boundingRect();
}

void ArcSource::paint(QPainter *painter,
                      const QStyleOptionGraphicsItem *opt,
                      QWidget *widget){
    Q_UNUSED(opt);
    Q_UNUSED(widget);
    Q_UNUSED(painter);
}
