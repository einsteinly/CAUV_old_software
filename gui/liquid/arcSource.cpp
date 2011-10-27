#include "arcSource.h"

#include <cmath>

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsScene>
#include <QApplication>


#include <debug/cauv_debug.h>
#include <utility/qt_streamops.h>

#include "arc.h"
#include "arcSink.h"

using namespace liquid;

/***************************** AbstractArcSource *****************************/
AbstractArcSource::AbstractArcSource(ArcStyle const& of_style,
                                     void* sourceDelegate,
                                     Arc* arc)
    : m_style(of_style),
      m_arc(arc),
      m_sourceDelegate(sourceDelegate),
      m_ephemeral_sink(NULL){
    debug() << "AbstractArcSource()" << this;
}
AbstractArcSource::~AbstractArcSource(){
    debug() << "~AbstractArcSource()" << this;
}

Arc* AbstractArcSource::arc() const{
    return m_arc;
}

ArcStyle const& AbstractArcSource::style() const{
    return m_style;
}

void AbstractArcSource::mousePressEvent(QGraphicsSceneMouseEvent *e){
    debug() << "AbstractArcSource::mousePressEvent";
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

void AbstractArcSource::mouseMoveEvent(QGraphicsSceneMouseEvent *e){
    debug() << "AbstractArcSource::mouseMoveEvent";
    if(m_ephemeral_sink){
        checkAndHighlightSinks(e->scenePos());
        scene()->sendEvent(m_ephemeral_sink, e);
    }else{
        e->ignore();
    }
}

void AbstractArcSource::mouseReleaseEvent(QGraphicsSceneMouseEvent *e){
    debug() << "AbstractArcSource::mouseReleaseEvent";
    if(e->button() & Qt::LeftButton){
        removeHighlights();
        // !!! TODO: call doAcceptConnection
        if(m_ephemeral_sink){
            // sink arranges its own deletion
            scene()->sendEvent(m_ephemeral_sink, e);
            m_ephemeral_sink = NULL;
        }
        // accepted -> forward to base explicitly
        QGraphicsObject::mouseReleaseEvent(e);
    }else{
        e->ignore();
    }
}


#include "ephemeralArcEnd.h"
AbstractArcSink* AbstractArcSource::newArcEnd(){
    debug() << "newArcEnd:: this:" << this << "&style:" << &m_style;
    return new EphemeralArcEnd(m_style);
}

void AbstractArcSource::removeHighlights(){
    foreach(AbstractArcSink* k, m_highlighted_items)
        k->doPresentHighlight(0);
}

void AbstractArcSource::checkAndHighlightSinks(QPointF scene_pos){
    QPointF ds(40,40);
    QRectF near_field(scene_pos - ds, scene_pos + ds);
    QGraphicsScene *s = scene();
    if(!s){
        error() << "no scene!";
        return;
    }
    AbstractArcSink *k;
    QList<QGraphicsItem *> near_items = s->items(near_field);
    QSet<AbstractArcSink*> near_set;
    debug() << near_items.size() << "nearby items";    
    foreach(QGraphicsItem* g, near_items)
        if((k = dynamic_cast<AbstractArcSink*>(g)) &&
            k->willAcceptConnection(m_sourceDelegate)){
            near_set << k;
            QPointF d = k->scenePos() - scene_pos;
            qreal dl = d.x()*d.x() +d.y()*d.y();
            k->doPresentHighlight(1.0/(1.0 + 0.002*dl));
        }
    debug() << "now highlighting" << near_set.size() << "items";
    // for each of the no longer highlighted items:
    foreach(AbstractArcSink* k, m_highlighted_items - near_set)
        k->doPresentHighlight(0);
    m_highlighted_items = near_set;
}


/********************************* ArcSource *********************************/
ArcSource::ArcSource(void* sourceDelegate,
                     Arc* arc)
    : AbstractArcSource(arc->style(), sourceDelegate, arc),
      m_front_line(NULL),
      m_back_line(NULL){
    
    arc->setFrom(this);
    arc->setZValue(1);

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
}

QSizeF ArcSource::sizeHint(Qt::SizeHint which, const QSizeF &constraint) const{
    Q_UNUSED(which);
    Q_UNUSED(constraint);
    return boundingRect().size() + QSizeF(0,6);
}

void ArcSource::setGeometry(QRectF const& rect){
    prepareGeometryChange();
    // sets geometry()
    QGraphicsLayoutItem::setGeometry(rect);
    debug() << "ArcSource::setGeometry" << rect << "(pos=" << pos() << ")";
    setPos(rect.topLeft() - boundingRect().topLeft());
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


