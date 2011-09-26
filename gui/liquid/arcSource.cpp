#include "arcSource.h"

#include <cmath>

#include <QGraphicsSceneMouseEvent>

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
        // whole pixels matters!
        //QPointF centre = m_ephemeral_sink->boundingRect().size().center();
        //centre.rx() = std::floor(centre.rx());
        m_ephemeral_sink->setPos(e->pos());// - centre);
        m_ephemeral_sink->mousePressEvent(e);
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
        // !!! TODO: sourceDelegate stuff...
        m_ephemeral_sink->mouseMoveEvent(e);
    }else{
        e->ignore();
    }
}

void AbstractArcSource::mouseReleaseEvent(QGraphicsSceneMouseEvent *e){
    debug() << "AbstractArcSource::mouseReleaseEvent";
    if(e->button() & Qt::LeftButton){
        if(m_ephemeral_sink){
            // sink arranges its own deletion
            m_ephemeral_sink->mouseReleaseEvent(e);
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

/********************************* ArcSource *********************************/
ArcSource::ArcSource(void* sourceDelegate,
                     Arc* arc)
    : AbstractArcSource(arc->style(), sourceDelegate, arc),
      m_front_line(NULL),
      m_back_line(NULL){
    
    arc->setFrom(this);

    setFlag(ItemHasNoContents);
    setSizePolicy(QSizePolicy::Fixed);

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
    return boundingRect().size();
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


