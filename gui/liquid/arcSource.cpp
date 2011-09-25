#include "arcSource.h"

using namespace liquid;

/***************************** AbstractArcSource *****************************/
AbstractArcSource::AbstractArcSource(ArcStyle const& of_style,
                                     void* sourceDelegate
                                     Arc* arc)
    : m_style(of_style),
      m_arc(arc),
      m_sourceDelegate(sourceDelegate),
      m_ephemeral_sink(NULL){
}

Arc* AbstractArcSource::arc() const{
    return m_arc;
}

ArcStyle const& AbstractArcSource::style() const{
    return m_style;
}

void AbstractArcSource::mousePressEvent(QGraphicsSceneMouseEvent *e){
    if(event->button() & Qt::LeftButton){
        if(m_ephemeral_arc_end)
            error() << "unmatched mousePressEvent";
        m_ephemeral_arc_end = newArcEnd();
        m_arc->addTo(m_ephemeral_arc_end);
        // in item (this) coordinates (this is now parent of new end):
        // whole pixels matters!
        QPointF centre = m_ephemeral_arc_end->boundingRect().center();
        centre.rx() = std::floor(centre.rx());
        m_ephemeral_arc_end->setPos(event->pos() - centre);
        // !!! TODO: check this
        QGraphicsObject::mousePressEvent(event); 
        event->accept();
    }else{
        event->ignore();
    }
}

void AbstractArcSource::mouseMoveEvent(QGraphicsSceneMouseEvent *e){
    if(m_ephemeral_arc_end){
        m_ephemeral_arc_end->mouseMoveEvent(event);
    }else{
        event->ignore();
    }
}

void AbstractArcSource::mouseReleaseEvent(QGraphicsSceneMouseEvent *e){
    if(event->button() & Qt::LeftButton){
        if(m_ephemeral_arc_end){
            m_ephemeral_arc_end->mouseReleaseEvent(event);
            deleteLater(m_ephemeral_arc_end);
            m_ephemeral_arc_end = NULL;
        }
        // accepted -> forward to base explicitly
        QGraphicsObject::mouseReleaseEvent(event);
    }else{
        event->ignore();
    }
}


#include "ephemeralArcEnd.h"
virtual QGraphicsItem* AbstractArcSource::newArcEnd(){
    return new ephemeralArcEnd();
}

/********************************* ArcSource *********************************/
ArcSource::ArcSource(ArcStyle const& of_style,
                     void* sourceDelegate)
    : AbstractArcSource(of_style, sourceDelegate),
      m_front_line(NULL),
      m_back_line(NULL){

    setFlag(ItemHasNoContents);

    m_back_line = new QGraphicsLineItem(-m_style.back.start_length,0,0,0,this);
    m_front_line = new QGraphicsLineItem(-m_style.front.start_length,0,0,0,this);

    QLinearGradient back_gradient(
        QPointF(-m_style.back.start_length,0),
        connectionPoint()
    );
    back_gradient.setColorAt(0, m_style.back.start_col);
    back_gradient.setColorAt(1, m_style.back.col);
    m_back_line->setPen(QPen(
        QBrush(back_gradient), m_style.back.thickness, Qt::SolidLine, Qt::FlatCap
    ));
    
    QLinearGradient front_gradient(
        QPointF(-m_style.front.start_length,0),
        connectionPoint()
    );
    front_gradient.setColorAt(0, m_style.front.start_col);
    front_gradient.setColorAt(1, m_style.front.col);
    m_front_line->setPen(QPen(
        QBrush(front_gradient),m_style.front.thickness, Qt::SolidLine, Qt::FlatCap
    ));
}

QRectF ArcSource::boundingRect() const{
    return QRectF();
}

void ArcSource::paint(QPainter *painter,
                      const QStyleOptionGraphicsItem *opt,
                      QWidget *widget=0){
    Q_UNUSED(opt);
    Q_UNUSED(widget);
    Q_UNUSED(painter);
}


