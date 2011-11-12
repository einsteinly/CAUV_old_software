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

#include "multiArc.h"

#include <QPainterPath>
#include <QPointF>
#include <QPen>
#include <QGraphicsSceneMouseEvent>

#include <utility/rounding.h>
#include <utility/qt_streamops.h>

#include <cassert>

#include <debug/cauv_debug.h>

#include "multiArcEnd.h"
#include "style.h"

using namespace cauv;
using namespace cauv::gui;
using namespace liquid;

MultiArc::MultiArc(ManagedElement const& m,
                   liquid::ArcStyle const& style,
                   ConnectableInterface *from,
                   ConnectableInterface *to)
    : QObject(),
      QGraphicsPathItem(),
      ManagedElement(m),
      m_to(),
      m_from(from),
      m_front(new QGraphicsPathItem(this)),
      m_ephemeral_arc_end(NULL),
      m_style(style){
    assert(from);
    QGraphicsObject *from_as_go = from->asQGraphicsObject();
    assert(from_as_go);
    setParentItem(from_as_go);

    setFlag(ItemIsMovable, false);

    connect(from_as_go, SIGNAL(boundriesChanged()), this, SLOT(updateLayout()));
    if(to)
        addTo(to);

    updateLayout();
}

liquid::ArcStyle const& MultiArc::style() const{
    return m_style;
}


void MultiArc::addTo(ConnectableInterface* to){
    assert(to);
    m_to.push_back(to);
    QGraphicsObject *to_as_go = to->asQGraphicsObject();
    connect(to_as_go, SIGNAL(boundriesChanged()), this, SLOT(updateLayout()));
    connect(to_as_go, SIGNAL(disconnected()), this, SLOT(updateLayout()));
    updateLayout();
}

void MultiArc::removeTo(ConnectableInterface* to){
    m_to.removeOne(to);
    assert(m_to.indexOf(to) == -1);
    QGraphicsObject *to_as_go = to->asQGraphicsObject();    
    disconnect(to_as_go, 0, this, 0);
    updateLayout();
}

void MultiArc::mousePressEvent(QGraphicsSceneMouseEvent *event){
    if(event->button() & Qt::LeftButton){
        if(m_ephemeral_arc_end)
            error() << "unmatched mousePressEvent";
        //assert(!m_ephemeral_arc_end);
        m_ephemeral_arc_end = new MultiArcEnd(this, true);
        // in item (this) coordinates (this is now parent of new end):
        // whole pixels matters!
        QPointF centre = m_ephemeral_arc_end->boundingRect().center();
        centre.rx() = std::floor(centre.rx());
        m_ephemeral_arc_end->setPos(event->pos() - centre);
        // accepted -> forward to base explicitly
        //QGraphicsObject::mousePressEvent(event); 
    }else{
        event->ignore();
    }
}

void MultiArc::mouseMoveEvent(QGraphicsSceneMouseEvent *event){
    if(m_ephemeral_arc_end){
        m_ephemeral_arc_end->mouseMoveEvent(event);
    }else{
        event->ignore();
    }
}

void MultiArc::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    if(event->button() & Qt::LeftButton){
        if(m_ephemeral_arc_end)
            m_ephemeral_arc_end->mouseReleaseEvent(event);
        // deletion arranged by mouseReleaseEvent
        m_ephemeral_arc_end = NULL;
        // accepted -> forward to base explicitly
        QGraphicsPathItem::mouseReleaseEvent(event);
    }else{
        event->ignore();
    }
}


QRectF MultiArc::boundingRect() const{
    return QGraphicsPathItem::boundingRect();
}

void MultiArc::updateLayout(){

    // we draw in m_from's coordinate system (it is this's parent)
    QPointF start_point = m_from->connectionPoint();
    QPointF split_point = start_point + QPointF(8,0);
    
    // own style (back)
    setPen(QPen(
        QBrush(m_style.back.col), m_style.back.thickness, Qt::SolidLine, Qt::FlatCap
    ));

    // front style:
    m_front->setPen(QPen(
        QBrush(m_style.front.col), m_style.front.thickness, Qt::SolidLine, Qt::FlatCap
    ));

    QPainterPath path(start_point);
    path.lineTo(split_point);

    if(m_to.size()){
        foreach(ConnectableInterface* ci, m_to){
            QGraphicsObject* o = ci->asQGraphicsObject();
            path.moveTo(split_point);
            // mapToItem maps connection point in 'to' object into m_from's
            // coordinates 
            QPointF end_point(o->mapToItem(m_from->asQGraphicsObject(), ci->connectionPoint()));
            QPointF c1(split_point + QPointF(5+std::fabs(end_point.x() - split_point.x())/2, 0));
            QPointF c2(end_point   - QPointF(5+std::fabs(end_point.x() - split_point.x())/2, 0));
            path.cubicTo(c1, c2, end_point);
            //debug() << "f=" << start_point << "s=" << split_point << "t=" << end_point;
            //path.lineTo(end_point);
        }
    }else{
        path.lineTo(split_point + QPointF(4,0));
    }

    setPath(path);
    m_front->setPath(path);
}
