#include "multiArc.h"

#include <QPainterPath>
#include <QPointF>
#include <QPen>

#include <utility/rounding.h>
#include <utility/qt_streamops.h>

#include <cassert>

#include <debug/cauv_debug.h>

using namespace cauv;
using namespace cauv::gui;

const static QColor Line_Colour = QColor(100, 128, 180);
const static qreal Thickness = 4.0;

const static qreal Lead_In_Length = 8.0;

MultiArc::MultiArc(ConnectableInterface *from, ConnectableInterface *to)
    : m_to(),
      m_from(from){
    assert(from);
    QGraphicsObject *from_as_go = from->asQGraphicsObject();
    assert(from_as_go);
    setParentItem(from_as_go);

    connect(from_as_go, SIGNAL(boundriesChanged()), this, SLOT(updateLayout()));
    if(to){
        addTo(to);
    }
    updateLayout();
}

QColor MultiArc::startColour(){
    return Line_Colour;
}

QColor MultiArc::endColour(){
    return Line_Colour;
}

void MultiArc::addTo(ConnectableInterface* to){
    assert(to);
    m_to.push_back(to);
    QGraphicsObject *to_as_go = to->asQGraphicsObject();
    connect(to_as_go, SIGNAL(boundriesChanged()), this, SLOT(updateLayout()));
    connect(to_as_go, SIGNAL(disconnected()), this, SLOT(updateLayout()));
    updateLayout();
}

void MultiArc::updateLayout(){
    // we draw in m_from's coordinate system (it is this's parent)
    QPointF start_point = m_from->connectionPoint();
    QPointF split_point = start_point + QPointF(8,0);
    
    QBrush brush(Line_Colour);
    setPen(QPen(brush, Thickness));

    QPainterPath path(start_point);
    path.lineTo(split_point);

    if(m_to.size()){
        foreach(ConnectableInterface* o, m_to){
            path.moveTo(split_point);
            // mapToItem maps point 0,0 in 'to' object into m_from's
            // coordinates
            QPointF end_point(o->asQGraphicsObject()->mapToItem(m_from->asQGraphicsObject(), 0,0));
            QPointF c1(split_point + QPointF(std::fabs(end_point.x() - split_point.x())/2, 0));
            QPointF c2(end_point   - QPointF(std::fabs(end_point.x() - split_point.x())/2, 0));
            path.cubicTo(c1, c2, end_point);
            //debug() << "f=" << start_point << "s=" << split_point << "t=" << end_point;
            //path.lineTo(end_point);
        }
    }else{
        path.lineTo(split_point + QPointF(4,0));
    }

    setPath(path);
}
