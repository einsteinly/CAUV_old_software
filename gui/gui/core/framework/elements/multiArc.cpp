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
const static QColor Lead_In_Colour = QColor(100, 128, 180, 0);
const static qreal Line_Width = 4.0;

const static qreal Lead_In_Length = 8.0;

MultiArc::MultiArc(ConnectableInterface * from, ConnectableInterface * to)
    : m_to(),
      m_from(from->asQGraphicsObject()){
    assert(m_from);

    connect(m_from, SIGNAL(boundriesChanged()), this, SLOT(updateLayout()));
    if(to){
        addTo(to);
    }
    updateLayout();
}

void MultiArc::addTo(ConnectableInterface* to){
    assert(to);
    m_to.push_back(to->asQGraphicsObject());
    connect(m_to.back(), SIGNAL(boundriesChanged()), this, SLOT(updateLayout()));
    connect(m_to.back(), SIGNAL(disconnected()), this, SLOT(updateLayout()));
    updateLayout();
}

void MultiArc::updateLayout(){
    QPointF start_point = m_from->pos();
    QPointF lead_in_start = start_point - QPointF(Lead_In_Length, 0); // before start
    QPointF split_point = start_point + QPointF(8,0);
    
    QLinearGradient line_gradient(lead_in_start, start_point);
    line_gradient.setColorAt(0, Lead_In_Colour);
    line_gradient.setColorAt(1, Line_Colour);
    QBrush brush(line_gradient);
    setPen(QPen(brush, Line_Width));

    // from connection stub (even if not connected to anything)
    // !!! this should be part of from item, would also clean up the gradient
    // thing
    QPainterPath path(lead_in_start);
    path.lineTo(split_point);

    if(m_to.size()){
        foreach(QGraphicsObject* o, m_to){
            path.moveTo(split_point);
            QPointF end_point(o->pos());
            QPointF c1(split_point + QPointF((end_point.x() - split_point.x())/2, 0));
            QPointF c2(end_point   - QPointF((end_point.x() - split_point.x())/2, 0));
            path.cubicTo(c1, c2, end_point);
            //debug() << "f=" << start_point << "s=" << split_point << "t=" << end_point;
            //path.lineTo(end_point);
        }
    }else{
        path.lineTo(split_point + QPointF(4,0));
    }

    setPath(path);
}
