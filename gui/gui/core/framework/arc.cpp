#include "arc.h"

#include <QGraphicsObject>
#include <QPainterPath>
#include <QPointF>
#include <QPen>

#include <utility/rounding.h>

using namespace cauv;
using namespace cauv::gui;

JoiningArc::JoiningArc(QGraphicsObject * from, QGraphicsObject * to) : m_to(to), m_from(from) {
    setParentItem(m_from);
    setPen(QPen(QColor(220, 220, 220)));
    connect(from, SIGNAL(xChanged()), this, SLOT(updatePosition()));
    connect(from, SIGNAL(yChanged()), this, SLOT(updatePosition()));
    connect(to, SIGNAL(xChanged()), this, SLOT(updatePosition()));
    connect(to, SIGNAL(yChanged()), this, SLOT(updatePosition()));
    connect(to, SIGNAL(destroyed()), this, SLOT(deleteLater()));
}

void JoiningArc::updatePosition() {
    bool toOnLeft = m_to->pos().x() + m_to->boundingRect().width() < 0;
    bool toOnRight = m_to->pos().x() > m_from->boundingRect().width();

    qreal side = (m_to->pos().x() + m_to->boundingRect().width()/2 > m_from->boundingRect().width()/2) ? 1 : -1;
    qreal onTop = (m_to->pos().y() + m_to->boundingRect().height()/2 > m_from->boundingRect().height()/2) ? 1 : -1;

    // from
    qreal fromX = 0;
    if(side > 0) fromX = m_from->boundingRect().width();
    qreal fromY = m_from->boundingRect().height()/2;

    // to
    qreal toX = m_to->pos().x();
    if(side < 0) toX += m_to->boundingRect().width();
    qreal toY = m_to->pos().y() + m_to->boundingRect().height()/2;

    // values are halved as the curves come from both sides, forming
    // half of the eventual curve each
    qreal curveSizeX = side * clamp(0, abs((fromX - toX)/2), 30);
    qreal curveSizeY = onTop * clamp(0, abs((fromY - toY)/2), 30);

    QPainterPath path;
    path.moveTo(fromX, fromY);

    if(!toOnRight && !toOnLeft) {
        // overlapping - just use a straight line
        path.lineTo(QPointF(toX, toY));
    } else {
        path.cubicTo(QPointF(fromX + curveSizeX, fromY), // control point 1
                     QPointF(fromX + curveSizeX, fromY + curveSizeY), // control point 2
                     QPointF(fromX + curveSizeX, fromY + curveSizeY)); // end point

        qreal lineLengthY = abs(fromY - toY) - abs(curveSizeY * 2);
        if(lineLengthY < 0) lineLengthY = 0;
        path.lineTo(QPointF(fromX + curveSizeX, fromY + curveSizeY + (onTop * lineLengthY)));


        qreal lineLengthX = abs(fromX - toX) - abs(curveSizeX * 2);
        if(lineLengthX < 0) lineLengthX = 0;

        path.moveTo((-side * lineLengthX) + toX, toY);
        path.cubicTo(QPointF((-side * lineLengthX) + toX - curveSizeX, toY), // control point 1
                     QPointF((-side * lineLengthX) + toX - curveSizeX, toY - curveSizeY), // control point 2
                     QPointF((-side * lineLengthX) + toX - curveSizeX, toY - curveSizeY)); // end point


        path.moveTo((-side * lineLengthX) + toX, toY);
        path.lineTo(QPointF(toX, toY));
    }

    setPath(path);
}
