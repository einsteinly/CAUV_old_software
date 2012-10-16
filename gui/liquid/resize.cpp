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

#include "resize.h"

#include <QPainter>
#include <QCursor>
#include <QStyleOptionGraphicsItem>

using namespace liquid;

ResizeHandle::ResizeHandle(QGraphicsObject *parent) : QGraphicsObject(parent),
m_size(25, 25), m_pen(QColor(Qt::white), 2){
    this->setHandlesChildEvents(true);
    this->setFlag(QGraphicsItem::ItemIsMovable);
    //this->setFlag(QGraphicsItem::ItemIsSelectable);
    this->setCursor(QCursor(Qt::SizeFDiagCursor));
   
    #ifdef QT_PROFILE_GRAPHICSSCENE
        setProfileName("liquid::ResizeHandle");
    #endif // def QT_PROFILE_GRAPHICSSCENE
}

QRectF ResizeHandle::boundingRect() const{
    return QRectF(QPointF(0,0), m_size);
}

void ResizeHandle::paint(QPainter *painter, const QStyleOptionGraphicsItem * option, QWidget *){
    const qreal lod = option->levelOfDetailFromTransform(painter->worldTransform());
    if(lod > 0.2){
        painter->setPen(pen());

        qreal numLines = 3;
        qreal margin = 3;

        // work out spacing
        qreal spacingH = (size().width() - (2*margin)) / numLines;
        qreal spacingV = (size().height() - (2*margin)) / numLines;

        for (int i = 0; i < numLines; i++){
            painter->drawLine(margin + (i * spacingH), size().height()-margin,
                              size().width() - margin, margin + (i * spacingV));
        }
    }
}

QPen ResizeHandle::pen() const{
    return m_pen;
}

void ResizeHandle::setPen(QPen const& pen){
    m_pen = pen;
    update(boundingRect());
}

QSizeF ResizeHandle::newSize() const {
    return QSizeF(x() + size().width(), y() + size().height());
}

QSizeF ResizeHandle::size() const{
    return m_size;
}

void ResizeHandle::setSize(QSizeF const& s){
    prepareGeometryChange();
    m_size = s;
    update(boundingRect());
}


