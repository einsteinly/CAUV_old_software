/* Copyright 2012 Cambridge Hydronautics Ltd.
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

#include "shadow.h"

#include <QGraphicsPathItem>
#include <QGraphicsBlurEffect>
#include <QPen>

liquid::Shadow::Shadow(QGraphicsItem* parent)
    : QGraphicsObject(parent),
      m_shape(new QGraphicsPathItem(this)),
      m_blur_effect_applied(false){
    setFlag(ItemHasNoContents);
    m_shape->setPen(QPen(Qt::NoPen));
    setBrush(QBrush(Qt::NoBrush));
}

QRectF liquid::Shadow::boundingRect() const{
    return QRect();
}

void liquid::Shadow::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){
    Q_UNUSED(painter);
    Q_UNUSED(option);
    Q_UNUSED(widget);
}

void liquid::Shadow::setShape(QPainterPath shape){
    m_shape->setPath(shape);
}

void liquid::Shadow::setBrush(QBrush brush){
    m_shape->setBrush(brush);
    if(brush != QBrush(Qt::NoBrush)){
        if(!m_blur_effect_applied) {
            QGraphicsBlurEffect *blur = new QGraphicsBlurEffect();
            blur->setBlurRadius(25);
            m_shape->setGraphicsEffect(blur);
        }
        m_blur_effect_applied = true;
    }else if(m_blur_effect_applied){
        m_shape->setGraphicsEffect(NULL);
        m_blur_effect_applied = false;
    }
}

