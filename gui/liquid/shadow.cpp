/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
    
    #ifdef QT_PROFILE_GRAPHICSSCENE
        setProfileName("liquid::Shadow");
    #endif // def QT_PROFILE_GRAPHICSSCENE
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
            auto blur = new QGraphicsBlurEffect();
            blur->setBlurRadius(25);
            m_shape->setGraphicsEffect(blur);
        }
        m_blur_effect_applied = true;
    }else if(m_blur_effect_applied){
        m_shape->setGraphicsEffect(NULL);
        m_blur_effect_applied = false;
    }
}

