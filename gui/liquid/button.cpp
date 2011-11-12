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

#include "button.h"

#include <QPixmapCache>
#include <QGraphicsPixmapItem>
#include <QGraphicsSceneMouseEvent>
#include <QTimer>
#include <QAbstractGraphicsShapeItem>

using namespace liquid;

Button::Button(QRectF clip,
               QString base_fname,
               QAbstractGraphicsShapeItem *back_item,
               QGraphicsItem *parent)
    : QGraphicsWidget(parent), m_clip(clip), m_default(), m_hover(),
      m_pressed(), m_background(back_item){
    setAcceptHoverEvents(true);
    //setHandlesChildEvents(true);
    setFlag(ItemClipsChildrenToShape);
    resize(clip.size());

    m_default = loadPix(base_fname + ".png");
    m_hover   = loadPix(base_fname + ".hover.png");
    m_pressed = loadPix(base_fname + ".pressed.png");

    m_hover->hide();
    m_pressed->hide();
}

Button::Button(QRectF clip,
               QGraphicsItem *default_item,
               QGraphicsItem *hover_item,
               QGraphicsItem *pressed_item,
               QAbstractGraphicsShapeItem *back_item,
               QGraphicsItem *parent)
    : QGraphicsWidget(parent),
      m_clip(clip),
      m_default(default_item),
      m_hover(hover_item),
      m_pressed(pressed_item),
      m_background(back_item){

    setAcceptHoverEvents(true);
    //setHandlesChildEvents(true);

    default_item->setParentItem(this);
    
    if(m_hover)
        m_hover->hide();
    if(m_pressed)
        m_pressed->hide();

    if(!m_background){
        m_background = new QGraphicsEllipseItem(0, 0, clip.width(), clip.height(), this);
        m_background->setFlag(ItemStacksBehindParent);
        m_background->setBrush(QBrush(Qt::white));
    }
    
}

void Button::setPen(QPen pen){
    if(m_background)
        m_background->setPen(pen);
}

void Button::setBrush(QBrush brush){
    if(m_background)
        m_background->setBrush(brush);
}

QSizeF Button::sizeHint(Qt::SizeHint which, const QSizeF &constraint) const{
    Q_UNUSED(which);
    Q_UNUSED(constraint);
    return m_clip.size();
}

QRectF Button::boundingRect() const{
    return m_clip;
}

QPainterPath Button::shape() const{
    QPainterPath p;
    p.addRect(m_clip);
    return p;
}


void Button::hoverEnterEvent(QGraphicsSceneHoverEvent *event){
    Q_UNUSED(event);
    if((!m_pressed) || (!m_pressed->isVisible())){
        if(m_hover){
            m_hover->show();
            m_default->hide();
        }
        if(m_background){
            m_background->setBrush(QBrush(m_background->brush().color().darker(103)));
        }
    }
}

void Button::hoverLeaveEvent(QGraphicsSceneHoverEvent *event){
    Q_UNUSED(event);
    if(m_hover)
        m_hover->hide();
    m_default->show();
    if(m_background)
        m_background->setBrush(QBrush(m_background->brush().color().lighter(103)));
}

void Button::mousePressEvent(QGraphicsSceneMouseEvent *event){
    Q_UNUSED(event);
    if(m_hover)
        m_hover->hide();
    if(m_pressed){
        m_default->hide();
        m_pressed->show();
    }else{
        m_default->show();
    }
    if(m_background){
        m_background->setBrush(QBrush(m_background->brush().color().darker(105)));
    }
}

void Button::mouseMoveEvent(QGraphicsSceneMouseEvent *event){
    Q_UNUSED(event);
    // !!! how do hover and pressed interact?
}

void Button::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    if(contains(event->pos())){
        Q_EMIT(pressed());
        QTimer::singleShot(120, this, SLOT(delayedRaise()));
    }else{
        if(m_pressed)
            m_pressed->hide();
        if(m_hover)
            m_hover->hide();
        m_default->show();
    }
}

void Button::delayedRaise(){
    if(m_pressed)
        m_pressed->hide();
    if(m_hover){
        m_hover->show();
        m_default->hide();
    }else
        m_default->show();
    if(m_background)
        m_background->setBrush(QBrush(m_background->brush().color().lighter(105)));
}

QGraphicsPixmapItem* Button::loadPix(QString n){
    QPixmap p;
    if(!QPixmapCache::find(n, &p)){
        p.load(n);
        QPixmapCache::insert(n, p);
    }
    QGraphicsPixmapItem *r = new QGraphicsPixmapItem(p, this);

    return r;
}

