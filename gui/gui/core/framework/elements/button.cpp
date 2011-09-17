#include "button.h"

#include <QPixmapCache>
#include <QGraphicsPixmapItem>
#include <QGraphicsSceneMouseEvent>
#include <QTimer>

using namespace cauv;
using namespace cauv::gui;

Button::Button(QRectF clip, QString base_fname, QGraphicsItem *parent)
    : QGraphicsObject(parent), m_clip(clip), m_default(), m_hover(), m_pressed(){
    setAcceptHoverEvents(true);
    setFlag(ItemClipsChildrenToShape);

    m_default = loadPix(base_fname + ".png");
    m_hover   = loadPix(base_fname + ".hover.png");
    m_pressed = loadPix(base_fname + ".pressed.png");

    m_hover->hide();
    m_pressed->hide();
}

QRectF Button::boundingRect() const{
    return m_clip;
}

void Button::paint(QPainter *p, const QStyleOptionGraphicsItem *o, QWidget *w){
    Q_UNUSED(p);
    Q_UNUSED(o);
    Q_UNUSED(w);
}

QPainterPath Button::shape() const{
    QPainterPath p;
    p.addRect(m_clip);
    return p;
}


void Button::hoverEnterEvent(QGraphicsSceneHoverEvent *event){
    Q_UNUSED(event);
    if(!m_pressed->isVisible()){
        m_hover->show();
        m_default->hide();
    }
}

void Button::hoverLeaveEvent(QGraphicsSceneHoverEvent *event){
    Q_UNUSED(event);
    if(m_hover->isVisible()){
        m_default->show();
        m_hover->hide();        
    }
}

void Button::mousePressEvent(QGraphicsSceneMouseEvent *event){
    Q_UNUSED(event);
    m_hover->hide();
    m_default->hide();
    m_pressed->show();
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
        m_pressed->hide();
        m_hover->hide();
        m_default->show();
    }
}

void Button::delayedRaise(){
    m_pressed->hide();
    m_hover->show();
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

