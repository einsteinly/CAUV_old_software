/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "nodeHeader.h"

#include <algorithm>

#include <QGraphicsPathItem>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsSceneHoverEvent>
#include <QPropertyAnimation>
#include <QFontMetrics>

#include "button.h"
#include "style.h"
#include "proxyWidget.h"

#define CAUV_DEBUG_COMPAT
#include <debug/cauv_debug.h>

using namespace liquid;


NodeHeader::NodeHeader(NodeStyle const& style, QGraphicsObject *parent)
    : QGraphicsObject(parent),
      m_style(style),
      m_width(0),
      m_overlay_back(),
      m_title(),
      m_info_text(){
    setAcceptHoverEvents(true);
    setFlag(ItemHasNoContents);
    setCacheMode(ItemCoordinateCache);

    m_overlay_back = new QGraphicsPathItem(this);
    m_overlay_back->setPen(m_style.header.pen);
    m_overlay_back->setBrush(m_style.header.brush);
    m_overlay_back->setZValue(1);

    m_title        = new QGraphicsSimpleTextItem(this);
    m_title->setPen(m_style.header.title.pen);
    m_title->setBrush(m_style.header.title.brush);
    m_title->setFont(m_style.header.title.font);
    m_title->setZValue(2);

    m_info_text    = new LODItem<QGraphicsSimpleTextItem>(this);
    m_info_text->setPen(m_style.header.info.pen);
    m_info_text->setBrush(m_style.header.info.brush);
    m_info_text->setFont(m_style.header.info.font);
    m_info_text->setZValue(2);

    m_width = minimumWidth();

    setCacheMode(DeviceCoordinateCache);
    m_title->setCacheMode(ItemCoordinateCache);
    m_info_text->setCacheMode(ItemCoordinateCache);

    #ifdef QT_PROFILE_GRAPHICSSCENE
    setProfileName("liquid::NodeHeader");
    m_overlay_back->setProfileName("liquid::NodeHeader::overlay_back");
    m_title->setProfileName("liquid::NodeHeader::title");
    m_info_text->setProfileName("liquid::NodeHeader::info_text");
    #endif // def QT_PROFILE_GRAPHICSSCENE
}

float NodeHeader::minimumWidth() const{
    const QFontMetrics title_fm(m_title->font());
    const QFontMetrics info_fm(m_info_text->font());

    return m_style.tl_radius/2 + std::max(
        title_fm.width(m_title->text()),
        info_fm.width(m_info_text->text())
    );
}

QRectF NodeHeader::boundingRect() const{
    return QRectF(0,0,m_width,m_style.header.height);
}

void NodeHeader::paint(QPainter *p, const QStyleOptionGraphicsItem *o, QWidget *w){
    Q_UNUSED(p);
    Q_UNUSED(o);
    Q_UNUSED(w);
}

void NodeHeader::hoverEnterEvent(QGraphicsSceneHoverEvent *event){
    Q_UNUSED(event);
    QPropertyAnimation *fadeOut = new QPropertyAnimation(this, "opacity");
    fadeOut->setEndValue(0.2);
    fadeOut->setDuration(100);
    fadeOut->start();
}

void NodeHeader::hoverLeaveEvent(QGraphicsSceneHoverEvent *event){
    Q_UNUSED(event);
    QPropertyAnimation *fadeIn = new QPropertyAnimation(this, "opacity");
    fadeIn->setEndValue(1.0);
    fadeIn->setDuration(100);
    fadeIn->start();
}

void NodeHeader::setTitle(QString title){
    if(m_title->text() != title){
        m_title->setText(title);
        setWidth(m_width);
    }
}

void NodeHeader::setInfo(QString info){
    if(m_info_text->text() != info){
        m_info_text->setText(info);
        setWidth(m_width);
    }
}

void NodeHeader::addButton(QString name, Button *button){
    button->setParentItem(this);
    m_button_lookup[name] = button;
    m_buttons << button;
    button->setFlag(ItemIgnoresParentOpacity);
    button->setZValue(0);
    // force re-layout
    setWidth(m_width);
}

Button* NodeHeader::getButton(QString name){
     return m_button_lookup.value(name);
}

void NodeHeader::setWidth(qreal w){
    if(w != m_width)
        prepareGeometryChange();
    m_width = w;
    size_t num_buttons = m_buttons.size();
    
    // !!! TODO: could use QGraphicsLinearLayout, etc
    // buttons:
    QPointF cursor(2,2);

    if(!m_buttons.isEmpty()){
        for(size_t i = 0; i < (num_buttons+1)/2; i++){
            m_buttons[i]->setPos(cursor);
            cursor += QPointF(m_buttons[i]->size().width(),0);
        }
        
        cursor = QPointF(m_width-m_buttons.last()->size().width(),2);
        for(size_t i = 1; i < 1+num_buttons/2; i++){
            size_t idx = num_buttons - i;
            m_buttons[idx]->setPos(cursor);
            cursor -= QPointF(m_buttons[idx]->size().width(),0);
        }
    }

    // overlay back:
    QPainterPath p(QPointF(m_style.tl_radius, 0));

    p.lineTo(w, 0);
    p.lineTo(w, m_style.header.height);
    p.lineTo(0, m_style.header.height);
    p.lineTo(0, m_style.header.height - m_style.tl_radius);
    p.arcTo(
        QRectF(0, 0,
               m_style.tl_radius, m_style.tl_radius),
        -180, -90
    );

    p.lineTo(m_style.tl_radius, 0);
    
    m_overlay_back->setPath(p);
    
    // overlay text:
    QPointF title_pos = NodeHeader::boundingRect().center() - m_title->boundingRect().center();
    title_pos.ry() = 1 + m_title->boundingRect().top();
    m_title->setPos(QPointF(title_pos.toPoint()));

    QPointF info_pos = NodeHeader::boundingRect().center() - m_info_text->boundingRect().center();
    QRectF title_rect = m_title->boundingRect();
    title_rect.translate(title_pos);

    info_pos.ry() = title_rect.bottom() + 1 + m_info_text->boundingRect().top();
    m_info_text->setPos(QPointF(info_pos.toPoint()));
}

