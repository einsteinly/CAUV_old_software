#include "nodeHeader.h"

#include <QGraphicsPathItem>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsSceneHoverEvent>
#include <QPropertyAnimation>

#include <debug/cauv_debug.h>

#include "button.h"
#include "style.h"

using namespace cauv;
using namespace cauv::gui;


NodeHeader::NodeHeader(NodeStyle const& style, QGraphicsObject *parent)
    : QGraphicsObject(parent),
      m_style(style),
      m_width(0),
      m_closebutton(),
      m_collapsebutton(),
      m_execbutton(),
      m_dupbutton(),
      m_overlay_back(),
      m_title(),
      m_info_text(){
    setAcceptHoverEvents(true);
    setFlag(ItemHasNoContents);

    m_closebutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/x_button"), NULL, this
    );
    m_collapsebutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/collapse_button"), NULL, this
    );
    m_execbutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/reexec_button"), NULL, this
    );
    m_dupbutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/dup_button"), NULL, this
    );
    m_closebutton->setFlag(ItemIgnoresParentOpacity);
    m_collapsebutton->setFlag(ItemIgnoresParentOpacity);
    m_execbutton->setFlag(ItemIgnoresParentOpacity);
    m_dupbutton->setFlag(ItemIgnoresParentOpacity);

    m_overlay_back = new QGraphicsPathItem(this);
    m_overlay_back->setPen(m_style.header.pen);
    m_overlay_back->setBrush(m_style.header.brush);

    m_title        = new QGraphicsSimpleTextItem(this);
    m_title->setPen(m_style.header.title.pen);
    m_title->setBrush(m_style.header.title.brush);
    m_title->setFont(m_style.header.title.font);

    m_info_text    = new QGraphicsSimpleTextItem(this);
    m_info_text->setPen(m_style.header.info.pen);
    m_info_text->setBrush(m_style.header.info.brush);
    m_info_text->setFont(m_style.header.info.font);

    m_title->setText("File Input");
    m_info_text->setText("12.6MB/s 17Hz");

    connect(m_closebutton, SIGNAL(pressed()), this, SIGNAL(closePressed()));
    connect(m_collapsebutton, SIGNAL(pressed()), this, SIGNAL(collapsePressed()));
    connect(m_execbutton, SIGNAL(pressed()), this, SIGNAL(execPressed()));
    connect(m_dupbutton, SIGNAL(pressed()), this, SIGNAL(duplicatePressed()));
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

void NodeHeader::setWidth(qreal w){
    if(m_width == w)
        return;

    prepareGeometryChange();
    m_width = w;
    
    // buttons:
    QPointF cursor(2,2);
    QPointF delta((m_width-4)/4,0);
    
    m_closebutton->setPos(cursor);
    m_collapsebutton->setPos(cursor += delta);
    m_execbutton->setPos(cursor += delta);
    m_dupbutton->setPos(cursor += delta);
    
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
    QPointF title_pos = boundingRect().center() - m_title->boundingRect().center();
    title_pos.ry() = 1 + m_title->boundingRect().top();
    m_title->setPos(title_pos);

    QPointF info_pos = boundingRect().center() - m_info_text->boundingRect().center();
    QRectF title_rect = m_title->boundingRect();
    title_rect.translate(title_pos);

    info_pos.ry() = title_rect.bottom() + 1 + m_info_text->boundingRect().top();
    m_info_text->setPos(info_pos);

}

