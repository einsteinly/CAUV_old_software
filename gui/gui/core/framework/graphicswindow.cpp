#include "graphicswindow.h"

#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QRectF>
#include <QGraphicsLinearLayout>
#include <QGraphicsLayoutItem>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsItemGroup>
#include <QDebug>

#include <debug/cauv_debug.h>

using namespace cauv;
using namespace cauv::gui;


struct Cross : public QGraphicsLineItem {
    Cross(qreal size) : QGraphicsLineItem(0, 0, size, size) {
        m_line = new QGraphicsLineItem(0, size, size, 0, this);
    }

    void setPen(const QPen &pen){
        m_line->setPen(pen);
        QGraphicsLineItem::setPen(pen);
    }

    void setSize(qreal size){
        m_line->setLine(0, size, size, 0);
        setLine(0, 0, size, size);
    }

private:
    void setLine(qreal x1, qreal y1, qreal x2, qreal y2){
        QGraphicsLineItem::setLine(x1, y1, x2, y2);
    }
protected:
    QGraphicsLineItem * m_line;
};


GraphicsWindowButton::GraphicsWindowButton(QGraphicsItem * item, qreal width, qreal height) :
        m_item(item), m_background(new QGraphicsEllipseItem(0, 0, width, height, this)) {

    m_background->setBrush(QBrush(Qt::white));
    item->setParentItem(m_background);

    this->setHandlesChildEvents(true);
    this->setAcceptHoverEvents(true);
}

GraphicsWindowButton::~GraphicsWindowButton() {
    debug(2) << "~GraphicsWindowButton()";
}

QSizeF GraphicsWindowButton::sizeHint(Qt::SizeHint, const QSizeF &) const {
    return m_background->rect().size();
}

void GraphicsWindowButton::setPen(QPen const& pen){
    m_background->setPen(pen);
}

void GraphicsWindowButton::hoverEnterEvent(QGraphicsSceneHoverEvent *){
    setCursor(Qt::PointingHandCursor);
    m_background->setBrush(QBrush(m_background->brush().color().darker(103)));
    update(boundingRect());
}

void GraphicsWindowButton::hoverLeaveEvent(QGraphicsSceneHoverEvent *){
    setCursor(Qt::PointingHandCursor);
    m_background->setBrush(QBrush(m_background->brush().color().lighter(103)));
    update(boundingRect());
}

void GraphicsWindowButton::mousePressEvent(QGraphicsSceneMouseEvent *){
    m_background->setBrush(QBrush(m_background->brush().color().darker(105)));
    update(boundingRect());
}

void GraphicsWindowButton::mouseReleaseEvent(QGraphicsSceneMouseEvent *){
    m_background->setBrush(QBrush(m_background->brush().color().lighter(105)));

    Q_EMIT pressed();
}


GraphicsWindow::GraphicsWindow(QGraphicsItem *parent) :
        QGraphicsObject(parent), m_cornerRadius(10),
        m_backgroundPen(QPen(QColor(190, 190, 190))),
        m_backgroundBrush(QColor(238, 238, 238)),
        m_buttonsWidget(new QGraphicsWidget(this)),
        m_layout(new QGraphicsLinearLayout(Qt::Horizontal))
{
    m_buttonsWidget->setLayout(m_layout);
    m_layout->addStretch(100);

    // close button
    Cross * cross = new Cross(8);
    cross->setPos(6, 6);
    QPen pen (m_backgroundPen.color().darker(105));
    pen.setCapStyle(Qt::RoundCap);
    pen.setWidthF(2.2);
    cross->setPen(pen);
    m_closeButton = new GraphicsWindowButton(cross);
    addButton(m_closeButton);
    connect(m_closeButton, SIGNAL(pressed()), this, SLOT(close()));

    this->setFlag((QGraphicsItem::ItemIsMovable));
    this->setFlag((QGraphicsItem::ItemIsSelectable));

    setSize(QSizeF(200, 200));
    setCursor(Qt::ArrowCursor);
}

GraphicsWindow::~GraphicsWindow(){
    debug(2) << "~GraphicsWindow()";
}

void GraphicsWindow::addButton(GraphicsWindowButton * button){
    button->setPen(m_backgroundPen);
    m_layout->addItem(button);
}

QSizeF GraphicsWindow::size() const{
    return m_size;
}

void GraphicsWindow::setSize(QSizeF size){
    if(size.width()<150)
        size.setWidth(150);
    prepareGeometryChange();
    m_size = size;
    m_buttonsWidget->setGeometry((cornerRadius()/2), -17,
                                 size.width()-(cornerRadius()/2), 30);
    update(boundingRect());
}

qreal GraphicsWindow::cornerRadius() const{
    return m_cornerRadius;
}

void GraphicsWindow::setCornerRadius(qreal radius){
    m_cornerRadius = radius;
    update(boundingRect());
}

QPen GraphicsWindow::backgroundPen() const{
    return m_backgroundPen;
}

void GraphicsWindow::setBackgroundPen(QPen pen){
    m_backgroundPen = pen;
    update(boundingRect());
}

QBrush GraphicsWindow::backgroundBrush() const{
    return m_backgroundBrush;
}

void GraphicsWindow::setBackgroundBrush(QBrush brush){
    m_backgroundBrush = brush;
    update(boundingRect());
}

void GraphicsWindow::setClosable(bool close){
    m_closeButton->setVisible(close);
}

/*
void GraphicsWindow::mousePressEvent(QGraphicsSceneMouseEvent *event){
    QGraphicsItem::mousePressEvent(event);
}

void GraphicsWindow::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    QGraphicsItem::mouseReleaseEvent(event);
}*/

QRectF GraphicsWindow::boundingRect() const{
    return QRectF(QPointF(0,0), size());
}

void GraphicsWindow::paint(QPainter *painter, const QStyleOptionGraphicsItem * option, QWidget *){

    QBrush brush = backgroundBrush();
    if(isSelected()){
        brush.setColor(brush.color().darker(102));
    }
    
    painter->setPen(backgroundPen());
    painter->setBrush(brush);
    painter->drawRoundedRect(boundingRect(), m_cornerRadius, m_cornerRadius, Qt::AbsoluteSize);

}

void GraphicsWindow::close(){
    Q_EMIT closed(this);
    this->deleteLater();
}
