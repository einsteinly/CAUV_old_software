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

#include "resize.h"
#include "elements/button.h"

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




GraphicsWindow::GraphicsWindow(QGraphicsItem *parent) :
        QGraphicsObject(parent), m_cornerRadius(10),
        m_backgroundPen(QPen(QColor(190, 190, 190))),
        m_backgroundBrush(QColor(238, 238, 238)),
        m_buttonsWidget(new QGraphicsWidget(this)),
        m_layout(new QGraphicsLinearLayout(Qt::Horizontal)),
        m_contentWidget(new QGraphicsWidget(this)),
        m_contentLayout(new QGraphicsLinearLayout(Qt::Vertical))
{
    m_buttonsWidget->setLayout(m_layout);
    m_layout->addStretch(100);
    m_contentWidget->setLayout(m_contentLayout);

    setSize(QSizeF(200, 200));
    setCursor(Qt::ArrowCursor);
    this->setFlag((QGraphicsItem::ItemIsMovable));
    this->setFlag((QGraphicsItem::ItemIsSelectable));
    this->connect(this, SIGNAL(xChanged()), this, SIGNAL(boundriesChanged()));
    this->connect(this, SIGNAL(yChanged()), this, SIGNAL(boundriesChanged()));

    // close button
    Cross * cross = new Cross(8);
    cross->setPos(6, 6);
    QPen pen (m_backgroundPen.color().darker(105));
    pen.setCapStyle(Qt::RoundCap);
    pen.setWidthF(2.2);
    cross->setPen(pen);
    m_closeButton = new Button(QRectF(0,0,20,20), cross);
    addButton(m_closeButton);
    connect(m_closeButton, SIGNAL(pressed()), this, SLOT(close()));

    // resize handle
    m_resizeHandle = new ResizeHandle(this);
    m_resizeHandle->connect(m_resizeHandle, SIGNAL(xChanged()), this, SLOT(resized()));
    m_resizeHandle->connect(m_resizeHandle, SIGNAL(yChanged()), this, SLOT(resized()));
    m_resizeHandle->setX(size().width());
    m_resizeHandle->setY(size().height());
    m_resizeHandle->setVisible(false);
    m_resizeHandle->setZValue(100);
    QPen resizePen(m_backgroundPen);
    resizePen.setWidth(2);
    resizePen.setCapStyle(Qt::RoundCap);
    m_resizeHandle->setPen(resizePen);

}

GraphicsWindow::~GraphicsWindow(){
    debug(2) << "~GraphicsWindow()";
}

void GraphicsWindow::addButton(Button * button){
    button->setPen(m_backgroundPen);
    m_layout->addItem(button);
}

void GraphicsWindow::addItem(QGraphicsLayoutItem * item){
    m_contentLayout->addItem(item);
}

QSizeF GraphicsWindow::size() const{
    return m_size;
}

void GraphicsWindow::setSize(QSizeF const& size){
    prepareGeometryChange();
    m_size = size;
    m_buttonsWidget->setGeometry((cornerRadius()/2), -17,
                                 size.width()-(cornerRadius()/2), 30);
    m_contentWidget->setGeometry(0, 5, size.width(), size.height()-(cornerRadius()/2));
    update(boundingRect());
    Q_EMIT boundriesChanged();
}

void GraphicsWindow::resized(){
    QSizeF newSize(150, 150);
    if(m_resizeHandle->newSize().width() >= 150){
        newSize.setWidth(m_resizeHandle->newSize().width());
    } else m_resizeHandle->setX(150 - m_resizeHandle->size().width());
    if(m_resizeHandle->newSize().height() >= 150){
        newSize.setHeight(m_resizeHandle->newSize().height());
    } else m_resizeHandle->setY(150 - m_resizeHandle->size().height());
    setSize(newSize);
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

void GraphicsWindow::setBackgroundPen(QPen const& pen){
    m_backgroundPen = pen;
    update(boundingRect());
}

QBrush GraphicsWindow::backgroundBrush() const{
    return m_backgroundBrush;
}

void GraphicsWindow::setBackgroundBrush(QBrush const& brush){
    m_backgroundBrush = brush;
    update(boundingRect());
}

void GraphicsWindow::setClosable(bool close){
    m_closeButton->setVisible(close);
}

void GraphicsWindow::setResizable(bool sizeable){
    m_resizeHandle->setVisible(sizeable);
}

QGraphicsObject * GraphicsWindow::asQGraphicsObject(){
    return this;
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

    Q_UNUSED(option);

    QBrush brush = backgroundBrush();
    if(isSelected()){
        brush.setColor(brush.color().darker(102));
    }
    
    painter->setPen(backgroundPen());
    painter->setBrush(brush);
    painter->drawRoundedRect(boundingRect(), m_cornerRadius, m_cornerRadius, Qt::AbsoluteSize);

}

void GraphicsWindow::close(){
    Q_EMIT disconnected();
    Q_EMIT closed(this);
    this->deleteLater();
}
