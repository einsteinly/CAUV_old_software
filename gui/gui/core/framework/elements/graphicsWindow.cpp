#include "graphicsWindow.h"

#include <set>

#include <QGraphicsPathItem>
#include <QGraphicsLayoutItem>
#include <QGraphicsLinearLayout>

#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>
#include <utility/qt_streamops.h>

#include "style.h"
#include "button.h"
#include "nodeHeader.h"
#include "resize.h"

using namespace cauv;
using namespace cauv::gui;

GraphicsWindow::GraphicsWindow(NodeStyle const& style, QGraphicsItem *parent)
    : QGraphicsObject(parent),
      m_size(QSizeF(0,0)),
      m_header(new NodeHeader(style, this)),
      m_contentWidget(new QGraphicsWidget(this)),
      m_contentLayout(new QGraphicsLinearLayout(Qt::Vertical)),
      m_style(style),
      m_back(new QGraphicsPathItem(this)){
    
    setFlag(ItemIsMovable);
    setFlag(ItemHasNoContents);

    m_back->setFlag(ItemStacksBehindParent);
    m_back->setZValue(10);
    
    m_contentLayout->setSpacing(0);
    m_contentWidget->setLayout(m_contentLayout);
    m_contentWidget->setPos(0, m_style.header.height);

    // resize handle
    m_resizeHandle = new ResizeHandle(this);
    m_resizeHandle->connect(m_resizeHandle, SIGNAL(xChanged()), this, SLOT(resized()));
    m_resizeHandle->connect(m_resizeHandle, SIGNAL(yChanged()), this, SLOT(resized()));
    m_resizeHandle->setX(size().width());
    m_resizeHandle->setY(size().height());
    m_resizeHandle->setVisible(false);
    m_resizeHandle->setZValue(100);
    m_resizeHandle->setPen(m_style.resize_handle_pen);


    Button* close_button = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/x_button"), NULL, this
    );
    m_header->addButton("close", close_button);

    connect(close_button, SIGNAL(pressed()), this, SLOT(close()));
    
    updateLayout();
}

void GraphicsWindow::close(){
    Q_EMIT closed(this);
    this->deleteLater();
}

QRectF GraphicsWindow::boundingRect() const{
    // otherwise this's would never receive mouse events, and it needs to in
    // order to be movable
    return m_back->boundingRect();
}

void GraphicsWindow::paint(QPainter* p, const QStyleOptionGraphicsItem* o, QWidget *w){
    // don't draw anything, ItemHasNoContents flag is set
    Q_UNUSED(p);
    Q_UNUSED(o);
    Q_UNUSED(w);
}


void GraphicsWindow::addButton(Button *button){
    // !!! TODO
    //m_layout->addItem(button);
}

void GraphicsWindow::addItem(QGraphicsLayoutItem *item){
    m_contentLayout->addItem(item);
}

QSizeF GraphicsWindow::size() const{
    return m_size;
}

void GraphicsWindow::setSize(QSizeF const& size){
    debug(5) << "GraphicsWind::setSize(" << size << ")";
    prepareGeometryChange();
    m_size = size;
    //m_buttonsWidget->setGeometry((cornerRadius()/2), -17,
    //                             size.width()-(cornerRadius()/2), 30);
    m_contentWidget->setGeometry(
        0, m_style.header.height,
        size.width(), size.height()-(m_style.header.height + m_style.bl_radius/2)
    );

    m_resizeHandle->setX(size.width() - m_resizeHandle->size().width());
    m_resizeHandle->setY(size.height() - m_resizeHandle->size().height());

    m_header->setWidth(size.width());
    updateLayout();
}

void GraphicsWindow::resized(){
    debug(5) << "GraphicsWind::resized()" << m_resizeHandle->newSize();
    QSizeF newSize(100, 100);
    if(m_resizeHandle->newSize().width() >= newSize.width())
        newSize.setWidth(m_resizeHandle->newSize().width());
    if(m_resizeHandle->newSize().height() >= newSize.height())
        newSize.setHeight(m_resizeHandle->newSize().height());
    setSize(newSize);
}


void GraphicsWindow::setClosable(bool close){
    // !! TODO
    //m_closeButton->setVisible(close);
}

void GraphicsWindow::setResizable(bool sizeable){
    m_resizeHandle->setVisible(sizeable);
}

void GraphicsWindow::layoutChanged(){
    // since this's geometry hangs off m_back:
    this->prepareGeometryChange();

    QPainterPath p(QPointF(m_style.tl_radius, 0));
    
    const qreal width = size().width();
    const qreal height = size().height();

    p.lineTo(width, 0);
    p.lineTo(width, height);
    p.lineTo(m_style.bl_radius, height);
    p.arcTo(
        QRectF(0, height - m_style.bl_radius,
               m_style.bl_radius, m_style.bl_radius),
        -90, -90
    );
    p.lineTo(0, m_style.tl_radius);
    p.arcTo(QRectF(0, 0, m_style.tl_radius, m_style.tl_radius), -180, -90);
    p.lineTo(m_style.tl_radius, 0);

    m_back->setPen(m_style.pen);
    m_back->setBrush(m_style.brush);
    
    m_back->setPath(p);
}

void GraphicsWindow::updateLayout(){
    layoutChanged();
}


