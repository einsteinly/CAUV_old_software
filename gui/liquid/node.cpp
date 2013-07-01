/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "node.h"

#include <set>
#include <algorithm>

#include <boost/range/adaptor/reversed.hpp>

#include <QGraphicsPathItem>
#include <QGraphicsLayoutItem>
#include <QGraphicsLinearLayout>
#include <QGraphicsSceneMouseEvent>

#include <debug/cauv_debug.h>
#include <utility/qt_streamops.h>

#include "requiresCutout.h"
#include "style.h"
#include "button.h"
#include "nodeHeader.h"
#include "resize.h"
#include "shadow.h"
#include "itemFridge.h"

using namespace liquid;

QSizeF LiquidNode::Minimum_Size = QSizeF(100, 40);

LiquidNode::LiquidNode(NodeStyle const& style, QGraphicsItem *parent)
    : QGraphicsObject(parent),
      m_size(Minimum_Size),
      m_header(new ItemFridge<NodeHeader>(style, this)),
      //m_buttonsWidget(),
      m_contentWidget(new QGraphicsWidget(this)),
      m_contentLayout(new QGraphicsLinearLayout(Qt::Vertical)),
      m_back(new QGraphicsPathItem(this)),
      m_status_highlight(new Shadow(this)),
      m_items_requiring_cutout(),      
      m_style(style),
      m_status(OK){

    setFlag(ItemIsMovable);
    setFlag(ItemHasNoContents);

    m_back->setFlag(ItemStacksBehindParent);
    m_back->setZValue(10);
    
    m_contentLayout->setSpacing(0);
    m_contentLayout->setContentsMargins(1,1,1,1);
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

    m_status_highlight->setFlag(ItemStacksBehindParent);
    m_status_highlight->setZValue(-100);

    Button *close_button = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/x_button"), nullptr, this
    );
    addButton("close", close_button);

    connect(close_button, SIGNAL(pressed()), this, SLOT(close()));
    connect(m_contentWidget, SIGNAL(geometryChanged()), this, SLOT(updateLayout()));

    status(OK);

    setSize(Minimum_Size);
    updateLayout();

    setCacheMode(DeviceCoordinateCache);
    m_contentWidget->setCacheMode(DeviceCoordinateCache);
    m_back->setCacheMode(DeviceCoordinateCache);

#ifdef QT_PROFILE_GRAPHICSSCENE
    setProfileName("liquid::LiquidNode");
    m_contentWidget->setProfileName("liquid::LiquidNode::contentWidget");
    m_back->setProfileName("liquid::LiquidNode::back");
#endif // def QT_PROFILE_GRAPHICSSCENE
}

LiquidNode::~LiquidNode(){
    debug(7) << "~LiquidNode()" << this;
}

void LiquidNode::close(){
    Q_EMIT closed(this);
    deleteLater();
}

QRectF LiquidNode::boundingRect() const{
    // otherwise 'this' would never receive mouse events, and it needs to in
    // order to be movable
    return m_back->boundingRect();
}

QPainterPath LiquidNode::shape() const{
    return m_back->shape();
}

void LiquidNode::paint(QPainter* p, const QStyleOptionGraphicsItem* o, QWidget *w){
    // don't draw anything, ItemHasNoContents flag is set
    Q_UNUSED(p);
    Q_UNUSED(o);
    Q_UNUSED(w);
}

void LiquidNode::mouseMoveEvent(QGraphicsSceneMouseEvent* event){
    if((event->buttons() & Qt::LeftButton) && (flags() & ItemIsMovable)){
        QPointF newPos(mapToParent(event->pos()) - matrix().map(event->buttonDownPos(Qt::LeftButton)));
        // enforce whole-px coordinates
        setPos(QPointF(newPos.toPoint()));
    }else{
        event->ignore();
    }
}

void LiquidNode::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event){
    Q_UNUSED(event);
    Q_EMIT doubleClicked();
}

void LiquidNode::addButton(QString name, Button *button){
    m_header->addButton(name, button);
    m_header->freeze();
}

void LiquidNode::addItem(QGraphicsLayoutItem *item){
    m_contentLayout->addItem(item);
    // Documentation suggests that adding things to the layout should also mean
    // they get reparented to the widget of the layout (m_contentWidget), but
    // this doesn't seem to happen....
    QGraphicsItem *as_qgi = dynamic_cast<QGraphicsItem*>(item);
    if(as_qgi)
        as_qgi->setParentItem(m_contentWidget);

    RequiresCutout *req_cutout = dynamic_cast<RequiresCutout*>(item);
    debug(7) << "addItem:: requires cutout = " << req_cutout;
    if(req_cutout){
        // this ugly signal connection is necessary because if the cutout
        // positions depend on the layout of the contents of a widget added as
        // an item, then if the layout changes we need to change where the
        // cutouts are:
        QGraphicsWidget* as_widget = dynamic_cast<QGraphicsWidget*>(item);
        if(as_widget)
            connect(as_widget, SIGNAL(geometryChanged()), this, SLOT(updateLayout()));
        m_items_requiring_cutout << req_cutout;
    }
}

void LiquidNode::removeItem(QGraphicsLayoutItem *item){
    m_contentLayout->removeItem(item);
    RequiresCutout *req_cutout = dynamic_cast<RequiresCutout*>(item);
    if(req_cutout)
        // one should be equivalent to (but twice as fast as) removeAll
        m_items_requiring_cutout.removeOne(req_cutout);
    // removing things from a layout passes ownership, so make sure things are
    // safely deleted:
    QObject *as_qo = dynamic_cast<QObject*>(item);
    if(as_qo)
        as_qo->deleteLater();
    else
        delete item;
}

QSizeF LiquidNode::size() const{
    return m_size;
}

void LiquidNode::setSize(QSizeF const& size){
    debug(6) << "LiquidNode::setSize(" << size << ")";

    // this will cause updateLayout to be called via the geometryChanged
    // signal, which in turn actually sets the new size
    const double header_height = m_style.header.height + m_style.bl_radius/2;
    const double header_width = m_header->minimumWidth();
    m_contentWidget->setGeometry(
        0, m_style.header.height,
        std::max(size.width(), header_width), std::max(0.0, size.height() - header_height)
    );
    //m_buttonsWidget->setGeometry((cornerRadius()/2), -17,
    //                             size.width()-(cornerRadius()/2), 30);
}

void LiquidNode::resized(){
    if(m_resizeHandle->isVisible()){
        debug(6) << "LiquidNode::resized()" << m_resizeHandle->newSize();
        QSizeF newSize = Minimum_Size;
        if(m_resizeHandle->newSize().width() >= newSize.width())
            newSize.setWidth(m_resizeHandle->newSize().width());
        if(m_resizeHandle->newSize().height() >= newSize.height())
            newSize.setHeight(m_resizeHandle->newSize().height());
        setSize(newSize);
    }else{
        debug(6) << "LiquidNode::resized() NOT resized: handle not active";
    }
}


void LiquidNode::setClosable(bool close){
    Button *closebutton = m_header->getButton("close");
    if(closebutton){
        if(close)
            closebutton->show();
        else
            closebutton->hide();
    }
    m_header->thaw();
    m_header->freeze();
}

void LiquidNode::setResizable(bool sizeable){
    m_resizeHandle->setVisible(sizeable);
}

void LiquidNode::layoutChanged(){
    QPainterPath p(QPointF(m_style.tl_radius, 0));

    const qreal width = size().width();
    const qreal height = size().height();
    
    std::map<qreal,CutoutStyle> cutouts_at;
    for(RequiresCutout* r : m_items_requiring_cutout)
        for(CutoutStyle const& g : r->cutoutGeometry())
            cutouts_at[m_contentWidget->pos().y() +
                       r->asQGI()->pos().y() +
                       g.main_cutout.y_offset] = g;
    debug(5) << "layoutChanged:" << cutouts_at.size() << "cutouts";

    p.lineTo(width, 0);
    p.lineTo(width, height);
    p.lineTo(m_style.bl_radius, height);
    p.arcTo(
        QRectF(0, height - m_style.bl_radius,
               m_style.bl_radius, m_style.bl_radius),
        -90, -90
    );
    
    // relying on iteration order of map being the order of the keys
    typedef std::pair<qreal, CutoutStyle> y_style_pair_t;
    for(const y_style_pair_t& yg : boost::adaptors::reverse(cutouts_at)){
        CutoutStyle::CutoutGeometry const& co = yg.second.main_cutout;
        debug(8) << "Cutout y position:" << yg.first;
        p.lineTo(0, yg.first + co.cutout_base/2);
        p.lineTo(co.cutout_depth, yg.first + co.cutout_tip/2);
        p.lineTo(co.cutout_depth, yg.first - co.cutout_tip/2);
        p.lineTo(0, yg.first - co.cutout_base/2);
    }

    p.lineTo(0, m_style.tl_radius);
    p.arcTo(QRectF(0, 0, m_style.tl_radius, m_style.tl_radius), -180, -90);
    p.lineTo(m_style.tl_radius, 0);

    m_back->setPen(m_style.pen);
    m_back->setBrush(m_style.brush);

    // since this's geometry hangs off m_back:
    this->prepareGeometryChange();

    // translate moves the path to the centre of pixels (pos() is forced to be
    // an integer in mouseMoveEvent)
    p.translate(0.5,0.5);
    m_back->setPath(p);
    m_status_highlight->setShape(m_back->shape());
}

void LiquidNode::updateLayout(){
    m_contentLayout->invalidate();
    setSizeFromContents();
    layoutChanged();
}

void LiquidNode::setSizeFromContents(){
    debug() << "LiquidNode::setSizeFromContents(" << m_contentWidget->size()
            << "(hint:" <<  m_contentWidget->size() << "))";

    const float header_height = m_style.header.height + m_style.bl_radius/2;
    const double header_min_width = m_header->minimumWidth();
    m_size.setWidth(std::max(m_contentWidget->size().width(), header_min_width));
    m_size.setHeight(m_contentWidget->size().height() + header_height);

    if(m_size.width() < Minimum_Size.width())
        m_size.setWidth(Minimum_Size.width());
    if(m_size.height() < Minimum_Size.height())
        m_size.setHeight(Minimum_Size.height());

    m_resizeHandle->setX(m_size.width() - m_resizeHandle->size().width());
    m_resizeHandle->setY(m_size.height() - m_resizeHandle->size().height());
    
    m_header->setWidth(m_size.width());
    m_header->freeze();
}

NodeStyle LiquidNode::style() const{
    return m_style;
}

void LiquidNode::setTitle(QString text){
    m_header->setTitle(text);
    m_header->freeze();
}

void LiquidNode::setInfo(QString text){
    m_header->setInfo(text);
    m_header->freeze();
}

LiquidNode::Status LiquidNode::status() const{
    return m_status;
}

void LiquidNode::status(Status const& s, const std::string& status_information){
    m_status = s;
    setInfo(QString::fromStdString(status_information));
    if(s == OK){
        m_status_highlight->setBrush(QBrush(Qt::NoBrush));
    }else{
        m_status_highlight->setBrush(QBrush(QColor(180,0,0,200)));
    }
}

