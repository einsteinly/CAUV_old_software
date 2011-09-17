#include "node.h"

#include <set>

#include <QGraphicsPathItem>

#include <common/cauv_utils.h>

#include "style.h"
#include "nodeInput.h"
#include "button.h"

using namespace cauv;
using namespace cauv::gui;

Node::Node(NodeStyle const& style)
    : QGraphicsObject(),
      m_back(new QGraphicsPathItem(this)),
      m_style(style){
    
    setFlag(ItemIsMovable);
    setFlag(ItemHasNoContents);

    //!!!
    (new NodeInput(m_style, NodeIOType::Image, true, this))->setPos(0,40);
    (new NodeInput(m_style, NodeIOType::Image, false, this))->setPos(0,54);
    (new NodeInput(m_style, NodeIOType::Parameter, true, this))->setPos(0,68);
    (new NodeInput(m_style, NodeIOType::Parameter, false, this))->setPos(0,82);

    Button *b = new Button(
        QRectF(0,0,24,24),
        QString(":/resources/icons/dup_button"),
        this
    );
    b->setPos(20, 2);

    updateLayout();
}

QRectF Node::boundingRect() const{
    // otherwise this's would never receive mouse events, and it needs to in
    // order to be movable
    return m_back->boundingRect();
}

void Node::paint(QPainter* p, const QStyleOptionGraphicsItem* o, QWidget *w){
    // don't draw anything, ItemHasNoContents flag is set
    Q_UNUSED(p);
    Q_UNUSED(o);
    Q_UNUSED(w);
}

void Node::updateLayout(){
    // since this's geometry hangs off m_back:
    this->prepareGeometryChange();
    
    m_back->setPen(m_style.pen);
    m_back->setBrush(m_style.brush);

    QPainterPath p(QPointF(m_style.tl_radius, 0));
    
    const qreal fake_width = 80;
    const qreal fake_height = 110;
    
    // !!!
    std::set<qreal> inputs_at;
    inputs_at.insert(40);
    inputs_at.insert(54);
    inputs_at.insert(68);
    inputs_at.insert(82);

    p.lineTo(fake_width, 0);
    p.lineTo(fake_width, fake_height);
    p.lineTo(m_style.tl_radius, fake_height);
    p.arcTo(
        QRectF(0, fake_height - m_style.bl_radius,
               m_style.bl_radius, m_style.bl_radius),
        -90, -90
    );
    
    reverse_foreach(qreal y, inputs_at){
        p.lineTo(0, y + m_style.in_socket_cutout_base/2);
        p.lineTo(m_style.in_socket_cutout_depth, y + m_style.in_socket_cutout_tip/2);
        p.lineTo(m_style.in_socket_cutout_depth, y - m_style.in_socket_cutout_tip/2);
        p.lineTo(0, y - m_style.in_socket_cutout_base/2);
    }

    p.lineTo(0, m_style.tl_radius);
    p.arcTo(QRectF(0, 0, m_style.tl_radius, m_style.tl_radius), -180, -90);

    p.lineTo(m_style.tl_radius, 0);
    
    m_back->setPath(p);
}


