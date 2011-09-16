#include "node.h"

#include <set>

#include <common/cauv_utils.h>

#include "style.h"

using namespace cauv;
using namespace cauv::gui;

Node::Node(NodeStyle const& style)
    : QObject(),
      QGraphicsPathItem(),
      m_style(style){
    
    setFlag(ItemIsMovable);

    updateLayout();
}


void Node::updateLayout(){
    prepareGeometryChange();
    
    setPen(m_style.pen);
    setBrush(m_style.brush);

    QPainterPath p(QPointF(m_style.tl_radius, 0));
    
    const qreal fake_width = 80;
    const qreal fake_height = 110;

    std::set<qreal> inputs_at;
    inputs_at.insert(40);
    inputs_at.insert(60);
    inputs_at.insert(80);

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
    
    setPath(p);
}


