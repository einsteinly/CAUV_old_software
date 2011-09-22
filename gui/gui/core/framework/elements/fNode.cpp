#include "fnode.h"

#include <set>

#include <common/cauv_utils.h>

#include "style.h"
#include "nodeInput.h"
#include "button.h"
#include "nodeHeader.h"

using cauv::gui::FNode;

FNode::FNode(Manager& m, QGraphicsItem *parent)
    : GraphicsWindow(F_Node_Style, parent),
      ManagedElement(m){

    Button *collapsebutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/collapse_button"), NULL, this
    );
    m_header->addButton("collapse", collapsebutton);

    Button *execbutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/reexec_button"), NULL, this
    );
    m_header->addButton("exec", execbutton);
    
    Button *dupbutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/dup_button"), NULL, this
    );
    m_header->addButton("duplicate", dupbutton);
    

    setSize(QSizeF(104,130));

    //!!!
    (new NodeInput(m_style, NodeIOType::Image, true, this))->setPos(0,40);
    (new NodeInput(m_style, NodeIOType::Image, false, this))->setPos(0,54);
    (new NodeInput(m_style, NodeIOType::Parameter, true, this))->setPos(0,68);
    (new NodeInput(m_style, NodeIOType::Parameter, false, this))->setPos(0,82);
}

void FNode::layoutChanged(){
    // since this's geometry hangs off m_back:
    this->prepareGeometryChange();

    QPainterPath p(QPointF(m_style.tl_radius, 0));
    
    const qreal width = size().width();
    const qreal height = size().height();
    
    // !!!
    std::set<qreal> inputs_at;
    inputs_at.insert(40);
    inputs_at.insert(54);
    inputs_at.insert(68);
    inputs_at.insert(82);

    p.lineTo(width, 0);
    p.lineTo(width, height);
    p.lineTo(m_style.bl_radius, height);
    p.arcTo(
        QRectF(0, height - m_style.bl_radius,
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

    m_back->setPen(m_style.pen);
    m_back->setBrush(m_style.brush);
    
    m_back->setPath(p);
}

