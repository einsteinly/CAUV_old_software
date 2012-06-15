/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "nodeInput.h"

#include "style.h"

#include "debug/cauv_debug.h"
#include "utility/qt_streamops.h"

using namespace cauv;
using namespace cauv::gui;

/*
static QPainterPath pathForGeometry(liquid::NodeStyle const& s,
                                    liquid::NodeStyle::Input::Geometry const& g){
    QPainterPath p;

    p.moveTo(QPointF(0, -s.in_socket_cutout_base/2));
    p.lineTo(QPointF(0, -g.cutout_base/2));
    p.lineTo(QPointF(g.cutout_depth, -g.cutout_tip/2));
    p.lineTo(QPointF(g.cutout_depth,  g.cutout_tip/2));
    p.lineTo(QPointF(0,  g.cutout_base/2));
    p.lineTo(QPointF(0,  s.in_socket_cutout_base/2));

    p.lineTo(QPointF(s.in_socket_cutout_depth,  s.in_socket_cutout_base/2));
    p.lineTo(QPointF(s.in_socket_cutout_depth, -s.in_socket_cutout_base/2));

    return p;
}
*/

NodeInput::NodeInput(liquid::NodeStyle const& style,
                     NodeIOType::e const& type,
                     bool required,
                     QGraphicsItem *parent)
    : QGraphicsPathItem(parent),
      m_type(type),
      m_required(required),
      m_style(style){
    
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    /*
    setFlag(ItemStacksBehindParent);
    
    if(type == NodeIOType::Image){
        setPen(m_style.InputStyle.Image.pen);
        setBrush(m_style.InputStyle.Image.brush);
    }else{
        setPen(m_style.InputStyle.Param.pen);
        setBrush(m_style.InputStyle.Param.brush);
    }

    liquid::NodeStyle::Input::Geometry geom = (required)? m_style.InputStyle.Required:
                                                  m_style.InputStyle.Optional;
    setPath(pathForGeometry(m_style, geom));*/

    QPainterPath p;
    m_rect = QRectF(
        0,
        -Required_Image_Input().main_cutout.cutout_base/2,
        Required_Image_Input().main_cutout.cutout_depth,
        Required_Image_Input().main_cutout.cutout_base
    );
    p.addRect(m_rect);
    setPath(p);
    setPen(QPen(QColor(20,160,20,64)));
}

QList<liquid::CutoutStyle> NodeInput::cutoutGeometry() const{
    QList<liquid::CutoutStyle> r;
    // !!! TODO: depends on image/param/required/optional
    return r << Required_Image_Input;
}

void NodeInput::setGeometry(QRectF const& rect){
    debug() << "nodeInput::setGeometry" << rect;
    debug() << "current position:" << pos();
    setPos(rect.topLeft() - m_rect.topLeft());
}

QSizeF NodeInput::sizeHint(Qt::SizeHint which, const QSizeF& constraint) const{
    Q_UNUSED(which);
    Q_UNUSED(constraint);
    return m_rect.size();
}

