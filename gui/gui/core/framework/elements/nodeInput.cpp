#include "nodeInput.h"

#include "style.h"

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
}

QList<liquid::CutoutStyle> NodeInput::cutoutGeometry() const{
    QList<liquid::CutoutStyle> r;
    // !!! TODO: depends on image/param/required/optional
    return r << Required_Image_Input;
}


QSizeF NodeInput::sizeHint(Qt::SizeHint which, const QSizeF& constraint) const{
    // !!! TODO
    return QSizeF(100, 16);
}

