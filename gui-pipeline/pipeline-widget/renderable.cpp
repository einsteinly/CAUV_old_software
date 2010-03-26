#include "renderable.h"
#include "pipelineWidget.h"

#include <common/debug.h>

MouseEvent::MouseEvent(QMouseEvent* qm,
                       boost::shared_ptr<Renderable> r,
                       PipelineWidget const& p)
    : x((qm->x() - p.width()/2) / p.m_pixels_per_unit - p.m_win_centre_x - r->m_pos_x),
      y((p.height()/2 - qm->y()) / p.m_pixels_per_unit - p.m_win_centre_y - r->m_pos_y),
      buttons(qm->buttons()){
      debug(-1) << "MouseEvent constructed: x=" << x << "y=" << y
                << "qm: x=" << qm->x() << "y=" << qm->y()
                << "wc: x=" << p.m_win_centre_x << "y=" << p.m_win_centre_y;
}

MouseEvent::MouseEvent(MouseEvent const& m,
                       boost::shared_ptr<Renderable> r)
    : x(m.x - r->m_pos_x), y(m.y - r->m_pos_y),
      buttons(m.buttons){
}


Renderable::Renderable(PipelineWidget& p, double x, double y)
    : m_pos_x(x), m_pos_y(y), m_parent(p){
}


