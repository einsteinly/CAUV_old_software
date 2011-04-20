#include "mouseEvent.h"

#include <QGraphicsSceneMouseEvent>

#include <debug/cauv_debug.h>

#include "pipelineWidget.h"
#include "renderable.h"
#include "util.h"

using namespace cauv::pw;

MouseEvent::MouseEvent(QGraphicsSceneMouseEvent* qm, PipelineWidget const& p)
    : pos((qm->pos().x() - p.width()/2) / p.m_pixels_per_unit - p.m_win_centre.x,
          (p.height()/2 - qm->pos().y()) / p.m_pixels_per_unit - p.m_win_centre.y),
      buttons(qm->buttons()){
}
    

MouseEvent::MouseEvent(QGraphicsSceneMouseEvent* qm,
                       boost::shared_ptr<Renderable> r,
                       PipelineWidget const& p)
    : pos((qm->pos().x() - p.width()/2) / p.m_pixels_per_unit - p.m_win_centre.x - r->m_pos.x,
          (p.height()/2 - qm->pos().y()) / p.m_pixels_per_unit - p.m_win_centre.y - r->m_pos.y),
      buttons(qm->buttons()){
      debug(2) << "MouseEvent constructed: p=" << pos
                << "qm: x=" << qm->screenPos().x() << "y=" << qm->screenPos().y()
                << "wc: x=" << p.m_win_centre.x << "y=" << p.m_win_centre.y;
}


MouseEvent::MouseEvent(PipelineWidget const& p)
    : pos((p.m_last_mouse_pos.x() - p.width()/2) / p.m_pixels_per_unit - p.m_win_centre.x,
          (p.height()/2 - p.m_last_mouse_pos.y()) / p.m_pixels_per_unit - p.m_win_centre.y),
      buttons(0){
}

MouseEvent::MouseEvent(boost::shared_ptr<Renderable> r,
                       PipelineWidget const& p)
    : pos((p.m_last_mouse_pos.x() - p.width()/2) / p.m_pixels_per_unit - p.m_win_centre.x - r->m_pos.x,
          (p.height()/2 - p.m_last_mouse_pos.y()) / p.m_pixels_per_unit - p.m_win_centre.y - r->m_pos.y),
      buttons(0){
}

MouseEvent::MouseEvent(MouseEvent const& m,
                       boost::shared_ptr<Renderable> r)
    : pos(m.pos.x - r->m_pos.x, m.pos.y - r->m_pos.y),
      buttons(m.buttons){
}


