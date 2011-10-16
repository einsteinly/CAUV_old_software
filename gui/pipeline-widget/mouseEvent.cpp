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

#include "mouseEvent.h"

#include <QMouseEvent>

#include <debug/cauv_debug.h>

#include "pipelineWidget.h"
#include "renderable.h"
#include "util.h"

using namespace cauv::pw;

MouseEvent::MouseEvent(QMouseEvent* qm, PipelineWidget const& p)
    : pos((qm->x() - p.width()/2) / p.m_pixels_per_unit - p.m_win_centre.x,
          (p.height()/2 - qm->y()) / p.m_pixels_per_unit - p.m_win_centre.y),
      buttons(qm->buttons()){
}
    

MouseEvent::MouseEvent(QMouseEvent* qm,
                       boost::shared_ptr<Renderable> r,
                       PipelineWidget const& p)
    : pos((qm->x() - p.width()/2) / p.m_pixels_per_unit - p.m_win_centre.x - r->m_pos.x,
          (p.height()/2 - qm->y()) / p.m_pixels_per_unit - p.m_win_centre.y - r->m_pos.y),
      buttons(qm->buttons()){
      debug(2) << "MouseEvent constructed: p=" << pos
                << "qm: x=" << qm->x() << "y=" << qm->y()
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


