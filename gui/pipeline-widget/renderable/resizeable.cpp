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

#include "resizeable.h"

#include <QtOpenGL>

#include <debug/cauv_debug.h>

#include "../container.h"

using namespace cauv::pw;

const static BBox Handle_Size(-12, 0, 0, 12);

Resizeable::Resizeable(container_ptr_t c, BBox const& s, BBox const& min, BBox
                       const& max, double const& fixed_aspect)
    : Renderable(c), m_bbox(s), m_click_offset(), m_resizing(false),
      m_min_bbox(min), m_max_bbox(max), m_aspect(fixed_aspect){
}

void Resizeable::mouseMoveEvent(MouseEvent const& event){
    debug(2) << "Resizeable::mouseMoveEvent(" << event.pos << ")";
    if(m_resizing && (event.buttons & Qt::LeftButton)){
        m_bbox.max.x = (event.pos - m_click_offset).x;
        m_bbox.min.y = (event.pos - m_click_offset).y;

        if(m_bbox.w() > m_max_bbox.w()) m_bbox.max.x = m_bbox.min.x + m_max_bbox.w();
        if(m_bbox.h() > m_max_bbox.h()) m_bbox.min.y = m_bbox.max.y - m_max_bbox.h();
        if(m_bbox.w() < m_min_bbox.w()) m_bbox.max.x = m_bbox.min.x + m_min_bbox.w();
        if(m_bbox.h() < m_min_bbox.h()) m_bbox.min.y = m_bbox.max.y - m_min_bbox.h();
        
        if(m_aspect > 0){
            if(m_bbox.w() / m_bbox.h() > m_aspect)
                m_bbox.min.y = m_bbox.max.y - m_bbox.w() / m_aspect;
            else
                m_bbox.max.x = m_bbox.min.x + m_bbox.h() * m_aspect;
        }

        // need a re-layout
        m_context->refreshLayout();
    }
}

bool Resizeable::mousePressEvent(MouseEvent const& event){
    debug(2) << "Resizeable::mousePressEvent(" << event.pos << ")";
    Point blc = Point(m_bbox.max.x, m_bbox.min.y);
    if((Handle_Size + blc).contains(event.pos)){
        m_click_offset = event.pos - blc;
        m_resizing = true;
        return true;
    }
    return false;
}

void Resizeable::mouseReleaseEvent(MouseEvent const&){
    m_resizing = false;
}

BBox Resizeable::bbox(){
    return m_bbox;
}

void Resizeable::drawHandle(){
    glLineWidth(1);
    glColor(Colour(1, 0.5));
    glTranslatef(0, 0, 0.1);
    const Point bl(m_bbox.max.x, m_bbox.min.y);
    const int bars = 3;
    const int i_inc = Handle_Size.w() / bars;
    const int j_inc = Handle_Size.h() / bars;
    glBegin(GL_LINES);
    for(int i=i_inc, j=j_inc; i<=Handle_Size.w() && j<=Handle_Size.h(); i+=i_inc, j+=j_inc){
        glVertex(bl + Point(-i, 1));
        glVertex(bl + Point(-1, j));
    }
    glEnd();
}

void Resizeable::aspect(double const& a){
    m_aspect = a;
    m_bbox.min.y = m_bbox.max.y - m_bbox.w() / a;
    m_context->refreshLayout();
}


