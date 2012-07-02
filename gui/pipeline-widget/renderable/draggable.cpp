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

#include "draggable.h"
#include "../container.h"

using namespace cauv::gui::pw;

Draggable::Draggable(container_ptr_t c)
    : Renderable(c), m_click_pos(),
      m_mouseover(false), m_pressed(false){
}

void Draggable::mouseMoveEvent(MouseEvent const& event){
    if(m_pressed && (event.buttons & Qt::LeftButton)){
        m_pos += event.pos - m_click_pos;
        // need a re-draw
        m_context->postRedraw(0);
    }else if(!m_mouseover){
        m_mouseover = true;
        m_context->postRedraw(0);
    }
}

bool Draggable::mousePressEvent(MouseEvent const& event){
    m_click_pos = event.pos;
    m_pressed = true;
    return true;
}

void Draggable::mouseGoneEvent(){
    if(m_mouseover || m_pressed){
        m_mouseover = false;
        m_pressed = false;
        m_context->postRedraw(0);
    }
}


