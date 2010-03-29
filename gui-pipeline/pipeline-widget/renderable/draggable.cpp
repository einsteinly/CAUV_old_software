#include "draggable.h"
#include "../container.h"

Draggable::Draggable(container_ptr_t c)
    : Renderable(c), m_click_pos(),
      m_mouseover(false), m_pressed(false){
}

void Draggable::mouseMoveEvent(MouseEvent const& event){
    if(m_pressed && (event.buttons & Qt::LeftButton)){
        m_pos += event.pos - m_click_pos;
        // need a re-draw
        m_context->postRedraw();
    }else if(!m_mouseover){
        m_mouseover = true;
        m_context->postRedraw();
    }
}

void Draggable::mousePressEvent(MouseEvent const& event){
    m_click_pos = event.pos;
    m_pressed = true;
}

void Draggable::mouseGoneEvent(){
    if(m_mouseover || m_pressed){
        m_mouseover = false;
        m_pressed = false;
        m_context->postRedraw();
    }
}


