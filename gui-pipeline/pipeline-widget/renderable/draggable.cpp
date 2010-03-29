#include "draggable.h"
#include "../pipelineWidget.h"

Draggable::Draggable(PipelineWidget& p)
    : Renderable(p), m_click_pos(),
      m_mouseover(false), m_pressed(false){
}

void Draggable::mouseMoveEvent(MouseEvent const& event){
    if(m_pressed && (event.buttons & Qt::LeftButton)){
        m_pos += event.pos - m_click_pos;
        // need a re-draw
        m_parent.updateGL();
    }else if(!m_mouseover){
        m_mouseover = true;
        m_parent.updateGL();
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
        m_parent.updateGL();
    }
}


