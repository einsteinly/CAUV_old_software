#ifndef __DRAGGABLE_RENDERABLE_H__
#define __DRAGGABLE_RENDERABLE_H__

#include "../renderable.h"

class Draggable: public Renderable{
    public:
        Draggable(container_ptr_t c);
        virtual void mouseMoveEvent(MouseEvent const& event);
        virtual void mousePressEvent(MouseEvent const& event);
        virtual void mouseGoneEvent();

    protected:
        Point m_click_pos;

        bool m_mouseover;
        bool m_pressed;
};

#endif // ndef __DRAGGABLE_RENDERABLE_H__

