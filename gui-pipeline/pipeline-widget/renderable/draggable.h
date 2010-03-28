#ifndef __DRAGGABLE_RENDERABLE_H__
#define __DRAGGABLE_RENDERABLE_H__

#include "../renderable.h"

class Draggable: public Renderable{
    public:
        Draggable(PipelineWidget& p);
        virtual void mouseMoveEvent(MouseEvent const& event);
        virtual void mousePressEvent(MouseEvent const& event);
        virtual void mouseGoneEvent();

    protected:
        double m_click_pos_x;
        double m_click_pos_y;

        bool m_mouseover;
        bool m_pressed;
};

#endif // ndef __DRAGGABLE_RENDERABLE_H__

