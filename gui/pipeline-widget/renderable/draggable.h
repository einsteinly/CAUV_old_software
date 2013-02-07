/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __DRAGGABLE_RENDERABLE_H__
#define __DRAGGABLE_RENDERABLE_H__

#include "../renderable.h"

namespace cauv{
namespace gui{
namespace pw{

class Draggable: public Renderable{
    public:
        Draggable(container_ptr_t c);
        virtual ~Draggable(){ }

        virtual void mouseMoveEvent(MouseEvent const& event);
        virtual bool mousePressEvent(MouseEvent const& event);
        virtual void mouseGoneEvent();

    protected:
        Point m_click_pos;

        bool m_mouseover;
        bool m_pressed;
};

} // namespace pw
} // namespace gui
} // namespace cauv

#endif // ndef __DRAGGABLE_RENDERABLE_H__

