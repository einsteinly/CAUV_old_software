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

#ifndef __DRAGGABLE_RENDERABLE_H__
#define __DRAGGABLE_RENDERABLE_H__

#include "../renderable.h"

namespace cauv{
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
} // namespace cauv

#endif // ndef __DRAGGABLE_RENDERABLE_H__

