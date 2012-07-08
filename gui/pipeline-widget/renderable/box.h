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

#ifndef __BOX_RENDERABLE_H__
#define __BOX_RENDERABLE_H__

#include "draggable.h"

namespace cauv{
namespace gui{
namespace pw{

class Box: public Draggable{
    public:
        Box(container_ptr_t c, double const& w, double const& h)
            : Draggable(c), m_box(0, -h, w, 0){
        }
        virtual ~Box(){ }

        virtual void draw(drawtype_e::e){
            if(m_mouseover)
                glColor4f(1.0, 0.0, 0.0, 0.5);
            else
                glColor4f(1.0, 1.0, 1.0, 0.5);
            glBox(m_box);
        }
        
        virtual bool tracksMouse(){
            return true;
        }

        virtual BBox bbox(){
            return m_box;
        }

    private:
        BBox m_box;
};

} // namespace pw
} // namespace gui
} // namespace cauv

#endif // ndef __BOX_RENDERABLE_H__

