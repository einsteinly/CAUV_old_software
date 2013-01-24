/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __RENDERABLE_H__
#define __RENDERABLE_H__

#include <boost/shared_ptr.hpp>

#include "util.h"
#include "keyEvent.h"
#include "mouseEvent.h"
#include "pwTypes.h"

namespace cauv{
namespace gui{
namespace pw{

class Renderable{
    public:

        Renderable(container_ptr_t c, Point const& at = Point());
        virtual ~Renderable(){ }

        friend bool operator<(Renderable const& l, Renderable const& r){
            return l.m_sort_key < r.m_sort_key;
        }

        virtual void draw(drawtype_e::e flags) = 0;

        /* overload to receive mouse events
         */
        virtual void mouseMoveEvent(MouseEvent const&){ }
        virtual bool mousePressEvent(MouseEvent const&){ return false; }
        virtual void mouseReleaseEvent(MouseEvent const&){ }
        virtual void mouseGoneEvent(){ }

        /* overload to receive keyboard events
         */
        virtual bool keyPressEvent(KeyEvent const&){ return false; }
        virtual bool keyReleaseEvent(KeyEvent const&){ return false; }

        /* Should mouse events be passed even if not button is pressed?
         */
        virtual bool tracksMouse(){ return false; }
        /* Should mouse events be passed at all?
         */
        virtual bool acceptsMouseEvents(){ return true; }

        /* used to decide when to pass mouse events when button not pressed
         */
        virtual BBox bbox(){ return BBox(); }
        
        /* return m_pos refered to top level coordinates
         */
        virtual Point topLevelPos() const;

        // public data:
        Point m_pos;

    protected:
        std::string m_sort_key;
        container_ptr_t m_context;
};

} // namespace pw
} // namespace gui
} // namespace cauv

#endif // ndef __RENDERABLE_H__

