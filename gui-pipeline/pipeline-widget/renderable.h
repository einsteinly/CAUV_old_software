#ifndef __RENDERABLE_H__
#define __RENDERABLE_H__

#include <QMouseEvent>

#include <boost/shared_ptr.hpp>

#include "util.h"
#include "mouseEvent.h"

class Container;

class Renderable{
    public:
        // public typedefs
        typedef Container* container_ptr_t;
    public:
        Renderable(container_ptr_t c, Point const& at = Point());
        virtual void draw(bool picking) = 0;

        /* overload to receive mouse events
         */
        virtual void mouseMoveEvent(MouseEvent const&){ }
        virtual bool mousePressEvent(MouseEvent const&){ return false; }
        virtual void mouseReleaseEvent(MouseEvent const&){ }
        virtual void mouseGoneEvent(){ }

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
        Point topLevelPos() const;

        // public data:
        Point m_pos;

    protected:
        container_ptr_t m_context;
};

#endif // ndef __RENDERABLE_H__

