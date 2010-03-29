#ifndef __RENDERABLE_H__
#define __RENDERABLE_H__

#include <QMouseEvent>

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "util.h"
#include "mouseEvent.h"

class Renderable;
class PipelineWidget;

class Renderable: public boost::enable_shared_from_this<Renderable>{
    public:
        Renderable(PipelineWidget& p, Point const& at = Point());
        virtual void draw(bool picking) = 0;

        /* overload to receive mouse events
         */
        virtual void mouseMoveEvent(MouseEvent const&){ }
        virtual void mousePressEvent(MouseEvent const&){ }
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
        
        // public data:
        Point m_pos;

    protected:
        PipelineWidget& m_parent;
};

class NullRenderable: public Renderable{
    public:
        NullRenderable() // hackery, please ignore
            : Renderable(reinterpret_cast<PipelineWidget&>(*this)) { }
        virtual void draw(bool) { }
};

#endif // ndef __RENDERABLE_H__

