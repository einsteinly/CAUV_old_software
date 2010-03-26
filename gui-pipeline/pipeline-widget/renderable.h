#ifndef __RENDERABLE_H__
#define __RENDERABLE_H__

#include <QMouseEvent>

#include <boost/shared_ptr.hpp>

class Renderable;
class PipelineWidget;

/* MouseEvents are in projected coordinates: the constructors are used to
 * referr the event to another context
 */
struct MouseEvent{
    /* referr from qt coordinates to top-level renderables: */
    MouseEvent(QMouseEvent* qm,
               boost::shared_ptr<Renderable> r,
               PipelineWidget const& p);
    /* referr from a renderable to children (ie, just offset position) */
    MouseEvent(MouseEvent const& m,
               boost::shared_ptr<Renderable> r);
        
    double x, y;
    Qt::MouseButtons buttons;
};


struct BBox{
    bool contains(double x, double y){
        return (x >= xmin) && (x <= xmax) && (y >= ymin) && (y <= ymax);
    }
    double xmin, ymin, xmax, ymax;
};


class Renderable{
    public:
        Renderable(PipelineWidget& p, double x = 0, double y = 0);
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
        virtual BBox bbox(){ BBox r = {0, 0, 0, 0}; return r; }
        
        // public data:
        double m_pos_x;
        double m_pos_y;

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

