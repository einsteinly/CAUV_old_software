#ifndef __RENDERABLE_H__
#define __RENDERABLE_H__

#include <QMouseEvent>

#include <boost/shared_ptr.hpp>

class Renderable;
class PipelineWidget;

/* MouseEvents are in projected coordinates */
struct MouseEvent{
    MouseEvent(QMouseEvent* qm,
               boost::shared_ptr<Renderable> r,
               PipelineWidget const& p);

    double x, y;
    Qt::MouseButtons buttons;
};


class Renderable{
    public:
        Renderable(PipelineWidget& p, double x = 0, double y = 0);
        virtual void draw() = 0;

        /* overload to receive mouse events
         */
        virtual void mouseMoveEvent(MouseEvent const&){ }
        virtual void mousePressEvent(MouseEvent const&){ }
        virtual void mouseReleaseEvent(MouseEvent const&){ }

        /* Should mouse events be passed even if not button is pressed?
         */
        virtual bool tracksMouse(){ return false; }
        /* Should mouse events be passed at all?
         */
        virtual bool acceptsMouseEvents(){ return true; }
        
        // public data:
        double m_pos_x;
        double m_pos_y;

    protected:
        PipelineWidget& m_parent;
};

#endif // ndef __RENDERABLE_H__

