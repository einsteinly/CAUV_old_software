#ifndef __MOUSE_EVENT_H__
#define __MOUSE_EVENT_H__

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
    /* referr from saved last mouse position to top-level renderables */
    MouseEvent(boost::shared_ptr<Renderable> r,
               PipelineWidget const& p);
    /* referr from a renderable to children (ie, just offset position) */
    MouseEvent(MouseEvent const& m,
               boost::shared_ptr<Renderable> r);
    
    /* coordinates in current projection */
    double x, y;

    Qt::MouseButtons buttons;
};

#endif // ndef __MOUSE_EVENT_H__

