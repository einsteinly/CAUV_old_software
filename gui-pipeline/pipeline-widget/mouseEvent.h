#ifndef __MOUSE_EVENT_H__
#define __MOUSE_EVENT_H__

#include <QMouseEvent>

#include <boost/shared_ptr.hpp>

#include "util.h"

class Renderable;
class PipelineWidget;

// FIXME: all the coordinate referring stuff should probably be in Container,
// rather than here!

/* MouseEvents are in projected coordinates: the constructors are used to
 * refer the event to another context
 */
struct MouseEvent{
    /* refer from qt coordinates to top-level */
    MouseEvent(QMouseEvent* qm,
               PipelineWidget const& p);

    /* refer from qt coordinates to top-level renderables: */
    MouseEvent(QMouseEvent* qm,
               boost::shared_ptr<Renderable> r,
               PipelineWidget const& p);

    /* refer from saved last mouse position to top-level */
    MouseEvent(PipelineWidget const& p);

    /* refer from saved last mouse position to top-level renderables */
    MouseEvent(boost::shared_ptr<Renderable> r,
               PipelineWidget const& p);

    /* refer from a renderable to children (ie, just offset position) */
    MouseEvent(MouseEvent const& m,
               boost::shared_ptr<Renderable> r);
    
    /* coordinates in current projection */
    Point pos;

    Qt::MouseButtons buttons;
};

#endif // ndef __MOUSE_EVENT_H__

