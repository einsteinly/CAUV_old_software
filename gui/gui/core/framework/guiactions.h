#ifndef GUIACTIONS_H
#define GUIACTIONS_H

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <debug/cauv_debug.h>

class QGraphicsScene;

namespace liquid {
    class LiquidView;
}

namespace cauv {

    class CauvNode;

    namespace gui {

        class AUV;
        class NodePicker;
        class CauvMainWindow;
        class NodeScene;

        struct GuiActions {
            // weak ptrs to window (which is actually the same as node)
            // breaks the cycle of window -> actions -> window
            boost::weak_ptr<CauvMainWindow> window;
            boost::weak_ptr<CauvNode> node;

            // models
            boost::shared_ptr<AUV> auv;
            boost::shared_ptr<NodeScene> scene;

            // gui elements
            liquid::LiquidView * view;
            NodePicker * nodes;

            ~GuiActions() {
                debug(2) << "~GuiActions()";
            }
        };
    } //namespace gui
} // namespace cauv


#endif // GUIACTIONS_H
