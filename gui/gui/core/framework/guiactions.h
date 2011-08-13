#ifndef GUIACTIONS_H
#define GUIACTIONS_H

#include <boost/shared_ptr.hpp>

class QGraphicsScene;

namespace cauv {

    class CauvNode;

    namespace gui {

        class AUV;
        class NodePicker;
        class NodeVisualiser;
        class CauvMainWindow;

        struct GuiActions {
            boost::shared_ptr<CauvMainWindow> window;
            boost::shared_ptr<CauvNode> node;
            boost::shared_ptr<AUV> auv;

            boost::shared_ptr<QGraphicsScene> scene;
            boost::shared_ptr<NodeVisualiser> view;

            boost::shared_ptr<NodePicker> nodes;
        };
    } //namespace gui
} // namespace cauv


#endif // GUIACTIONS_H
