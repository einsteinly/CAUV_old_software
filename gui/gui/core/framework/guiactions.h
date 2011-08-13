#ifndef GUIACTIONS_H
#define GUIACTIONS_H

#include <boost/shared_ptr.hpp>

class QMainWindow;
class QGraphicsScene;
class QGraphicsView;

namespace cauv {

    class CauvNode;

    namespace gui {

        class AUV;
        class NodePicker;

        struct GuiActions {
            boost::shared_ptr<QMainWindow> window;
            boost::shared_ptr<CauvNode> node;
            boost::shared_ptr<AUV> auv;

            boost::shared_ptr<QGraphicsScene> scene;
            boost::shared_ptr<QGraphicsView> view;

            boost::shared_ptr<NodePicker> nodes;
        };
    } //namespace gui
} // namespace cauv


#endif // GUIACTIONS_H
