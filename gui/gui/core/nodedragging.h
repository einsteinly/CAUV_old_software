#ifndef NODEDRAGGING_H
#define NODEDRAGGING_H

#include <vector>
#include <boost/shared_ptr.hpp>

#include <debug/cauv_debug.h>

#include <QEvent>
#include <QGraphicsRectItem>

#include "model/nodes_fwd.h"

namespace cauv {
    namespace gui {

        class NodeDragSource{
        public:
            virtual std::vector<boost::shared_ptr<NodeBase> > getDroppedNodes() = 0;
        };


        struct drop_not_handled : public std::exception {};


        template <class T> class DropHandlerInterface {
        public:
            // return true if the object will handle the node, but
            // don't actually handle it yet
            virtual bool accepts(boost::shared_ptr<NodeBase> const& node) = 0;

            // return some object that the caller is interested in
            // if passed an obejct you're not interested in throw a
            // drop_not_handled exception
            virtual T handle(boost::shared_ptr<NodeBase> const& node) = 0;

            virtual ~DropHandlerInterface(){
                debug(2) << "~DropHandlerInterface()";
            }
        };

        
        class NodeDropListener {
        public:
            virtual bool accepts(boost::shared_ptr<NodeBase> const& node) = 0;
            bool routeNode(boost::shared_ptr<NodeBase> const& s, QPointF pos);

        protected:

            // all nodes get passed into this
            virtual void onNodeDroppedAt(boost::shared_ptr<NodeBase> const&, QPointF ) {}
            virtual void onNodeDropped(boost::shared_ptr<NodeBase> const&) {}

            // then they also get passed into one of these
            virtual void onNodeDropped(boost::shared_ptr<NumericNode> const& ) {}
            virtual void onNodeDropped(boost::shared_ptr<ImageNode> const& ) {}
            virtual void onNodeDropped(boost::shared_ptr<FloatYPRNode> const& ) {}
            virtual void onNodeDropped(boost::shared_ptr<FloatXYZNode> const& ) {}
            virtual void onNodeDropped(boost::shared_ptr<GroupingNode> const& ) {}
        };



        // this class is used with QObject subclasses to allow them to receive drops
        class NodeDropFilter : public QObject {
        public:
            NodeDropFilter(NodeDropListener * listener);
            bool eventFilter(QObject * object, QEvent *event);

        protected:
            NodeDropListener *m_listener;
        };


        // this class provides an area that nodes can be dropped onto in a QGraphicsScene
        class NodeSceneDropArea: public NodeDropFilter, public QGraphicsRectItem {
            Q_OBJECT
        public:
            NodeSceneDropArea(NodeDropListener * listener);
            bool sceneEvent(QEvent *event);

        public Q_SLOTS:
            void updateGeometry(QRectF const& rect);

        protected:
            NodeDropListener *m_listener;
        };


    } // namespace gui
} // namespace cauv

#endif // NODEDRAGGING_H
