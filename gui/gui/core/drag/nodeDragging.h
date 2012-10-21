/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __CAUV_NODEDRAGGING_H__
#define __CAUV_NODEDRAGGING_H__

#include <vector>
#include <boost/shared_ptr.hpp>

#include <QtGui>

namespace cauv {
    namespace gui {

        class Node;

        struct drop_not_handled : public std::exception {};


        template <class T> class DropHandlerInterface {
        public:
            // return true if the object will handle the node, but
            // don't actually handle it yet
            virtual bool accepts(boost::shared_ptr<Node> const& node) = 0;

            // return some object that the caller is interested in
            // if passed an obejct you're not interested in throw a
            // drop_not_handled exception
            virtual T handle(boost::shared_ptr<Node> const& node) = 0;
        };

        
        class NodeDropListener {
        public:
            virtual bool accepts(boost::shared_ptr<Node> const& node) = 0;
            bool routeNode(boost::shared_ptr<Node> const& s, QPointF pos);

        protected:
            virtual void onNodeDroppedAt(boost::shared_ptr<Node> const&, QPointF ) {}
            virtual void onNodeDropped(boost::shared_ptr<Node> const&) {}
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

#endif // __CAUV_NODEDRAGGING_H__
