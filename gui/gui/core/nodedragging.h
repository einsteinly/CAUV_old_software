#ifndef DATASTREAMDRAGGING_H
#define DATASTREAMDRAGGING_H

#include <vector>
#include <boost/shared_ptr.hpp>

#include "model/nodes_fwd.h"

namespace cauv {
    namespace gui {

        class NodeDragSource{
        public:
            virtual std::vector<boost::shared_ptr<NodeBase> > getDroppedNodes() = 0;
        };


        class NodeDropListener {
        public:
            void onDrop(NodeDragSource * source);

        protected:

            bool routeNode(boost::shared_ptr<NodeBase> s);

            // all nodes get passed into this
            virtual void onNodeDropped(boost::shared_ptr<NodeBase> ) {}

            // then they also get passed into one of these
            virtual void onNodeDropped(boost::shared_ptr<NumericNode> ) {}
            virtual void onNodeDropped(boost::shared_ptr<ImageNode> ) {}
            virtual void onNodeDropped(boost::shared_ptr<FloatYPRNode> ) {}
            virtual void onNodeDropped(boost::shared_ptr<FloatXYZNode> ) {}
            virtual void onNodeDropped(boost::shared_ptr<GroupingNode> ) {}
        };

    } // namespace gui
} // namespace cauv

#endif // DATASTREAMDRAGGING_H
