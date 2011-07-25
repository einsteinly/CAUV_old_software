#ifndef DATASTREAMDRAGGING_H
#define DATASTREAMDRAGGING_H

#include <vector>
#include <boost/shared_ptr.hpp>

class QDropEvent;
class QDragEnterEvent;

namespace cauv {
    namespace gui {

        class NodeBase;
        class NumericNode;
        class ImageNode;
        class FloatYPRNode;
        class FloatXYZNode;
        class GroupingNode;

        class NodeDragSource{
        public:
            virtual std::vector<boost::shared_ptr<NodeBase> > getDroppedNodes() = 0;
        };


        class NodeDropListener {
        public:
            virtual void dragEnterEvent(QDragEnterEvent *event);
            virtual void dropEvent(QDropEvent *event);

        protected:

            bool routeStream(boost::shared_ptr<NodeBase> s);

            virtual void onNodeDropped(boost::shared_ptr<NumericNode> ) {}
            virtual void onNodeDropped(boost::shared_ptr<ImageNode> ) {}
            virtual void onNodeDropped(boost::shared_ptr<FloatYPRNode> ) {}
            virtual void onNodeDropped(boost::shared_ptr<FloatXYZNode> ) {}
            virtual void onNodeDropped(boost::shared_ptr<GroupingNode> ) {}
        };

    } // namespace gui
} // namespace cauv

#endif // DATASTREAMDRAGGING_H
