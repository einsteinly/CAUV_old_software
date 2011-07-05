#ifndef INPUT_NODE_H
#define INPUT_NODE_H

#include "../node.h"

namespace cauv{
namespace imgproc{

class InputNode: public Node{ 

    public:
        enum Subscription {ImageData, SonarData};
        InputNode(ConstructArgs const& args)
            : Node(args)
        {
            // don't allow the node to be executed until it has input available
            clearAllowQueue();
        }

        virtual void onImageMessage(boost::shared_ptr<const ImageMessage>) throw() {}
        
        virtual void onSonarDataMessage(boost::shared_ptr<const SonarDataMessage>){}

        /* input nodes need to be identified so that onImageMessage() can be
         * efficiently called on only input nodes
         */
        virtual bool isInputNode() const { return true; }
    
    protected:
        typedef boost::lock_guard<boost::recursive_mutex> lock_t;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef INPUT_NODE_H

