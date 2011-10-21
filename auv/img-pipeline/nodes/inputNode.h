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

        virtual void onImageMessage(boost::shared_ptr<const ImageMessage>){}
        
        virtual void onSonarDataMessage(boost::shared_ptr<const SonarDataMessage>){}

        virtual void onSonarImageMessage(boost::shared_ptr<const SonarImageMessage>){}

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

