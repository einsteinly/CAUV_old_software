/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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

#ifndef __GUI_OUTPUT_NODE_H__
#define __GUI_OUTPUT_NODE_H__

#include "../node.h"
#include "outputNode.h"

namespace cauv{
namespace imgproc{

class GuiOutputNode: public OutputNode{
    public:
        GuiOutputNode(ConstructArgs const& args)
            : OutputNode(args){
        }

        void init(){
            // one input:
            registerInputID("image_in", Const);

            // no outputs
            
            // one parameter: image compression quality for network
            // transmission
            registerParamID<BoundedFloat>(
                "jpeg quality", BoundedFloat(85, 0, 100, BoundedFloatType::Clamps)
            ); // 0-100
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t&){
            using boost::algorithm::replace_all_copy;

            image_ptr_t img = inputs["image_in"];
            float qual = param<BoundedFloat>("jpeg quality");
            
            debug(4) << "GuiOutputNode::doWork()" << *this;

            img->serializeQuality(int(qual));
            sendMessage(boost::make_shared<GuiImageMessage>(plName(), id(), img), UNRELIABLE_MSG);
        }

        int m_counter;
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __GUI_OUTPUT_NODE_H__
