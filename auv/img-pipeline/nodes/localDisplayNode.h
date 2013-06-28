/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __LOCAL_DISPLAY_NODE_H__
#define __LOCAL_DISPLAY_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class LocalDisplayNode: public OutputNode{
    public:
        LocalDisplayNode(ConstructArgs const& args)
            : OutputNode(args){
        }

        void init(){
            // one input:
            registerInputID("image_in", Const);
            
            cv::namedWindow("LocalDisplayNode", cv::WINDOW_AUTOSIZE);
            // no outputs
            // no parameters
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t&){

            image_ptr_t img = inputs["image_in"];
            
            debug(4) << "LocalDisplayNode::doWork";
           
            cv::imshow("LocalDisplayNode", img->mat());
            cv::waitKey(10);

        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __LOCAL_DISPLAY_NODE_H__
