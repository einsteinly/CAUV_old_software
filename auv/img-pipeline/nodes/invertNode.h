/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __INVERT_NODE_H__
#define __INVERT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>

#include "../node.h"
#include "../nodeFactory.h"


namespace cauv{
namespace imgproc{

class InvertNode: public Node{
    public:
        InvertNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("image", Const);

            // one output
            registerOutputID("image (not copied)");
        }

    protected:

        static void invert(cv::Mat& m) {
            if (m.depth() == CV_32F || m.depth() == CV_64F)
                m = 1 - m;
            else
                m = 255 - m;
        }

        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image"];
            img->apply(boost::bind(invert, _1));
            r["image (not copied)"] = img;
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __INVERT_NODE_H__
