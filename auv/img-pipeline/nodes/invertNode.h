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
            registerInputID("image");

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
