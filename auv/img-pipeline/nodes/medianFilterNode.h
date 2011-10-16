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

#ifndef __MEDIAN_FILTER_NODE_H__
#define __MEDIAN_FILTER_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class MedianFilterNode: public Node{
    public:
        MedianFilterNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image");

            // one output
            registerOutputID<image_ptr_t>("image (not copied)");
            
            // parameters: kernel radius: must be an odd integer
            registerParamID<int>("kernel", 3);
        }
        
        virtual ~MedianFilterNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image"];
            
            int ksize = param<int>("kernel");

            if(!(ksize & 1))
                warning() << "filter kernel size should be odd";

            debug(4) << "MedianFilterNode:" << ksize;
            
            try{
                cv::Mat m = img->mat();
                cv::medianBlur(m, m, ksize);
                img->mat(m);
                r["image (not copied)"] = img;
            }catch(cv::Exception& e){
                error() << "MedianFilterNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif//__MEDIAN_FILTER_NODE_H__
