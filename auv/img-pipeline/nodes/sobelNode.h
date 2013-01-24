/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __SOBEL_NODE_H__
#define __SOBEL_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/make_shared.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class SobelNode: public Node{
    public:
        SobelNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image_in", Const);
            
            // one output
            registerOutputID("image_out");
            
            // parameters: scale factor, interpolation mode
            registerParamID<int>("x order", 0, "Order of the x derivative");
            registerParamID<int>("y order", 0, "Order of the y derivative");
            registerParamID<int>("aperture size", 3, "Size of the Sobel kernel (-1 for Scharr)");
        }

    protected:
        static cv::Mat sobel(cv::Mat m, int xorder, int yorder, int size)
        {
            cv::Mat ret;
            cv::Sobel(m, ret, -1, xorder, yorder, size);
            return ret;
        }

        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image_in"];
            
            int xorder = param<int>("x order");
            int yorder = param<int>("y order");
            int size = param<int>("aperture size");
            
            augmented_mat_t in = img->augmentedMat();

            try{
                r["image_out"] = boost::make_shared<Image>(img->apply(boost::bind(sobel, _1, xorder, yorder, size)));
            }catch(cv::Exception& e){
                error() << "SobelNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __SOBEL_NODE_H__
