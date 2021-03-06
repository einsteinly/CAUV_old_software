/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CANNY_NODE_H__
#define __CANNY_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class CannyNode: public Node{
    public:
        CannyNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // slow node:
            m_speed = slow;
            
            // one input:
            registerInputID(Image_In_Name, Const);
            
            // one output
            registerOutputID(Image_Out_Copied_Name, image_ptr_t());
            
            // parameters:
            registerParamID<float>("threshold 1", 50);
            registerParamID<float>("threshold 2", 80);
            registerParamID<int>("aperture size", 3);
            registerParamID<int>("L2 gradient", 0);
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs[Image_In_Name];
            
            float t1 = param<float>("threshold 1");
            float t2 = param<float>("threshold 2");
            float ap = param<int>("aperture size");
            float g = param<int>("L2 gradient");

            cv::Mat dst;
            try{
                img->apply(boost::bind(cv::Canny, _1, boost::ref(dst), t1, t2, ap, g));
                r[Image_Out_Copied_Name] = boost::make_shared<Image>(dst);
            }catch(cv::Exception& e){
                error() << "CannyNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line << "\n\t"
                        << "The parameters to this node are:\n\t"
                        << "threshold 1 = " << param<float>("threshold 1") << "\n\t"
                        << "threshold 2 = " << param<float>("threshold 2") << "\n\t"
                        << "aperture size = " << param<int>("aperture size") << "\n\t"
                        << "L2 gradient = " << param<int>("L2 gradient") << "\n\t"
                        << "The inputs to this node are:\n\t"
                        << "*image_in = " << *img;
            }
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __CANNY_NODE_H__


