#ifndef __RESIZE_NODE_H__
#define __RESIZE_NODE_H__

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

class ResizeNode: public Node{
    public:
        ResizeNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : Node(sched, pl, n, t){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image_in");
            
            // one output
            registerOutputID<image_ptr_t>("image_out");
            
            // parameters: scale factor, interpolation mode
            registerParamID<float>("scale factor", 1.0f);
            registerParamID<int>("interpolation mode", cv::INTER_LINEAR);
        }
    
        virtual ~ResizeNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image_in"];
            
            float scale_fac = param<float>("scale factor");
            int interp = param<int>("interpolation mode");
            
            cv::Mat new_mat;

            try{
                cv::resize(img->cvMat(), new_mat, cv::Size(), scale_fac, scale_fac, interp);
                r["image_out"] = boost::make_shared<Image>(new_mat);
            }catch(cv::Exception& e){
                error() << "ResizeNode:\n\t"
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

#endif // ndef __RESIZE_NODE_H__
