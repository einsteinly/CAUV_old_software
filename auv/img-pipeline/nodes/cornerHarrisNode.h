#ifndef __CORNER_HARRIS_NODE_H__
#define __CORNER_HARRIS_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"

namespace cauv{
namespace imgproc{

// Detects corners based on the Harris corner detection algorithm
// Note that the image produced is very faint. Threshold or scale!   
class CornerHarrisNode: public Node{
    public:
        CornerHarrisNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : Node(sched, pl, n, t){
        }

        void init(){
            // slow node:
            m_speed = slow;
            
            // one input:
            registerInputID("image_in");
            
            // one output
            registerOutputID<image_ptr_t>("image_out");
            
            // parameters:
            registerParamID<int>("block size", 2);
            registerParamID<int>("aperture size", 3);
            registerParamID<float>("free parameter", 0.04);
        }
    
        virtual ~CornerHarrisNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image_in"];
            
            if (img->cvMat().channels() !=  1){
                error() << "CornerHarrisNode:\n\t"
                        << "Input image must be single channeled";
            }

            if (img->cvMat().elemSize() != 1){
                error() << "ThresholdMaskNode:\n\t"
                        << "Invalid image input - must be 8-bit";
                return r;
            }

            int bs = param<int>("block size");
            float ap = param<int>("aperture size");
            float k = param<float>("free parameter");

            boost::shared_ptr<Image> dst = boost::make_shared<Image>(
                cv::Mat(img->cvMat().cols,
                        img->cvMat().rows,
                        CV_32FC1));
            try{
                cv::cornerHarris(img->cvMat(), dst->cvMat(), bs, ap, (double)k);
                //boost::shared_ptr<Image> newi = boost::make_shared<Image>();
                //dst->cvMat().convertTo(newi->cvMat(), CV_8UC1);
                r["image_out"] = dst;

            }catch(cv::Exception& e){
                error() << "CornerHarrisNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line << "\n\t";
            }
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __CORNER_HARRIS_NODE_H__


