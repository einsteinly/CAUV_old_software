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
        CornerHarrisNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // slow node:
            m_speed = slow;
            
            // one input:
            registerInputID("image_in");
            
            // one output
            registerOutputID("image_out", image_ptr_t());
            
            // parameters:
            registerParamID<int>("block size", 2);
            registerParamID<int>("aperture size", 3);
            registerParamID<float>("free parameter", 0.04);
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            cv::Mat img = inputs["image_in"]->mat();
            
            if (img.channels() !=  1){
                error() << "CornerHarrisNode:\n\t"
                        << "Input image must be single channeled";
            }

            if (img.elemSize() != 1){
                error() << "ThresholdMaskNode:\n\t"
                        << "Invalid image input - must be 8-bit";
                return r;
            }

            int bs = param<int>("block size");
            float ap = param<int>("aperture size");
            float k = param<float>("free parameter");

            cv::Mat dst(img.cols, img.rows, CV_32FC1);
            try{
                cv::cornerHarris(img, dst, bs, ap, (double)k);
                //boost::shared_ptr<Image> newi = boost::make_shared<Image>();
                //dst->cvMat().convertTo(newi->cvMat(), CV_8UC1);
                r["image_out"] = boost::make_shared<Image>(dst);

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


