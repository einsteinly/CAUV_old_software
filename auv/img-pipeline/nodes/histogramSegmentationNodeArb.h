#ifndef __HISTOGRAMSEGMENT_H__
#define __HISTOGRAMSEGMENT_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <generated/messages.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class HistogramSegmentationNodeArb: public OutputNode{
    public:
        HistogramSegmentationNodeArb(ConstructArgs const& args)
            : OutputNode(args){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            //One input
            registerInputID("image_in");
            
            //One output
            registerOutputID("Pixels");
            
            //Parameters
            registerParamID<int>("Bin min", 100);
            registerParamID<int>("Bin max", 127);
            
        }
    
        virtual ~HistogramSegmentationNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            //int bins = param<int>("Number of bins");
            //int bin = param<int>("Bin");

            image_ptr_t img = inputs["image_in"];

            if(!img->cvMat().isContinuous())
                throw(parameter_error("Image must be continuous."));
            if((img->cvMat().type() & CV_MAT_DEPTH_MASK) != CV_8U)
                throw(parameter_error("Image must have unsigned bytes."));
            if(img->cvMat().channels() > 1)
                throw(parameter_error("Image must have only one channel."));
                //TODO: support vector parameters

            //float binWidth = 256 / bins;
            float binMin = param<int>("Bin min");//bin * binWidth;
            float binMax = param<int>("Bin max");//(bin + 1) * binWidth;
            cv::Mat out = cv::Mat::zeros(img->cvMat().rows, img->cvMat().cols, CV_8UC1);

            for(int i = 0; i < img->cvMat().rows; i++) {
                for(int j = 0; j < img->cvMat().cols; j++) {
                   if(img->cvMat().at<uint8_t>(i, j) >= binMin && img->cvMat().at<uint8_t>(i, j) < binMax) {
                       out.at<uint8_t>(i, j) = 255;
                   }
                }
            }

            r["Pixels"] = boost::make_shared<Image>(out);
            return r;
        }

    //Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif //ndef __HISTOGRAMSEGMENT_H__
