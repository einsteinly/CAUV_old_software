#ifndef __HISTOGRAMSEGMENT_H__
#define __HISTOGRAMSEGMENT_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class HistogramSegmentationNode: public OutputNode{
    public:
        HistogramSegmentationNode(ConstructArgs const& args)
            : OutputNode(args){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            //One input
            registerInputID("image_in", Const);
            
            //One output
            registerOutputID("Pixels");
            
            //Parameters
            registerParamID<int>("Number of bins", 42);
            registerParamID<int>("Bin", 0);
            
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){

            int bins = param<int>("Number of bins");
            int bin = param<int>("Bin");

            cv::Mat img = inputs["image_in"]->mat();

            if(!img.isContinuous())
                throw(parameter_error("Image must be continuous."));
            if(img.depth() != CV_8U)
                throw(parameter_error("Image must have unsigned bytes."));
            if(img.channels() > 1)
                throw(parameter_error("Image must have only one channel."));
                //TODO: support vector parameters

            float binWidth = 256 / bins;
            float binMin = bin * binWidth;
            float binMax = (bin + 1) * binWidth;
            cv::Mat out = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);

            for(int i = 0; i < img.rows; i++) {
                for(int j = 0; j < img.cols; j++) {
                   if(img.at<uint8_t>(i, j) >= binMin && img.at<uint8_t>(i, j) < binMax) {
                       out.at<uint8_t>(i, j) = 255;
                   }
                }
            }

            r["Pixels"] = boost::make_shared<Image>(out);
        }

    //Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif //ndef __HISTOGRAMSEGMENT_H__
