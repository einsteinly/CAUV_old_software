#ifndef __HISTOGRAMSEGMENT_H__
#define __HISTOGRAMSEGMENT_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <generated/messages.h>

#include "../node.h"
#include "outputNode.h"


class HistogramSegmentationNode: public OutputNode{
    public:
        HistogramSegmentationNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : OutputNode(sched, pl, t){
            // fast node:
            m_speed = fast;
            
            // one input:
            registerInputID("image_in");
            
            // no output
            registerOutputID<image_ptr_t>("Segments");
            
            // parameter: 
            registerParamID<int>("Number of bins", 42);
            registerParamID<int>("Bin", 0);
            
        }
    
        virtual ~HistogramSegmentationNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;
            
            int bins = param<int>("Number of bins");
            int bin = param<int>("Bin");
            
            image_ptr_t img = inputs["image_in"];
            
            if(!img->cvMat().isContinuous())
                throw(parameter_error("Image must be continuous."));
            if((img->cvMat().type() & CV_MAT_DEPTH_MASK) != CV_8U)
                throw(parameter_error("Image must be unsigned bytes."));
            if(img->cvMat().channels() > 1)
                throw(parameter_error("Image must have only single channel."));
                // TODO: support vector parameters

            float binWidth = 256 / bins;
            float binMin = bin * binWidth;
            float binMax = (bin + 1) * binWidth;
            cv::Mat out = cv::Mat::zeros(img->cvMat().rows, img->cvMat().cols, CV_8UC3);

            for(int i = 0; i < img->cvMat().cols; i++) {
                for(int j = 0; j < img->cvMat().rows; j++) {
                   if(img->cvMat().at<uint8_t>(i, j) > binMin && img->cvMat().at<uint8_t>(i, j) < binMax) {
                       out.at<uint8_t>(i, j) = 255;
                   }
                }
            }

            r["Segments"] = boost::make_shared<Image>(out);
            return r;
        }

    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __HISTOGRAMSEGMENT_H__
