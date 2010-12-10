#ifndef __HISTOGRAM_H__
#define __HISTOGRAM_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <generated/messages.h>

#include "../node.h"
#include "outputNode.h"

class HistogramNode: public OutputNode{
    public:
        HistogramNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : OutputNode(sched, pl, t){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            //One input
            registerInputID("image_in");
            
            //No output
            
            //Parameters
            registerParamID<int>("Number of bins", 42);
            registerParamID<std::string>("Channel type", "Unnamed");
            
        }
    
        virtual ~HistogramNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            int bins = param<int>("Number of bins");
            std::string type = param<std::string>("Channel type");

            image_ptr_t img = inputs["image_in"];

            if(!img->cvMat().isContinuous())
                throw(parameter_error("image must be continuous"));
            if((img->cvMat().type() & CV_MAT_DEPTH_MASK) != CV_8U)
                throw(parameter_error("image must have unsigned bytes"));
            if(img->cvMat().channels() > 1)
                throw(parameter_error("image must have only one channel"));
                //TODO: support vector parameters

            //Let's quantize the hue to 30 levels
            //And the saturation to 32 levels
            int histSize[] = {bins};
            //Hue varies from 0 to 179, see cvtColor
            float hranges[] = {0, 256};

            const float* ranges[] = {hranges};
            cv::MatND hist;
            //We compute the histogram from the 0-th and 1-st channels
            int channels[] = {0};

            cv::calcHist( &img->cvMat(), 1, channels , cv::Mat(), //Do not use mask
                hist, 1, histSize, ranges,
                true, //The histogram is uniform
                false );
            double maxVal = 0;
            minMaxLoc(hist, 0, &maxVal, 0, 0);
            int imgsize = img->cvMat().rows * img->cvMat().cols;

            int scale = 10;
            cv::Mat histImg = cv::Mat::zeros(bins * scale, bins * scale, CV_8UC3);
            std::vector<float> binVal;
            for(int h = 0; h < bins; h++){
                binVal.push_back(hist.at<float>(h) / imgsize);
            }

            //This is the messaging bit
            sendMessage(boost::make_shared<HistogramMessage>(binVal, type));
            return r;
        }

    //Register this node type
    DECLARE_NFR;
};

#endif //ndef __HISTOGRAM_H__
