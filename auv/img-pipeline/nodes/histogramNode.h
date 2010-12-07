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


class HistogramNode: public Node{
    public:
        HistogramNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
            // fast node:
            m_speed = fast;
            
            // one input:
            registerInputID("image_in");
            
            // no output
            
            // parameter: 
            registerParamID<int>("number of bins", 30);
            
        }
    
        virtual ~HistogramNode(){
            stop();
        }
        
        // this node should be run even if nothing is connected to its output
        virtual bool isOutputNode() { return true; } 

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;
            
            int bins = param<int>("number of bins");
            
            image_ptr_t img = inputs["image_in"];
            
            if(!img->cvMat().isContinuous())
                throw(parameter_error("image must be continuous"));
            if((img->cvMat().type() & CV_MAT_DEPTH_MASK) != CV_8U)
                throw(parameter_error("image must be unsigned bytes"));
            if(img->cvMat().channels() > 1)
                throw(parameter_error("image must have only single channel"));
                // TODO: support vector parameters
                
            // let's quantize the hue to 30 levels
            // and the saturation to 32 levels
            int histSize[] = {bins};
            // hue varies from 0 to 179, see cvtColor
            float hranges[] = { 0, 180 };

            const float* ranges[] = { hranges};
            cv::MatND hist;
            // we compute the histogram from the 0-th and 1-st channels
            int channels[] = {0};

            cv::calcHist( &img->cvMat(), 1, channels , cv::Mat(), // do not use mask
                hist, 1, histSize, ranges,
                true, // the histogram is uniform
                false );
            double maxVal=0;
            minMaxLoc(hist, 0, &maxVal, 0, 0);
            int imgsize = img->cvMat().rows*img->cvMat().cols;

            int scale = 10;
            cv::Mat histImg = cv::Mat::zeros(bins*scale, bins*scale, CV_8UC3);
            std::vector<float> binVal;
            for( int h = 0; h < bins; h++ ){
                    binVal.push_back(hist.at<float>(h)/imgsize);
                }
         
            //This is the messaging bit
            sendMessage(boost::make_shared<HistogramMessage>(binVal));     
            return r;
        }



    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __HISTOGRAM_H__
