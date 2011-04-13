#ifndef __HISTOGRAM_H__
#define __HISTOGRAM_H__

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

class HistogramNode: public Node{
    public:
        HistogramNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            //One input
            registerInputID("image_in");
            
            //Output histogram
            registerOutputID<NodeParamValue>("histogram");
            
            //Parameters
            registerParamID<int>("Number of bins", 42);
            registerParamID<std::string>("name", "unnamed histogram"
                                         "name for output histogram");
            
        }
    
        virtual ~HistogramNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            const int bins = param<int>("Number of bins");
            const std::string name = param<std::string>("name");

            image_ptr_t img = inputs["image_in"];

            if(!img->cvMat().isContinuous())
                throw(parameter_error("image must be continuous"));
            if((img->cvMat().type() & CV_MAT_DEPTH_MASK) != CV_8U)
                throw(parameter_error("image must have unsigned bytes"));
            if(img->cvMat().channels() > 1)
                throw(parameter_error("image must have only one channel"));
                //TODO: support vector parameters

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

            std::vector<float> binVal;
            for(int h = 0; h < bins; h++){
                binVal.push_back(hist.at<float>(h) / imgsize);
            }

            r["histogram"] = NodeParamValue(binVal);

            //This is the messaging bit
            sendMessage(boost::make_shared<HistogramMessage>(name, binVal));
            return r;
        }

    //Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif //ndef __HISTOGRAM_H__
