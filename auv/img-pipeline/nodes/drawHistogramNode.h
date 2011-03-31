#ifndef __DRAW_HISTOGRAM_NODE_H__
#define __DRAW_HISTOGRAM_NODE_H__

#include "../node.h"

#include <algorithm>

namespace cauv{
namespace imgproc{

class DrawHistogramNode: public Node{
    public:
        DrawHistogramNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t), m_counter(0){
        }

        void init(){
            registerParamID< std::vector<float> >("histogram",
            std::vector<float>(), "values to plot");
            registerOutputID<image_ptr_t>(Image_Out_Copied_Name);
        }
    
        virtual ~DrawHistogramNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            const int Rows = 100;
            out_map_t r;
            
            std::vector<float> histogram = param< std::vector<float> >("histogram");

            boost::shared_ptr<Image> out = boost::make_shared<Image>(
                cv::Mat::zeros(Rows, histogram.size(), CV_8UC1)
            );
            
            if(!histogram.size())
                throw parameter_error("no histogram to draw");

            float max = *std::max_element( histogram.begin(), histogram.end());
            for(unsigned i = 0; i < histogram.size(); i++){
                for(unsigned j = 0; j < unsigned(0.5+Rows * histogram[i]/max); j++)
                    out->cvMat().at<uint8_t>(Rows-j, i) = 0xff;
            }

            r[Image_Out_Copied_Name] = out;

            return r;
        }

        int m_counter;
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __DRAW_HISTOGRAM_NODE_H__
