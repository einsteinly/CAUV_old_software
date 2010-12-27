#ifndef __PERCENTILE_NODE_H__
#define __PERCENTILE_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>

#include "../node.h"


namespace cauv{
namespace imgproc{

class PercentileNode: public Node{
    public:
        PercentileNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image");

            // one output parameter
            registerOutputID<param_value_t>("ch1 value");
            registerOutputID<param_value_t>("ch2 value");
            registerOutputID<param_value_t>("ch3 value");
            
            // parameter: 
            registerParamID<float>("percentile", 50, "0-100 percentile of pixel values");
        }
    
        virtual ~PercentileNode(){
            stop();
        }

    protected:
        static std::string bar(float value, float max, int w){
            std::string r = "";
            for(int i = 0; i < w; i++){
                if(float(i)/w <= value/max)
                    r += "=";
                else
                    r += " ";
            }
            return r;
        }

        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image"];
            
            if(!img->cvMat().isContinuous())
                throw(parameter_error("image must be continuous"));
            if((img->cvMat().type() & CV_MAT_DEPTH_MASK) != CV_8U)
                throw(parameter_error("image must be unsigned bytes"));
            if(img->cvMat().channels() > 3)
                throw(parameter_error("image must be <= 3-channel"));
                // TODO: support vector parameters
            
            float pct = param<float>("percentile");
            
            const int channels = img->cvMat().channels();
            const int rows = img->cvMat().rows;
            const int cols = img->cvMat().cols;
            const int elem_size = img->cvMat().elemSize();
            const int num_pixels = rows * cols;
            const int pct_pixel = int(num_pixels * (pct/100.0f) + 0.5f);

            std::vector< std::vector<int> > value_histogram(3, std::vector<int>(256, 0));
            
            const unsigned char *rp, *cp, *bp;
            int row, col, ch;

            for(row = 0; row < rows; row++){
                rp = img->cvMat().ptr(row);
                for(col = 0, cp = rp; col < cols; col++, cp += elem_size)
                    for(ch = 0, bp = cp; ch < channels; ch++, bp++)
                        value_histogram[ch][*bp]++;
            }

            param_value_t channel_results[3] = {
                param_value_t(int(0)),
                param_value_t(int(0)),
                param_value_t(int(0))
            };
            
            for(int ch = 0; ch < channels; ch++){
                int running_total = 0;
                for(int i = 0; i < 256; i++){
                    debug(9) << "[" << BashColour::White << bar(running_total, num_pixels, 50) << "]"
                              << i << running_total;
                    if((running_total += value_histogram[ch][i]) >= pct_pixel){
                        channel_results[ch] = param_value_t(i);
                        break;
                    }
                }
            }

            r["ch1 value"] = channel_results[0];
            r["ch2 value"] = channel_results[1];
            r["ch3 value"] = channel_results[2];

            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __PERCENTILE_NODE_H__
