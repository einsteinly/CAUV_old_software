#ifndef __PERCENTILE_NODE_H__
#define __PERCENTILE_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class PercentileNode: public Node{
    public:
        PercentileNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image");

            // output parameters:
            registerOutputID<NodeParamValue>("ch1 value");
            registerOutputID<NodeParamValue>("ch2 value");
            registerOutputID<NodeParamValue>("ch3 value");
            // [dynamic]
            
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

            cv::Mat img = inputs["image"]->mat();
            
            if(img.channels() > 3)
                throw(parameter_error("image must have 3 or fewer channels"));
            if(img.depth() != CV_8U)
                throw(parameter_error("image must be unsigned bytes"));
            
            float pct = param<float>("percentile");
            
            const int channels = img.channels();
            const int pct_pixel = int(img.total() * (pct/100.0f) + 0.5f);

            std::vector< std::vector<int> > value_histogram(channels, std::vector<int>(256, 0));
            
            int dims[3] = {img.rows, img.cols, channels};
            size_t steps[2] = {img.step[0], img.step[1]};
            cv::Mat imgWithChannels(3, dims, CV_8U, img.data, steps);
            cv::MatIterator_<unsigned char> it = imgWithChannels.begin<unsigned char>(),
                                           end = imgWithChannels.end<unsigned char>();
            while(it != end) {
                for(int ch = 0; ch < channels; ch++, it++) {
                    value_histogram[ch][*it]++;
                }
            }

            std::vector<NodeParamValue> channel_results;
            
            for(int ch = 0; ch < channels; ch++){
                int running_total = 0;
                for(int i = 0; i < 256; i++){
                    debug(9) << "[" << BashColour::White << bar(running_total, img.total(), 50) << "]"
                              << i << running_total;
                    
                    running_total += value_histogram[ch][i];
                    if(running_total >= pct_pixel){
                        channel_results.push_back(NodeParamValue(i));
                        break;
                    }
                }
            }

            bool removed = true;
            int ch;
            for (ch = 0; ch < channels; ++ch) {
                output_id id = MakeString() << "ch" << (ch+1) << " value";
                //registerOutputID<NodeParamValue>(id, false);
                r[id] = channel_results[ch];
            }
            //for (; removed; ++ch)
                //if (ch > 0)
                    //removed = unregisterOutputID(MakeString() << "ch" << (ch+1) << " value", false);


            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __PERCENTILE_NODE_H__
