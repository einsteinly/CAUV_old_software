/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

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
            registerOutputID("value", Colour::fromRGB(0,0,0));
            
            // parameter: 
            registerParamID<BoundedFloat>("percentile", BoundedFloat(50,0,100,BoundedFloatType::Clamps), "percentile of pixel values");
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
        
        static Colour getPercentile(cv::Mat& img, float pct) {
            const int channels = img.channels();
            const int pct_pixel = int(img.total() * (pct/100.0f));
            
            if(channels != 1 && channels != 3)
                throw(parameter_error("image must have 1 or 3 channels"));
            if(img.depth() != CV_8U)
                throw(parameter_error("image must be unsigned bytes"));

            std::vector< std::vector<uint32_t> > value_histogram(channels, std::vector<uint32_t>(256, 0));
            
            for (cv::MatConstIterator_<uint8_t> it = img.begin<unsigned char>(),
                                                end = img.end<unsigned char>();
                 it != end;) {
                for(int ch = 0; ch < channels; ++ch, ++it) {
                    value_histogram[ch][*it]++;
                }
            }

            std::vector<float> channel_results;
            for(int ch = 0; ch < channels; ch++){
                int running_total = 0;
                int i;
                for(i = 0; i < 256; i++){
                    //debug(9) << "[" << BashColour::White << bar(running_total, img.total(), 50) << "]"
                    //         << i << running_total;
                    
                    running_total += value_histogram[ch][i];
                    if(running_total >= pct_pixel){
                        channel_results.push_back(i/255.0f);
                        break;
                    }
                }
                if(i == 256)
                    channel_results.push_back(1.0f);
            }
            assert(channel_results.size() == uint32_t(channels));

            if (channels == 3)
                return Colour::fromBGR(channel_results[0], channel_results[1], channel_results[2]);
            else 
                return Colour::fromGrey(channel_results[0]);
        }
        
        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image"];
            
            float pct = param<BoundedFloat>("percentile");
            
            r["value"] = img->apply(boost::bind(getPercentile, _1, pct));
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __PERCENTILE_NODE_H__
