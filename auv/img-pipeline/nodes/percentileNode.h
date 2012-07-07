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
            registerInputID("image", true);

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
        
        template<typename T, int Channels>
        struct channelsToColourHelper;
        
        template<typename T>
        struct channelsToColourHelper<T, 1> {
            static Colour colour(boost::array<T, 1> vals) {
                return Colour::fromGrey(vals[0]);
            }
        };
        template<typename T>
        struct channelsToColourHelper<T, 3> {
            static Colour colour(boost::array<T, 3> vals) {
                return Colour::fromBGR(vals[0], vals[1], vals[2]);
            }
        };
        template<typename T>
        struct channelsToColourHelper<T, 4> {
            static Colour colour(boost::array<T, 4> vals) {
                return Colour::fromBGRA(vals[0], vals[1], vals[2], vals[3]);
            }
        };
        
        template<typename T, int Channels>
        static Colour getPercentileChannels(const cv::Mat& img, float pct) {
            typedef cv::Vec<T,Channels> pixel_t;

            boost::array< boost::array<uint32_t, 256>, Channels > value_histogram;;

           
            for (cv::MatConstIterator_<pixel_t> it = img.begin<pixel_t>(),
                                                end = img.end<pixel_t>();
                 it != end; ++it) {
                const pixel_t& pixel = *it;
                for(int ch = 0; ch < Channels; ++ch) {
                    value_histogram[ch][pixel[ch]]++;
                }
            }

            const int pct_pixel = int(img.total() * (pct/100.0f));
            boost::array<float, Channels> channel_results;
            for(int ch = 0; ch < Channels; ch++){
                int running_total = 0;
                int i;
                for(i = 0; i < 256; i++){
                    //debug(9) << "[" << BashColour::White << bar(running_total, img.total(), 50) << "]"
                    //         << i << running_total;
                    
                    running_total += value_histogram[ch][i];
                    if(running_total >= pct_pixel){
                        channel_results[ch] = i/256.0f;
                        break;
                    }
                }
                if(i == 256)
                    channel_results[ch] = 1.0f;
            }

            return channelsToColourHelper<float,Channels>::colour(channel_results);
        }
        
        static Colour getPercentile(const cv::Mat& img, float pct) {
            const int channels = img.channels();
            
            if(img.depth() != CV_8U)
                throw(parameter_error("image must be unsigned bytes"));
            
            if (channels == 3)
                return getPercentileChannels<unsigned char, 3>(img, pct);
            else if (channels == 4)
                return getPercentileChannels<unsigned char, 4>(img, pct);
            else if (channels == 1) 
                return getPercentileChannels<unsigned char, 1>(img, pct);
            else
                throw(parameter_error("image must have 1, 3 or 4 channels"));

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
