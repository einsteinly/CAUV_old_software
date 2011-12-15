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
            registerOutputID("ch1 value", int(0));
            registerOutputID("ch2 value", int(0));
            registerOutputID("ch3 value", int(0));
            
            // parameter: 
            registerParamID<float>("percentile", 50, "0-100 percentile of pixel values");
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
        
        struct apply: boost::static_visitor< std::vector<int> >{
            apply(float const& percentile): m_pct(percentile){ }
            std::vector<int> operator()(cv::Mat img) const{
                const int channels = img.channels();
                const int pct_pixel = int(img.total() * (m_pct/100.0f));
                
                if(channels > 3 || channels < 1)
                    throw(parameter_error("image must have 1--3 channels"));
                if(img.depth() != CV_8U)
                    throw(parameter_error("image must be unsigned bytes"));

                std::vector< std::vector<uint32_t> > value_histogram(channels, std::vector<uint32_t>(256, 0));
                
                int dims[3] = {img.rows, img.cols, channels};
                size_t steps[2] = {img.step[0], img.step[1]};
                cv::Mat imgWithChannels(3, dims, CV_8U, img.data, steps);
                cv::MatConstIterator_<uint8_t> it = imgWithChannels.begin<unsigned char>(),
                                               end = imgWithChannels.end<unsigned char>();
                // FIXME: opencv bug? iterator version is broken for me on
                // single channel images (begin+1 == end)
                /*while(it != end) {
                    for(int ch = 0; ch < channels; ch++) {
                        value_histogram[ch][*it]++;
                    }
                    it++;
                }*/
                for(int row = 0; row < img.rows; row++)
                    for(int col = 0; col < img.cols; col++)
                        for(int ch = 0; ch < channels; ch++)
                            value_histogram[ch][img.at<uint8_t>(row,col,ch)]++;

                std::vector<int> channel_results;
                for(int ch = 0; ch < channels; ch++){
                    int running_total = 0;
                    int i;
                    for(i = 0; i < 256; i++){
                        //debug(9) << "[" << BashColour::White << bar(running_total, img.total(), 50) << "]"
                        //         << i << running_total;
                        
                        running_total += value_histogram[ch][i];
                        if(running_total >= pct_pixel){
                            channel_results.push_back(i);
                            break;
                        }
                    }
                    if(i == 256)
                        channel_results.push_back(255);
                }
                assert(channel_results.size() == uint32_t(channels));

                return channel_results;
            }
            std::vector<int> operator()(NonUniformPolarMat a) const{
                return operator()(a.mat);
            }
            std::vector<int> operator()(PyramidMat) const{
                error() << "percentile does not support pyramids";
                return std::vector<int>();
            }
            float m_pct;
        };
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            augmented_mat_t img = inputs["image"]->augmentedMat();
            
            float pct = param<float>("percentile");
            
            std::vector<int> channel_results = boost::apply_visitor(apply(pct), img);

            //bool removed = true;
            unsigned ch;
            for (ch = 0; ch < channel_results.size() && ch < 3; ++ch) {
                output_id id = MakeString() << "ch" << (ch+1) << " value";
                //registerOutputID<ParamValue>(id, false);
                r[id] = ParamValue(channel_results[ch]);
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
