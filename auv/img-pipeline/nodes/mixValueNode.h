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

#ifndef __MIX_VALUE_NODE_H__
#define __MIX_VALUE_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>

#include <common/msg_classes/colour.h>

#include "../node.h"


namespace cauv{
namespace imgproc{

class MixValueNode: public Node{
    public:
        MixValueNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("image");

            // one output
            registerOutputID("image (not copied)");
            
            // parameters:
            registerParamID<float>("image fac", 1, "Image value factor");
            registerParamID<float>("value fac", 1, "Mixing value factor");
            registerParamID<Colour>("value", Colour::fromRGB(0,0,0), "Colour to mix in");
        }

    protected:
        template<typename T, int Channels>
        static void channelMix(float a, cv::Mat& m, float b, const Colour& colour)
        {
            typedef cv::Vec<T,Channels> pixel_t;
            cv::Mat_<float> similarityMat(m.rows, m.cols);

            cv::MatIterator_<pixel_t> it, itend;

            // Iterate over all pixels...
            for (it = m.begin<pixel_t>(), itend = m.end<pixel_t>();
                 it != itend; ++it) {
                // .. and all channels per pixel (assume images are bgr(a))
                pixel_t& pixel = *it;
                for (int c = 0; c < Channels; ++c)
                    pixel[c] = clamp_cast<T>(a*pixel[c] + b*255.0f*colour.values[c]); 
            }
        }
        template<typename T>
        static void mix(float a, cv::Mat& m, float b, const Colour& colour)
        {
            int nchannels = m.channels();

            if (nchannels == 3 && (colour.type == ColourType::RGB || colour.type == ColourType::BGR))
                channelMix<T,3>(a, m, b, colour);
            else if (nchannels == 4 && (colour.type == ColourType::ARGB || colour.type == ColourType::BGRA))
                channelMix<T,4>(a, m, b, colour);
            else if (nchannels == 1 && (colour.type == ColourType::Grey))
                channelMix<T,1>(a, m, b, colour);
            else {
                error() << "Cannot use a" << nchannels << "image with" << colour.type;
            }
        }

        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image"];
            
            float img_f = param<float>("image fac");
            float value_f = param<float>("value fac");
            Colour value =  param<Colour>("value");
            
            img->apply(boost::bind(mix<unsigned char>, img_f, _1, value_f, value));
            
            r["image (not copied)"] = img;
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __MIX_VALUE_NODE_H__
