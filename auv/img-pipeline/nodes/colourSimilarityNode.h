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

#ifndef __COLOUR_SIMILARITY_NODE_H__
#define __COLOUR_SIMILARITY_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>

#include <common/msg_classes/colour.h>

#include "../node.h"


namespace cauv{
namespace imgproc{

class ColourSimilarityNode: public Node{
    public:
        ColourSimilarityNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("image");

            // one output
            registerOutputID("image");
            
            // parameters:
            registerParamID<Colour>("colour", Colour::fromRGB(1,1,0), "Colour to compare against");
            registerParamID<float>("sigma", 1, "Sigma to use for converting distance to similarity");
        }

    protected:
        template<typename T, int Channels>
        static cv::Mat channelSimilarity(const cv::Mat& m, const Colour& colour, float sigma)
        {
            typedef cv::Vec<T,Channels> pixel_t;
            
            cv::Mat_<float> similarityMat(m.rows, m.cols);

            cv::MatConstIterator_<pixel_t> it, itend;
            cv::MatIterator_<float> sit;

            // Iterate over all pixels...
            for (it = m.begin<pixel_t>(), itend = m.end<pixel_t>(), sit = similarityMat.begin();
                 it != itend; ++it, ++sit) {
                // .. and all channels per pixel (assume images are bgr(a))
                double sqdist = 0;
                const pixel_t& pixel = *it;
                for (int c = 0; c < Channels; ++c)
                    sqdist += (pixel[c]/255.0 - colour.values[c]) * (pixel[c]/255.0 - colour.values[c]);

                *sit = (float)std::exp(-sqdist/(2*sigma*sigma));
            }

            return similarityMat;
        }
        template<typename T>
        static cv::Mat similarity(const cv::Mat& m, const Colour& colour, float sigma)
        {
            int nchannels = m.channels();

            if (nchannels == 3 && (colour.type == ColourType::RGB || colour.type == ColourType::BGR))
                return channelSimilarity<T,3>(m, colour, sigma);
            else if (nchannels == 4 && (colour.type == ColourType::ARGB || colour.type == ColourType::BGRA))
                return channelSimilarity<T,4>(m, colour, sigma);
            else if (nchannels == 1 && (colour.type == ColourType::Grey))
                return channelSimilarity<T,1>(m, colour, sigma);
            else {
                error() << "Cannot use a" << nchannels << "image with" << colour.type;
                return cv::Mat();
            }
        }

        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image"];
            
            Colour colour = param<Colour>("colour");
            float sigma = param<float>("sigma");

            r["image"] = boost::make_shared<Image>(img->apply(boost::bind(similarity<unsigned char>, _1, colour, sigma)));
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __COLOUR_SIMILARITY_NODE_H__
