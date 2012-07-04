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
        template<typename T>
        static cv::Mat similarity(const cv::Mat& m, const Colour& colour, float sigma)
        {
            int nchannels = m.channels();
            if ((nchannels == 3 && colour.type != ColourType::RGB && colour.type != ColourType::BGR)
                || (nchannels == 4 && colour.type != ColourType::ARGB && colour.type != ColourType::BGRA)
                || (nchannels == 1 && colour.type != ColourType::Grey)) {
                error() << "Wrong colour type";
                return cv::Mat();
            }

            cv::Mat_<float> similarityMat(m.cols, m.rows);

            cv::MatConstIterator_<T> it, itend;
            cv::MatIterator_<float> sit;

            // Iterate over all pixels...
            for (it = m.begin<T>(), itend = m.end<T>(), sit = similarityMat.begin();
                 it != itend; ++sit) {
                // .. and all channels per pixel
                double sqdist = 0;
                for (int c = 0; c < nchannels; ++c, ++it)
                    sqdist += (*it/255.0 - colour.values[c]) * (*it/255.0 - colour.values[c]);

                *sit = (float)std::exp(-sqdist/(2*sigma*sigma));
            }

            return similarityMat;
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
