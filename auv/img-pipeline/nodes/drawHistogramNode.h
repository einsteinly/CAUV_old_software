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

#ifndef __DRAW_HISTOGRAM_NODE_H__
#define __DRAW_HISTOGRAM_NODE_H__

#include "../node.h"

#include <algorithm>

namespace cauv{
namespace imgproc{

class DrawHistogramNode: public Node{
    public:
        DrawHistogramNode(ConstructArgs const& args)
            : Node(args), m_counter(0){
        }

        void init(){
            registerParamID< std::vector<float> >(
                "histogram", std::vector<float>(), "values to plot", Must_Be_New
            );
            registerOutputID(Image_Out_Copied_Name);
        }
    
        virtual ~DrawHistogramNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            const unsigned Min_Width = 256;
            const unsigned rows = 100;
            unsigned cols_per_bar = 1;
            out_map_t r;
            
            const std::vector<float> histogram = param< std::vector<float> >("histogram");
            if(!histogram.size())
                throw parameter_error("no histogram to draw");

            if(histogram.size() < Min_Width)
                cols_per_bar = int(0.5 + Min_Width / histogram.size());

            cv::Mat out = cv::Mat::zeros(rows, histogram.size() * cols_per_bar, CV_8UC1);

            float max = *std::max_element( histogram.begin(), histogram.end());
            for(unsigned i = 0; i < histogram.size(); i++)
                for(unsigned k = 0; k < cols_per_bar; k++)
                    for(unsigned j = 1; j <= unsigned(rows * histogram[i]/max); j++){
                        assert(rows >= j);
                        assert(i*cols_per_bar+k < histogram.size()*cols_per_bar);
                        out.at<uint8_t>(rows - j, i*cols_per_bar+k) = 0xff;
                    }

            r[Image_Out_Copied_Name] = boost::make_shared<Image>(out);

            return r;
        }

        int m_counter;
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __DRAW_HISTOGRAM_NODE_H__
