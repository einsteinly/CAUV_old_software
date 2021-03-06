/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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

    protected:
        void doWork(in_image_map_t&, out_map_t& r){
            const unsigned Min_Width = 256;
            const unsigned rows = 100;
            unsigned cols_per_bar = 1;
            
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
        }

        int m_counter;
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __DRAW_HISTOGRAM_NODE_H__
