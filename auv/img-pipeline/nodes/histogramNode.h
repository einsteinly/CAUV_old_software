#ifndef __HISTOGRAM_H__
#define __HISTOGRAM_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"

namespace cauv{
namespace imgproc{

class HistogramNode: public Node{
    public:
        HistogramNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            //One input
            registerInputID("image_in", Const);
            
            //Output histogram
            registerOutputID("histogram", std::vector<float>());
            
            //Parameters
            registerParamID<int>("Number of bins", 42);
            registerParamID<std::string>("name", "unused" "");
            
        }

    protected:
        struct calcHistogram: boost::static_visitor< std::vector<float> >{
            calcHistogram(int bins) : m_bins(bins){}
            std::vector<float> operator()(cv::Mat a) const{
                if(!a.isContinuous())
                    throw(parameter_error("image must be continuous"));
                if((a.type() & CV_MAT_DEPTH_MASK) != CV_8U)
                    throw(parameter_error("image must have unsigned bytes"));
                if(a.channels() > 1)
                    throw(parameter_error("image must have only one channel"));
                    //TODO: support vector parameters

                int histSize[] = {m_bins};
                //Hue varies from 0 to 179, see cvtColor
                float hranges[] = {0, 256};

                const float* ranges[] = {hranges};
                cv::MatND hist;
                //We compute the histogram from the 0-th and 1-st channels
                int channels[] = {0};

                cv::calcHist( &a, 1, channels , cv::Mat(), //Do not use mask
                    hist, 1, histSize, ranges,
                    true, //The histogram is uniform
                    false );
                double maxVal = 0;
                minMaxLoc(hist, 0, &maxVal, 0, 0);
                int imgsize = a.rows * a.cols;

                std::vector<float> binVal;
                for(int h = 0; h < m_bins; h++){
                    binVal.push_back(hist.at<float>(h) / imgsize);
                }

                return binVal;
            }
            std::vector<float> operator()(NonUniformPolarMat a) const{
                return operator()(a.mat);
            }
            std::vector<float> operator()(PyramidMat a) const{
                // not sure if this is the right thing to do:
                return operator()(a.levels.at(0));
            }
            const int m_bins;
        };
        void doWork(in_image_map_t& inputs, out_map_t& r){

            const int bins = param<int>("Number of bins");
            augmented_mat_t img = inputs["image_in"]->augmentedMat();
            r["histogram"] = ParamValue(boost::apply_visitor(calcHistogram(bins), img));

        }

    //Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif //ndef __HISTOGRAM_H__
