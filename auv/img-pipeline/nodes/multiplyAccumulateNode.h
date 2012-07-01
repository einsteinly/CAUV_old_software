#ifndef __MULTIPLYACCUMULATE_H__
#define __MULTIPLYACCUMULATE_H__

#include <vector>
#include <cmath>

#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>

#include "../node.h"
#include "../nodeFactory.h"
#include "../pipelineTypes.h"

namespace cauv{
namespace imgproc{

class MultiplyAccumulateNode : public Node{
    public:
        MultiplyAccumulateNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            //Fast node
            m_speed = fast;
            
            registerInputID("image");
           
            registerOutputID("image (not copied)");

            registerParamID<float>("b", 0, "Multiplier");
            registerParamID<std::vector<float> >("value", std::vector<float>(), "Value to multiply/accumulate");
        }

    protected:
        template<typename T>
        static void fma(cv::Mat& m, float b, const std::vector<float>& value)
        {
            int nchannels = m.channels();
            if (nchannels > (int)value.size())
                throw parameter_error("not enough channel values");

            // Iterate over all pixels...
            for (cv::MatIterator_<T> it = m.begin<T>(),
                                     itend = m.end<T>();
                 it != itend; ++it) {
                // .. and all channels per pixel
                for (int c = 0; c < nchannels; ++c, ++it)
                    *it = round(*it + b*value[c]); 
            }
        }

        void doWork(in_image_map_t& inputs, out_map_t& r){
            image_ptr_t img = inputs["image"];

            img->apply(boost::bind(fma<unsigned char>, _1, param<float>("b"), param<std::vector<float> >("value")));

            r["image (not copied)"] = img;
        }

    //Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif //ndef __MULTIPLYACCUMULATE_H__
