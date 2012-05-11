#ifndef __THRESHOLD_MASK_NODE_H__
#define __THRESHOLD_MASK_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class ThresholdMaskNode: public Node{
    public:
        ThresholdMaskNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input
            registerInputID("channel (not copied)"); // must be matrix of a single channel

            // one output
            registerOutputID("output mask");

            // one parameter
            registerParamID<int>("threshold (>= is masked)",127);
        }

    protected:
        struct applyThreshold: boost::static_visitor<void>{
            applyThreshold(int threshold) : m_thresh(threshold){ }
            void operator()(cv::Mat a) const{
                const int max_value = 255;
                cv::threshold(a, a, m_thresh, max_value, cv::THRESH_BINARY);
            }
            void operator()(NonUniformPolarMat a) const{
                operator()(a.mat);
            }
            void operator()(PyramidMat) const{
                error() << "not implemented";
            }
            private:
            const int m_thresh;
        };
        void doWork(in_image_map_t& inputs, out_map_t& r){

            int threshold = param<int>("threshold (>= is masked)");         

            try {
                augmented_mat_t m = inputs["channel (not copied)"]->augmentedMat();
                boost::apply_visitor(applyThreshold(threshold), m);
                r["output mask"] = boost::make_shared<Image>(m);
            }catch(cv::Exception& e){
                error() << "ThresholdMaskNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }

            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __THRESHOLD_MASK_NODE_H__
