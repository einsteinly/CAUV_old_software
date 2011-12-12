#ifndef __THRESHOLD_MASK_NODE_H__
#define __THRESHOLD_MASK_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <common/cauv_utils.h>

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

        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            int threshold = param<int>("threshold (>= is masked)");         

            cv::Mat img = inputs["channel (not copied)"]->mat();
            
            const int max_value=255;
            try {
                cv::threshold(img, img, threshold, max_value, cv::THRESH_BINARY);
            }catch(cv::Exception& e){
                error() << "ThresholdMaskNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }

            r["output mask"] = boost::make_shared<Image>(img);
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __THRESHOLD_MASK_NODE_H__
