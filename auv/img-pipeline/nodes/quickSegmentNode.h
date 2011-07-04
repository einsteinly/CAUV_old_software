//Quick segment node takes an image
#ifndef __QUICKSEGMENT_NODE_H__
#define __QUICKSEGMENT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include <generated/messages.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class QuickSegmentNode: public OutputNode{
    public:
        QuickSegmentNode(ConstructArgs const& args)
            : OutputNode(args){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            // one input:
            registerInputID("image");
            
            // one output:
            registerOutputID<image_ptr_t>("mask");
            
            // parameters:
            registerParamID<float>("scale", 1.0);
        }
    
        virtual ~QuickSegmentNode(){
            stop();
        }
        
    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image"];
            
            float scale = param<float>("scale");

            cv::Scalar mean, stdev;

            try{
                    cv::meanStdDev(img->cvMat(), mean, stdev);
                    boost::shared_ptr<Image> out = boost::make_shared<Image>();
                    out->cvMat().zeros(img->cvMat().size(), CV_8UC1);

                    cv::MatConstIterator_<cv::Scalar> src_it, dest_it;
                    cv::MatConstIterator_<cv::Scalar> end_it =
                        img->cvMat().end<cv::Scalar>();
                    for(src_it = img->cvMat().begin<cv::Scalar>(),
                        dest_it = out->cvMat().begin<cv::Scalar>();
                        src_it != end_it;
                        ++src_it, ++dest_it)
                    {
                        if(cv::norm(*src_it - mean) < scale * cv::norm(stdev))
                            *dest_it = 255;
                    }
                    r["mask"] = out;
                    
            }catch(cv::Exception& e){
                error() << "QuickSegmentNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            return r;
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __QUICKSEGMENT_NODE_H__

