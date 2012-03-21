//Quick segment node takes an image
#ifndef __QUICKSEGMENT_NODE_H__
#define __QUICKSEGMENT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

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
            registerOutputID("mask");
            
            // parameters:
            registerParamID<float>("scale", 1.0);
        }
        
    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){

            cv::Mat img = inputs["image"]->mat();
            
            float scale = param<float>("scale");

            cv::Scalar mean, stdev;

            try{
                cv::meanStdDev(img, mean, stdev);
                cv::Mat out = cv::Mat::zeros(img.size(), CV_8UC1);

                cv::MatConstIterator_<cv::Scalar> src_it, dest_it;
                for(src_it = img.begin<cv::Scalar>(),
                    dest_it = out.begin<cv::Scalar>();
                    src_it != img.end<cv::Scalar>();
                    ++src_it, ++dest_it)
                {
                    for(int i = 0; i < 3; i++)
                    {
                        if(cv::norm(*src_it - mean) < scale * cv::norm(stdev - mean))
                            *dest_it = 255;
                    }
                }
                r["mask"] = boost::make_shared<Image>(out);
                    
            }catch(cv::Exception& e){
                error() << "QuickSegmentNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __QUICKSEGMENT_NODE_H__

