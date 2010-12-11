//Quick segment node takes an image
#ifndef __QUICKSEGMENT_NODE_H__
#define __QUICKSEGMENT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>

#include <generated/messages.h>

#include "../node.h"
#include "outputNode.h"


class QuickSegmentNode: public OutputNode{
    public:
        QuickSegmentNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : OutputNode(sched, pl, t){
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
                    for(src_it = img->cvMat().begin<cv::Scalar>(),
                        dest_it = out->cvMat().begin<cv::Scalar>();
                        src_it != img->cvMat().end<cv::Scalar>();
                        ++src_it, ++dest_it)
                    {
                        for(int i = 0; i < 3; i++)
                        {
                            cv::Scalar pix = *src_it;
                            if((pix[i] - mean[i]) * (pix[i] - mean[i]) <
                                scale * stdev[i] * scale * stdev[i])
                            {
                                *dest_it = 1;
                                break;
                            }
                        }
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

#endif // ndef __QUICKSEGMENT_NODE_H__

