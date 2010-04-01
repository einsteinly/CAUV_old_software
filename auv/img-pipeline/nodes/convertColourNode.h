#ifndef __CONVERTCOLOUR_NODE_H__
#define __CONVERTCOLOUR_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"


class ConvertColourNode: public Node{
    public:
        ConvertColourNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image_in");
            
            // one output
            registerOutputID("image_out");
            
            // parameter:
            registerParamID<int>("code", CV_RGB2GRAY);
            registerParamID<int>("channels", 0);
        }

    protected:
        out_image_map_t doWork(in_image_map_t& inputs){
            out_image_map_t r;

            image_ptr_t img = inputs["image_in"];
            
            int code = param<int>("code");
            int channels = param<int>("channels");
            
            boost::shared_ptr<Image> out = boost::make_shared<Image>();
            out->source(img->source());
            
            try{
                cv::cvtColor(img->cvMat(), out->cvMat(), code, channels);
                r["image_out"] = out;
            }catch(cv::Exception& e){
                error() << "ConvertColourNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line << "\n\t"
                        << "The parameters to this node are:\n\t"
                        << "code = " << param<int>("code") << "\n\t"
                        << "channels = " << param<int>("channels") << "\n\t"
                        << "The inputs to this node are:\n\t"
                        << "*image_in = " << *img;
            }
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __CONVERTCOLOUR_NODE_H__

