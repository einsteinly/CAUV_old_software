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
        ConvertColourNode(Scheduler& s)
            : Node(s){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image_in");
            
            // one output
            registerOutputID("image_out");
            
            // parameter:
            registerParamID<int>("code", CV_RGB2GRAY);
        }

    protected:
        out_image_map_t doWork(in_image_map_t& inputs){
            out_image_map_t r;

            image_ptr_t img = inputs["image_in"];
            
            int code = param<int>("code");
            
            boost::shared_ptr<Image> out = boost::make_shared<Image>();
            out->source(img->source());
            
            try{
                cv::cvtColor(out->cvMat(), img->cvMat(), code);
                r["image_out"] = out;
            }catch(cv::Exception& e){
                error() << "ConvertColourNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __CONVERTCOLOUR_NODE_H__

