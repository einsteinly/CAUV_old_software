#ifndef __FILE_INPUT_NODE_H__
#define __FILE_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <common/debug.h>

#include "../node.h"
#include "../image.h"


class FileInputNode: public Node{
    public:
        FileInputNode(Scheduler& s)
            : Node(s){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID("image_out");
            
            // one parameter: the filename
            registerParamID<std::string>("filename", "default.jpg");
        }

    protected:
        out_image_map_t doWork(in_image_map_t&){
            out_image_map_t r;
            
            debug() << "fileInputNode::doWork";
        
            std::string fname = param<std::string>("filename"); 
            cv::Mat img = cv::imread(fname.c_str());

            if(img.size().width > 0 && img.size().height > 0){
                r["image_out"] = image_ptr_t(new Image(img, Image::file)); 
                debug() << "fileInputNode::doWork result:" << fname << "->" << *r["image_out"];
            }else{
                debug() << "fileInputNode::doWork result:" << fname << "->" << "(no image)";
            }

            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __FILE_INPUT_NODE_H__
