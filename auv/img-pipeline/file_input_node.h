#ifndef __FILE_INPUT_NODE_H__
#define __FILE_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "node.h"
#include "image.h"


class FileInputNode: public Node{
    public:
        FileInputNode(Scheduler& s)
            : Node(s){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID("image");
            
            // one parameter: the filename
            registerParamID<std::string>("filename", "default.jpg");
        }

    protected:
        out_image_map_t doWork(in_image_map_t const&){
            out_image_map_t r;
        
            std::string fname = param<std::string>("filename");
            
            cv::Mat img = cv::imread(fname.c_str());
            
            r["image"] = image_ptr_t(new Image(img, Image::file));
            
            return r;
        }
};

#endif // ndef __FILE_INPUT_NODE_H__
