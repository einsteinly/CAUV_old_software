#ifndef __FILE_INPUT_NODE_H__
#define __FILE_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include "cv.h"

#include "node.h"
#include "image.h"


class FileInputNode: public Node{
    public:
        FileInputNode(Scheduler& s)
            : Node(s){
            
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID("image from file");
            
            // TODO: strong typing using templates for parameters?
            //registerParamID<std::string>("filename");
            
            // one parameter: the filename
            registerParamID<std::string>("filename", "default.jpg");
            
        }

    protected:
        out_image_map_t doWork(in_image_map_t const& inputs){
            std::string fname = param<std::string>("filename");
            
        }
};

#endif // ndef __FILE_INPUT_NODE_H__
