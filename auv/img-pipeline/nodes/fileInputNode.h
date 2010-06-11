#ifndef __FILE_INPUT_NODE_H__
#define __FILE_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "asynchronousNode.h"


class FileInputNode: public AsynchronousNode{
        typedef boost::lock_guard<boost::recursive_mutex> lock_t;

    public:
        FileInputNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : AsynchronousNode(sched, pl, t){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID<image_ptr_t>("image_out");
            
            // one parameter: the filename
            registerParamID<std::string>("filename", "default.jpg");
        } 

        virtual void paramChanged(param_id const& p){
            debug() << "FileInputNode::paramChanged";
            if(p == param_id("filename"))
                setAllowQueue();
            else
                warning() << "unknown parameter" << p << "set";
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            out_map_t r;
            
            debug() << "fileInputNode::doWork";
        
            std::string fname = param<std::string>("filename"); 
            cv::Mat img = cv::imread(fname.c_str());

            if(img.size().width > 0 && img.size().height > 0){
                r["image_out"] = image_ptr_t(new Image(img, Image::src_file)); 
                debug() << "fileInputNode::doWork result:" << fname << "->"
                        << *boost::get<image_ptr_t>(r["image_out"]);
            }else{
                warning() << "fileInputNode::doWork result:" << fname << "-> (no image)";
            }
            clearAllowQueue();            

            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __FILE_INPUT_NODE_H__
