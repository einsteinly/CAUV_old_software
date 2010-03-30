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
        FileInputNode(Scheduler& sched, ImageProcessor& pl)
            : AsynchronousNode(sched, pl), m_file_is_new(true){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID("image_out");
            
            // one parameter: the filename
            registerParamID<std::string>("filename", "default.jpg");
        } 

        template<typename T>
        void paramChanged(param_id const& p, T const& new_value){
            if(p == param_id("filename")){
                lock_t l(m_file_is_new_lock);
                m_file_is_new = true;
            }
        }

    protected:
        out_image_map_t doWork(in_image_map_t&){
            lock_t l(m_file_is_new_lock);
            out_image_map_t r;
            
            debug() << "fileInputNode::doWork";
        
            std::string fname = param<std::string>("filename"); 
            cv::Mat img = cv::imread(fname.c_str());

            if(img.size().width > 0 && img.size().height > 0){
                r["image_out"] = image_ptr_t(new Image(img, Image::src_file)); 
                m_file_is_new = false;
                debug() << "fileInputNode::doWork result:" << fname << "->" << *r["image_out"];
            }else{
                debug() << "fileInputNode::doWork result:" << fname << "->" << "(no image)";
            }

            return r;
        } 
        
        virtual bool allowQueueExec() throw(){
            lock_t l(m_file_is_new_lock); 
            return m_file_is_new;
        }

    private:
        boost::recursive_mutex m_file_is_new_lock;
        bool m_file_is_new;
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __FILE_INPUT_NODE_H__
