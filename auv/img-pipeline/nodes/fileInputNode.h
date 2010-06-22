#ifndef __FILE_INPUT_NODE_H__
#define __FILE_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/filesystem.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "asynchronousNode.h"


class FileInputNode: public AsynchronousNode{
        typedef boost::unique_lock<boost::recursive_mutex> lock_t;
    public:
        FileInputNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : AsynchronousNode(sched, pl, t), m_dir_mutex(),
              m_is_directory(false), m_iter(){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID<image_ptr_t>("image");
            
            // one parameter: the filename
            registerParamID<std::string>("filename", "default.jpg");
        } 

        virtual void paramChanged(param_id const& p){
            debug() << "FileInputNode::paramChanged";
            if(p == param_id("filename")){
                lock_t l(m_dir_mutex);
                std::string fname = param<std::string>("filename");
                if(boost::filesystem::is_directory(fname)){
                    m_is_directory = true;
                    closeVideo();
                }else{
                    m_is_directory = false;
                    openVideo(fname);
                }
                m_iter = boost::filesystem::directory_iterator();
                setAllowQueue();
            }else{
                warning() << "unknown parameter" << p << "set";
            }
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            out_map_t r;
            
            debug() << "fileInputNode::doWork";
        
            std::string fname = param<std::string>("filename");
            image_ptr_t image;

            if(!m_is_directory){
                if(!m_capture.isOpened()){
                    image = readImage(fname);
                    clearAllowQueue();
                }else{
                    image = boost::make_shared<Image>();
                    m_capture >> image->cvMat();
                    r["image"] = image;
                }
            }else{
                lock_t l(m_dir_mutex);
                const boost::filesystem::directory_iterator end;
                if(m_iter == end)
                    m_iter = boost::filesystem::directory_iterator(fname);
                // continue until the directory is exhausted, or an image is
                // successfully loaded
                for(; m_iter != end; m_iter++){
                    debug()  << "considering path:" << m_iter->string();
                    if(!boost::filesystem::is_directory(m_iter->status()) &&
                       (image = readImage(m_iter->string(), false))){
                        m_iter++;
                        break;
                    }
                }
                if(!image)
                    warning() << "no images in directory" << fname;
                // NB: allowQueue not cleared
            }
            if(image)
                image->source(Image::src_file);
            r["image"] = image;

            return r;
        }

        image_ptr_t readImage(std::string const& fname, bool warn=true) const{
            image_ptr_t r;
            cv::Mat img;

            try{
                img = cv::imread(fname.c_str());
            }catch(cv::Exception& e){
                error() << "FileInputNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }

            if(img.size().width > 0 && img.size().height > 0){
                r = boost::make_shared<Image>(img, Image::src_file);
                debug() << "fileInputNode::readImage:" << fname << "->" << *r;
            }else if(warn){
                warning() << "fileInputNode::readImage:" << fname << "-> (no image)";
            }

            return r;
        }

        bool openVideo(std::string const& fname){
            lock_t l(m_capture_lock);
            m_capture = cv::VideoCapture(fname);
            return m_capture.isOpened();
        }

        void closeVideo(){
            m_capture.release();
        }

    private:
        boost::recursive_mutex m_dir_mutex;

        bool m_is_directory;
        boost::filesystem::directory_iterator m_iter;

        cv::VideoCapture m_capture;
        boost::recursive_mutex m_capture_lock;
    
        // Register this node type
        DECLARE_NFR;
};

#endif // ndef __FILE_INPUT_NODE_H__

