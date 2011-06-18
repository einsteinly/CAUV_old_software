#ifndef __FILE_INPUT_NODE_H__
#define __FILE_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "asynchronousNode.h"


namespace cauv{
namespace imgproc{

class FileInputNode: public AsynchronousNode{
        typedef boost::unique_lock<boost::recursive_mutex> lock_t;
    public:
        FileInputNode(ConstructArgs const& args)
            : AsynchronousNode(args),
              m_is_directory(false), m_iter(){
        }

        void init(){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID<image_ptr_t>("image");
            
            // one parameter: the filename
            registerParamID<std::string>("filename", "default.jpg");
        }

        virtual void paramChanged(input_id const& p){
            debug(4) << "FileInputNode::paramChanged";
            if(p == input_id("filename")){
                lock_t l(m_dir_mutex);
                std::string fname = param<std::string>("filename");
                if(boost::filesystem::is_directory(fname)){
                    m_is_directory = true;
                    closeVideo();
                }else{
                    m_is_directory = false;
                    // if the file might be a video, try to open it
                    using boost::algorithm::iends_with;
                    if(iends_with(fname, ".mpg") || iends_with(fname, ".mpeg") ||
                       iends_with(fname, ".ogg") || iends_with(fname, ".mp4") ||
                       iends_with(fname, ".avi") || iends_with(fname, ".mov") ||
                       iends_with(fname, ".wmv") || iends_with(fname, ".3gp") ||
                       iends_with(fname, ".3g2") || iends_with(fname, ".asf") ||
                       iends_with(fname, ".vob") || iends_with(fname, ".asx") ||
                       iends_with(fname, ".aiff")){
                        openVideo(fname);
                    }
                }
                m_iter = boost::filesystem::directory_iterator();
                setAllowQueue();
            }else{
                warning() << "unknown parameter" << p << "set";
            }
        }
        
        virtual ~FileInputNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            out_map_t r;
            
            debug(4) << "fileInputNode::doWork";
        
            std::string fname = param<std::string>("filename");
            image_ptr_t image;

            if(!m_is_directory){
                lock_t cl(m_capture_lock);
                if(!m_capture.isOpened()){
                    image = readImage(fname);
                    clearAllowQueue();
                }else{
                    image = boost::make_shared<Image>();
                    m_capture >> image->cvMat();
                    if(!image->cvMat().rows || !image->cvMat().cols){
                        debug() << "video stream seems to have finished";
                        closeVideo();
                        image.reset();
                    }
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
                    debug(4)  << "considering path:" << m_iter->string();
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
                r = boost::make_shared<Image>(img);
                debug(4) << "fileInputNode::readImage:" << fname << "->" << *r;
            }else if(warn){
                warning() << "fileInputNode::readImage:" << fname << "-> (no image)";
            }

            return r;
        }

        bool openVideo(std::string const& fname){
            lock_t l(m_capture_lock);
            if(m_capture.open(fname))
                return true;
            else
                m_capture.release();
            return false;
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

} // namespace imgproc
} // namespace cauv

#endif // ndef __FILE_INPUT_NODE_H__

