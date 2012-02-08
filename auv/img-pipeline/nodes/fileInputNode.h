/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

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
              m_is_directory(false), m_iter(), m_seq(0){
        }

        void init(){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID("image");
            
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

    protected:
        void doWork(in_image_map_t&, out_map_t& r){
            debug(4) << "fileInputNode::doWork";
        
            std::string fname = param<std::string>("filename");
            cv::Mat image;

            if(!m_is_directory){
                lock_t cl(m_capture_lock);
                if(!m_capture.isOpened()){
                    image = readImage(fname);
                    clearAllowQueue();
                }else{
                    m_capture >> image;
                    if(image.empty()){
                        debug() << "video stream seems to have finished";
                        closeVideo();
                    }
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
                    if(!boost::filesystem::is_directory(m_iter->status())) {
                        image = readImage(m_iter->string(), false);
                        if (!image.empty())
                        {
                            m_iter++;
                            break;
                        }
                    }
                }
                if(image.empty())
                    warning() << "no images in directory" << fname;
                // NB: allowQueue not cleared
            }
            # warning implement per-fileinputnode instance_num soon! this could lead to really subtle bugs...
            r.internalValue("image") = boost::make_shared<Image>(image, now(), mkUID(SensorUIDBase::File/*+ m_instance_num*/, ++m_seq));
        }

        cv::Mat readImage(std::string const& fname, bool warn=true) const{
            cv::Mat img;

            try{
                img = cv::imread(fname.c_str());
            }catch(cv::Exception& e){
                error() << "FileInputNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }

            if(!img.empty()){
                debug(4) << "fileInputNode::readImage:" << fname << "-> (" << img.cols << "," << img.rows << ")";
            }else if(warn){
                warning() << "fileInputNode::readImage:" << fname << "-> (no image)";
            }

            return img;
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

        uint64_t m_seq;
    
        // Register this node type
        DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __FILE_INPUT_NODE_H__

