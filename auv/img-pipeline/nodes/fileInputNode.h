/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
              m_is_directory(false), m_iter(), m_seq(0),
              m_instance_num(nextInstanceNum()){
        }

        void init(){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID("image");
            registerOutputID("fps", 30.0f);
            
            // one parameter: the filename
            registerParamID<std::string>("filename", "default.jpg");
            registerParamID<bool>("loop", true);
        }

        virtual void paramChanged(input_id const& p){
            debug(4) << "FileInputNode::paramChanged";
            if(p == input_id("filename")){
                lock_t l(m_dir_mutex);
                std::string fname = param<std::string>("filename");
                closeVideo();
                if(boost::filesystem::is_directory(fname)){
                    m_is_directory = true;
                    m_fps = 30;
                    m_iter = boost::filesystem::directory_iterator();
                }else{
                    m_is_directory = false;
                    openVideo(fname);
                }
                setAllowQueue();
            }else{
                warning() << "unknown parameter" << p << "set";
            }
        }

    protected:
        void doWork(in_image_map_t&, out_map_t& r){
            debug(4) << "fileInputNode::doWork";
        
            std::string fname = param<std::string>("filename");
            bool loop = param<bool>("loop");
            cv::Mat image;

            {
                lock_t cl(m_capture_lock);

                if(m_capture.isOpened()) {
                    m_capture >> image;
                }
            }
                
            if (image.empty()) {
                if (!m_is_directory)
                {
                    // Reopen file if looping current video
                    if (loop) {
                        debug(2) << "Looping video" << fname;
                        if (openVideo(fname))
                            m_capture >> image;
                    }
                }
                else {
                    lock_t l(m_dir_mutex);
                    const boost::filesystem::directory_iterator end;
                   
                    int loops = 0;
                    // Try remaining paths
                    for(; m_iter != end || loop; m_iter++) {
                        // Loop if needed
                        if (loop && m_iter == end) {
                            debug(2) << "Looping directory" << fname;
                            m_iter = boost::filesystem::directory_iterator(fname);
                            loops++;
                        }

                        debug(4)  << "considering path:" << m_iter->path().native();
                        if(!boost::filesystem::is_directory(m_iter->status())) {
                            if (openVideo(m_iter->path().native())) {
                                debug(2) << "Playing" << m_iter->path().native();
                                m_capture >> image;
                                if (!image.empty())
                                {
                                    m_iter++;
                                    break;
                                }
                            }
                        }
                    }
                }
            
                
                // Worst case, image still empty
                if(image.empty()) {
                    if (m_frame_num > 0)
                        debug() << "Video stream finished after" << m_frame_num << "frames";
                    else
                        error() << "Video stream" << fname << "failed to open";
                    closeVideo();
                    clearAllowQueue();
                }
                else {
                    m_frame_num++;
                }
            }
            
            if(!image.empty())
                r.internalValue("image") = boost::make_shared<Image>(image.clone(), now(), mkUID(SensorUIDBase::File + m_instance_num, ++m_seq));
            r["fps"] = (float)m_fps;
        }

        bool openVideo(const std::string& fname){
            lock_t l(m_capture_lock);
            if(m_capture.open(fname)) {
                m_frame_num = 0;
                m_fps = m_capture.get(CV_CAP_PROP_FPS);
                return true;
            } else
                m_capture.release();
            return false;
        }

        void closeVideo(){
            m_capture.release();
        }

    private:
        static uint32_t nextInstanceNum(){
            // node creation actually always happens on the same thread, so we
            // don't need a lock here
            static uint32_t instance_num = 0;
            instance_num++;
            // if there are more than 32 fileinput nodes, this will be a
            // problem:
            return instance_num % 0x20;
        }

        boost::recursive_mutex m_dir_mutex;

        bool m_is_directory;
        boost::filesystem::directory_iterator m_iter;

        cv::VideoCapture m_capture;
        boost::recursive_mutex m_capture_lock;
        int m_frame_num;
        double m_fps;

        uint64_t m_seq;
        uint32_t m_instance_num;
    
        // Register this node type
        DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __FILE_INPUT_NODE_H__

