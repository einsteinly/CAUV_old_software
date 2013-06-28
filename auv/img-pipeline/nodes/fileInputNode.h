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

#if CV_MAJOR_VERSION < 2 || CV_MINOR_VERSION == 2 && CV_MAJOR_VERSION < 4 || CV_MAJOR_VERSION == 2 && CV_MAJOR_VERSION == 4 && CV_SUBMINOR_VERSION < 6
#ifndef WORKAROUND_CV_CAPPROP
#define WORKAROUND_CV_CAPPROP
#warning Workaround for missing CAP_PROP_FRAME_WIDTH/HEIGHT in OpenCV <= 2.4.5
namespace cv {
    enum { CAP_PROP_POS_MSEC       =0,
           CAP_PROP_POS_FRAMES     =1,
           CAP_PROP_POS_AVI_RATIO  =2,
           CAP_PROP_FRAME_WIDTH    =3,
           CAP_PROP_FRAME_HEIGHT   =4,
           CAP_PROP_FPS            =5,
           CAP_PROP_FOURCC         =6,
           CAP_PROP_FRAME_COUNT    =7,
           CAP_PROP_FORMAT         =8,
           CAP_PROP_MODE           =9,
           CAP_PROP_BRIGHTNESS    =10,
           CAP_PROP_CONTRAST      =11,
           CAP_PROP_SATURATION    =12,
           CAP_PROP_HUE           =13,
           CAP_PROP_GAIN          =14,
           CAP_PROP_EXPOSURE      =15,
           CAP_PROP_CONVERT_RGB   =16,
           CAP_PROP_WHITE_BALANCE_BLUE_U =17,
           CAP_PROP_RECTIFICATION =18,
           CAP_PROP_MONOCROME     =19,
           CAP_PROP_SHARPNESS     =20,
           CAP_PROP_AUTO_EXPOSURE =21, // DC1394: exposure control done by camera, user can adjust refernce level using this feature
           CAP_PROP_GAMMA         =22,
           CAP_PROP_TEMPERATURE   =23,
           CAP_PROP_TRIGGER       =24,
           CAP_PROP_TRIGGER_DELAY =25,
           CAP_PROP_WHITE_BALANCE_RED_V =26,
           CAP_PROP_ZOOM          =27,
           CAP_PROP_FOCUS         =28,
           CAP_PROP_GUID          =29,
           CAP_PROP_ISO_SPEED     =30,
           CAP_PROP_BACKLIGHT     =32,
           CAP_PROP_PAN           =33,
           CAP_PROP_TILT          =34,
           CAP_PROP_ROLL          =35,
           CAP_PROP_IRIS          =36,
           CAP_PROP_SETTINGS      =37
         };
}
#endif
#endif

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

        bool openVideo(std::string const& fname){
            lock_t l(m_capture_lock);
            if(m_capture.open(fname)) {
                m_frame_num = 0;
                m_fps = m_capture.get(cv::CAP_PROP_FPS);
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

