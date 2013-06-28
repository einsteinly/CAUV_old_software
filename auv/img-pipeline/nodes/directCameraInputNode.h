/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __DIRECT_CAMERA_INPUT_NODE_H__
#define __DIRECT_CAMERA_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "asynchronousNode.h"

#include <generated/types/SensorUIDBase.h>

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

#define MAX_DEVICES 5

namespace cauv{
namespace imgproc{

class DirectCameraInputNode: public AsynchronousNode{
        typedef boost::lock_guard<boost::recursive_mutex> lock_t;

    public:
        DirectCameraInputNode(ConstructArgs const& args)
            : AsynchronousNode(args),
              m_current_device(-1),
              m_seq(0){
        }

        void init(){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID("image_out");
            
            // one parameter:
            registerParamID<int>("device id", 0);
            
            openCapture();
        }

        virtual ~DirectCameraInputNode(){
            stop();
            m_capture = cv::VideoCapture();
            if(m_current_device != -1)
                m_capture_lock[m_current_device].unlock();
        }

        virtual void paramChanged(input_id const& p){
            if(p == "device id")
            {
                openCapture();
            }
        }

    protected:
        void doWork(in_image_map_t&, out_map_t& r){
            debug(4) << "DirectCameraInputNode::doWork";
            
            if(!m_capture.isOpened()){
                error() << "camera is not opened";
            }else{
                msleep(10);
                
                cv::Mat img;
                m_capture >> img;
                // use internalValue to avoid automatic UID setting on outputs                
                r.internalValue("image_out") = boost::make_shared<Image>(img.clone(), now(), mkUID(SensorUIDBase::Camera + m_current_device, ++m_seq));
            }
        }

    private:

        void openCapture(){
            #ifdef __APPLE__
            warning() << "Direct camera access is not possible on OS X: use shared (normal) cameras";
            #else
            int dev_id = param<int>("device id");
            if(dev_id >= MAX_DEVICES || dev_id < 0){
                error() << "invalid camera:" << dev_id;
            }else{
                bool l = m_capture_lock[dev_id].try_lock();
                if(!l){
                    error() << "camera already open" << dev_id;
                }else{
                    if(m_current_device != -1)
                        m_capture_lock[m_current_device].unlock();
                    m_current_device = dev_id;
                    try{
                        m_capture = cv::VideoCapture(dev_id);
                    }catch(cv::Exception& e){
                        error() << "capture exception:" << e.what();
                    }
                   
                    if(!m_capture.isOpened()){
                        error() << "could not open camera" << dev_id;
                        return;
                    } 
                    //m_capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
                    //m_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
                    m_capture.set(cv::CAP_PROP_FRAME_WIDTH, 320);
                    m_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 280);
                    setAllowQueue();
                }
            }
            #endif
        }
        
        cv::VideoCapture m_capture;
        int m_current_device;
        uint64_t m_seq;
        static boost::try_mutex m_capture_lock[MAX_DEVICES];
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __DIRECT_CAMERA_INPUT_NODE_H__

