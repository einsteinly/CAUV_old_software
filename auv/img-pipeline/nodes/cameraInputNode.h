#ifndef __CAMERA_INPUT_NODE_H__
#define __CAMERA_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "asynchronousNode.h"

class CameraInputNode: public AsynchronousNode{
        friend class CameraInputObserver;

        typedef boost::lock_guard<boost::recursive_mutex> lock_t;

    public:
        CameraInputNode(Scheduler& s)
            : AsynchronousNode(s), m_capture(){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID("image_out");
            
            // one parameter:
            registerParamID<int>("device id", 0);
            
            openCapture();
        }

        template<typename T>
        void paramChanged(param_id const& p, T const& new_value){
            if(p == "device id")
                openCapture();
        }

    protected:
        out_image_map_t doWork(in_image_map_t&){
            out_image_map_t r;
            
            debug() << "CameraInputNode::doWork";
            
            if(!m_capture.isOpened()){
                error() << "camera is not opened";
            }else{
                boost::shared_ptr<Image> img = boost::make_shared<Image>();
                m_capture >> img->cvMat();
                img->source(Image::src_camera);
                r["image_out"] = img;
            }

            return r;
        }
        
        virtual bool allowQueueExec() throw(){
            return m_capture.isOpened();
        }

    private:

        void openCapture(){
            int dev_id = param<int>("device id");
            lock_t l(m_capture_lock);
            m_capture = cv::VideoCapture(dev_id);
            
            if(!m_capture.isOpened()){
                error() << "could not open camera" << dev_id;
                return;
            } 
            //m_capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
            //m_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
            m_capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
            m_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 280);
        }
        
        boost::recursive_mutex m_capture_lock;
        cv::VideoCapture m_capture;
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __CAMERA_INPUT_NODE_H__

