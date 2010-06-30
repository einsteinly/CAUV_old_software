#ifndef __CAMERA_INPUT_NODE_H__
#define __CAMERA_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "asynchronousNode.h"

#define MAX_DEVICES 5

class CameraInputNode: public AsynchronousNode{
        typedef boost::lock_guard<boost::recursive_mutex> lock_t;

    public:
        CameraInputNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : AsynchronousNode(sched, pl, t), m_capture(){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID<image_ptr_t>("image_out");
            
            // one parameter:
            registerParamID<int>("device id", 0);
            
            openCapture();
        }

        virtual void paramChanged(param_id const& p){
            if(p == "device id")
            {
                openCapture();
            }
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            out_map_t r;
            
            debug() << "CameraInputNode::doWork";
            
            if(!m_capture.isOpened()){
                error() << "camera is not opened";
            }else{
                boost::shared_ptr<Image> img = boost::make_shared<Image>();
                m_capture >> img->cvMat();
                r["image_out"] = img;
            }

            return r;
        }

    private:

        void openCapture(){
            #ifdef __APPLE__
            warning() << "Capture doesn't work without an NSEventLoop";
            #else
            int dev_id = param<int>("device id");
            if(dev_id >= MAX_DEVICES || dev_id < 0){
                error() << "invalid camera:" << dev_id;
            }else{
                boost::try_mutex::scoped_try_lock l(m_capture_lock[dev_id]);
                if(!l){
                    error() << "camera already open" << dev_id;
                }else{
                    m_capture = cv::VideoCapture(dev_id);
                    
                    if(!m_capture.isOpened()){
                        error() << "could not open camera" << dev_id;
                        return;
                    } 
                    //m_capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
                    //m_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
                    m_capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
                    m_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 280);
                    setAllowQueue();
                }
            }
            #endif
        }
        
        cv::VideoCapture m_capture;
        static boost::try_mutex m_capture_lock[MAX_DEVICES];
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __CAMERA_INPUT_NODE_H__

