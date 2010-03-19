#ifndef __CAMERA_INPUT_NODE_H__
#define __CAMERA_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <camera/camera_observer.h>

#include "asynchronousNode.h"

class CameraInputNode;
class CameraInputObserver: public CameraObserver{
    public:
        CameraInputObserver(CameraInputNode& n);
        // override CameraObserver::onReceiveImage:
        virtual void onReceiveImage(CameraID cam_id, const cv::Mat& img);
    private:
        CameraInputNode& m_node;
};

class CameraInputNode: public AsynchronousNode{
        friend class CameraInputObserver;

        typedef boost::lock_guard<boost::recursive_mutex> lock_t;

    public:
        CameraInputNode(Scheduler& s)
            : AsynchronousNode(s), m_camera(){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID("image_out");
            
            // two parameters:
            registerParamID<int>("camera id", cam_forward);
            registerParamID<int>("device id", 0);
            
            try{
                openCamera();
            }catch(CameraException& e){
                error() << __func__ << e.what();
            }
        }

        template<typename T>
        void paramChanged(param_id const& p, T const& new_value){
            if(p == "camera id" || p == "device id")
                openCamera();
        }

    protected:
        out_image_map_t doWork(in_image_map_t&){
            out_image_map_t r;
            
            debug() << "CameraInputNode::doWork";
            
            lock_t l(m_latest_frame_lock);
            if(m_latest_frame)
                r["image_out"] = m_latest_frame;
            else
                warning() << "no latest frame, output will not be set";
            m_latest_frame.reset();

            return r;
        }
        
        virtual bool allowQueueExec() throw(){
            return !!m_latest_frame;
        }

    private:
        void setLatestFrame(boost::shared_ptr<Image> f){
            lock_t l(m_latest_frame_lock);
            m_latest_frame = f;
        }

        void openCamera(){
            int cam_id = param<int>("camera id");
            int dev_id = param<int>("device id");
            lock_t l(m_camera_lock);
            m_camera.reset();
            m_camera = boost::make_shared<Webcam>(CameraID(cam_id), dev_id);
            m_camera->addObserver(boost::make_shared<CameraInputObserver>(boost::ref(*this)));
        }
        
        boost::recursive_mutex m_camera_lock;
        boost::shared_ptr<Webcam> m_camera;

        boost::recursive_mutex m_latest_frame_lock;
        boost::shared_ptr<Image> m_latest_frame;
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __CAMERA_INPUT_NODE_H__

