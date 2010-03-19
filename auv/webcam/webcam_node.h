#ifndef __WEBCAM_NODE_H__
#define __WEBCAM_NODE_H__

#include <boost/shared_ptr.hpp>

#include <common/cauv_node.h>
#include <camera/camera.h>

class SpreadCameraObserver;

class WebcamNode : public CauvNode
{
    public:
        WebcamNode(const CameraID camera_id, const int device_id);

        void onRun();
    
    protected:
        boost::shared_ptr<Camera> m_camera;
        boost::shared_ptr<SpreadCameraObserver> m_cam_observer;
};

#endif//__WEBCAM_NODE_H__
