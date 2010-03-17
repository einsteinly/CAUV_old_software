#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <boost/shared_ptr.hpp>

#include <common/cauv_node.h>
#include <camera/camera.h>

class WebcamNode : public CauvNode
{
    public:
        WebcamNode(const CameraID camera_id, const int device_id);
    
    protected:
        boost::shared_ptr<Camera> m_camera;
};

#endif//__CONTROL_H__
