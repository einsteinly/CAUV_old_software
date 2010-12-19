#ifndef __CAUV_WEBCAM_NODE_H__
#define __CAUV_WEBCAM_NODE_H__

#include <boost/shared_ptr.hpp>

#include <common/cauv_node.h>
#include <generated/messages_fwd.h>
#include <camera/camera.h>

namespace cauv{

class SpreadCameraObserver;

class WebcamNode : public CauvNode
{
    public:
        WebcamNode();

        void setCameraID(CameraID::e cameraID);
        void setDeviceID(int deviceID);

    protected:
        CameraID::e m_cameraID;
        int m_deviceID;
        boost::shared_ptr<Camera> m_camera;
        boost::shared_ptr<SpreadCameraObserver> m_cam_observer;
        
        virtual void addOptions(boost::program_options::options_description& desc, boost::program_options::positional_options_description& pos);
        virtual int useOptionsMap(boost::program_options::variables_map& vm, boost::program_options::options_description& desc);
        
        virtual void onRun();
};

} // namespace cauv

#endif // ndef __CAUV_WEBCAM_NODE_H__
