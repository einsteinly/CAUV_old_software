#include "cameraInputNode.h"


CameraInputObserver::CameraInputObserver(CameraInputNode& n)
    : m_node(n){
}

void CameraInputObserver::onReceiveImage(CameraID cam_id, const cv::Mat& img){
    m_node.setLatestFrame(boost::make_shared<Image>(img, Image::src_camera));
    m_node.checkAddSched();
}

