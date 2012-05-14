#include <opencv/cv.h>
#include <utility/ratelimit.h>
#include <generated/types/ImageMessage.h>
#include <common/msg_classes/image.h>
#include <boost/make_shared.hpp>

#include "sim_camera.h"

namespace cauv {

SimCamera::SimCamera (osg::Node *track_node,
                      osg::Vec3d translation,
                      osg::Vec3d axis1, float angle1,
                      osg::Vec3d axis2, float angle2,
                      osg::Vec3d axis3, float angle3,
                      unsigned int width_, unsigned int height_,
                      CauvNode *sim_node_,
                      CameraID::e id,
                      unsigned int max_rate) :
           sim_node(sim_node_),
           cam_id(id),
           width(width_), height(height_),
           output_limit(1, max_rate),
           viewer(new osgViewer::Viewer()),
           image(new osg::Image()),
           camera(viewer->getCamera()),
           fixed_manip(new cauv::FixedNodeTrackerManipulator)
{
    viewer->setUpViewInWindow(0,0,width,height);
    image->allocateImage(width, height, 1, GL_BGR, GL_UNSIGNED_BYTE);
    image->setOrigin(osg::Image::BOTTOM_LEFT);
    camera->attach(osg::Camera::COLOR_BUFFER0, image);
    fixed_manip->setTrackNode(track_node);
    fixed_manip->setRotation(axis1, angle1, axis2, angle2, axis3, angle3);
    fixed_manip->setTranslation(translation);
    viewer->setCameraManipulator(fixed_manip.get());
}

void SimCamera::setup(osg::Node *root) {
    viewer->setSceneData(root);
    viewer->realize();
}

void SimCamera::tick(double timestamp) {
    viewer->frame(timestamp);
    viewer->advance();
    if(image->valid() && output_limit.click()) {
        image->flipVertical();
        cv::Mat data = cv::Mat(width, height, CV_8UC3, image->data(), 0);
        boost::shared_ptr<ImageMessage> msg = boost::make_shared<ImageMessage>(cam_id, cauv::Image(data), cauv::now());
        sim_node->send(msg, UNRELIABLE_MSG);
    }
}

}
