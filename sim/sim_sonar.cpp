#include <opencv/cv.h>
#include <utility/ratelimit.h>
#include <generated/types/ImageMessage.h>
#include <common/msg_classes/image.h>
#include <boost/make_shared.hpp>
#include <debug/cauv_debug.h>

#include "sim_sonar.h"

namespace cauv {

SimSonar::SimSonar (osg::Node *track_node,
                    osg::Vec3d translation,
                    osg::Vec3d axis1, float angle1,
                    osg::Vec3d axis2, float angle2,
                    osg::Vec3d axis3, float angle3,
                    unsigned int width_, unsigned int height_,
                    CauvNode *sim_node_,
                    unsigned int max_rate) :
           sim_node(sim_node_),
           width(width_), height(height_), depth(300),
           output_limit(1, max_rate),
           viewer(new osgViewer::Viewer()),
           image(new osg::Image()),
           camera(viewer->getCamera()),
           fixed_manip(new cauv::FixedNodeTrackerManipulator)
{
    viewer->setUpViewInWindow(0,0,width,height);
    image->allocateImage(width, height, 1, GL_DEPTH_COMPONENT, GL_FLOAT);
    image->setOrigin(osg::Image::BOTTOM_LEFT);
    camera->attach(osg::Camera::DEPTH_BUFFER, image);
    fixed_manip->setTrackNode(track_node);
    fixed_manip->setRotation(axis1, angle1, axis2, angle2, axis3, angle3);
    fixed_manip->setTranslation(translation);
    viewer->setCameraManipulator(fixed_manip.get());
    viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
}

void SimSonar::setup(osg::Node *root) {
    viewer->setSceneData(root);
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    double fovy,aspectratio,near,far;
    camera->getProjectionMatrixAsPerspective(fovy,aspectratio,near,far);
    camera->setProjectionMatrixAsPerspective(fovy,aspectratio,2,20); 
    viewer->realize();
}

void SimSonar::tick(double timestamp) {
    viewer->frame(timestamp);
    viewer->advance();
    if(image->valid() && output_limit.click()) {
        image->flipVertical();
        cv::Mat data = cv::Mat(height, width, CV_32F, image->data(), 0);
        unsigned char init = 0;
        cv::Mat sonarimage = cv::Mat(depth, width, CV_8UC1, init);
        const float near = 4;
        const float far = 40;
        for (unsigned int x = 0; x < width; x++) {
            for (unsigned int y = 0; y < height; y++) {
                float val = data.at<float>(y,x);
                val = 2 * near / (far + near - val * (far - near));
                sonarimage.at<unsigned char>(val * depth, x) += 60;
            }
        }
        boost::shared_ptr<ImageMessage> msg = boost::make_shared<ImageMessage>(CameraID::GemSonar, cauv::Image(sonarimage), cauv::now());
        sim_node->send(msg, UNRELIABLE_MSG);
    }
}

}
