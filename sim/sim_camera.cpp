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
                      std::string window_title,
                      unsigned int width_,
                      CauvNode *sim_node_,
                      CameraID::e id,
                      unsigned int max_rate) :
           sim_node(sim_node_),
           cam_id(id),
           window_title(window_title),
           width(width_),
           height(width_),
           output_limit(1, max_rate),
           viewer(new osgViewer::Viewer()),
           image(new osg::Image()),
           camera(viewer->getCamera()),
           fixed_manip(new cauv::FixedNodeTrackerManipulator),
           attenuator(new Attenuator(0,0))
{
    viewer->setUpViewInWindow(0,0,width,height);
    viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

    image->allocateImage(width, height, 1, GL_BGR, GL_UNSIGNED_BYTE);
    image->setOrigin(osg::Image::BOTTOM_LEFT);

    fixed_manip->setTrackNode(track_node);
    fixed_manip->setRotation(axis1, angle1, axis2, angle2, axis3, angle3);
    fixed_manip->setTranslation(translation);
    viewer->setCameraManipulator(fixed_manip.get());

	viewer->addEventHandler(new AttenuatorEventHandler(attenuator));
    viewer->setSceneData(attenuator->getRoot());
    
    camera = viewer->getCamera();
    camera->attach(osg::Camera::COLOR_BUFFER0, image);
    camera->setInheritanceMask(
          osg::CullSettings::ALL_VARIABLES &
          ~osg::CullSettings::CULL_MASK);
    camera->setCullMask(node_mask);

}

void SimCamera::setup(osg::Node *root) {
    attenuator->setScene(root);
    camera->setProjectionMatrixAsPerspective(60,1,0.5,100); 
    viewer->realize();

    typedef osgViewer::Viewer::Windows Windows;
    Windows windows;
    // Windows is just an std::vector of osgViewer::GraphicsWindows
    viewer->getWindows(windows);
    // this sets the title for all your windows, alternately you could just set it for windows[0] for example.
    for (Windows::iterator window = windows.begin();
         window != windows.end(); ++window)
    (*window)->setWindowName(window_title);
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
