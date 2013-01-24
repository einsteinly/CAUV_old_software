/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include "sim_sonar.h"

#include <cmath>
#include <boost/make_shared.hpp>

#include <opencv2/core/core.hpp>

#include <utility/ratelimit.h>
#include <generated/types/ImageMessage.h>
#include <generated/types/SonarImageMessage.h>
#include <common/msg_classes/image.h>
#include <debug/cauv_debug.h>

namespace cauv {

SimSonar::SimSonar (osg::Node *track_node,
                    osg::Vec3d translation,
                    osg::Vec3d axis1, float angle1,
                    osg::Vec3d axis2, float angle2,
                    osg::Vec3d axis3, float angle3,
                    unsigned int width_,
                    CauvNode *sim_node_,
                    unsigned int max_rate) :
           sim_node(sim_node_),
           width(width_), height(width_), resolution(300),
           range(20), fovx(120), fovy(10),
           output_limit(1, max_rate),
           viewer(new osgViewer::Viewer()),
           image(new osg::Image()),
           camera(viewer->getCamera()),
           fixed_manip(new cauv::FixedNodeTrackerManipulator)
{
    height = width*std::tan(fovy*M_PI/180)/std::tan(fovx/2*M_PI/180);
    viewer->setUpViewInWindow(0,0,width,height);
    image->allocateImage(width, height, 1, GL_DEPTH_COMPONENT, GL_FLOAT);
    image->setOrigin(osg::Image::BOTTOM_LEFT);
    camera->attach(osg::Camera::DEPTH_BUFFER, image);
    fixed_manip->setTrackNode(track_node);
    fixed_manip->setRotation(axis1, angle1, axis2, angle2, axis3, angle3);
    fixed_manip->setTranslation(translation);
    viewer->setCameraManipulator(fixed_manip.get());
    viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    camera->setInheritanceMask(
          osg::CullSettings::ALL_VARIABLES &
          ~osg::CullSettings::CULL_MASK);
    camera->setCullMask(node_mask);
}

void SimSonar::setup(osg::Node *root) {
    viewer->setSceneData(root);
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    camera->setProjectionMatrixAsPerspective(fovy*2,std::tan(fovx/2*M_PI/180)/std::tan(fovy*M_PI/180),0.5,range * 10); 
    viewer->realize();
}

void SimSonar::tick(double timestamp) {
    viewer->frame(timestamp);
    viewer->advance();
    if(image->valid() && output_limit.click()) {
        image->flipVertical();
        std::vector<uint8_t> beams(resolution * width);
        cv::Mat data = cv::Mat(height, width, CV_32F, image->data(), 0);
        //near plane
        const float near = 0.5;
        //far plane
        const float far = range * 10;
        std::vector<int32_t> bearings(width);
        //     /|
        //    / |
        //   /  |
        //  /   | near_width
        // /_a__|_____ a=radians(fovx/2)
        //  near
        float near_width = near * std::tan(fovx/2 / 360.0 * 2 * M_PI);
        for (unsigned int x = 0; x < width; x++) {
            float bearing = std::atan2((float(x)/width - 0.5f) * 2 * near_width, near);
            bearings[x] = bearing / (2 * M_PI) * 6400 * 0x10000;
            float scale = 1/std::cos(bearing);
            for (unsigned int y = height/2; y < height; y++) {
                float val = data.at<float>(y,x);
                val = 2 * near * far / (far + near - val * (far - near));
                if (val > 0.95 * far) {
                    continue;
                }
                val *= scale;
                unsigned int pos = val / far * resolution * 10;
                if (pos >= resolution) {
                    continue;
                }
                for (int xx = -2; xx <= 2; xx++) {
                    for (int yy = -2; yy <= 2; yy++) {
                        if (xx + (int)x < 0 || xx + x > width - 1 ||
                            yy + (int)pos < 0 || yy + pos > resolution) {
                            continue;
                        }
                        uint8_t *cur_val = &beams[x + xx + (pos + yy) * (width - 1)];
                        if (*cur_val < 200) {
                            *cur_val += 2;
                        }
                    }
                }
            }
        }
        boost::shared_ptr<SonarImageMessage> msg =
            boost::make_shared<SonarImageMessage>(
                SonarID::Gemini, PolarImage(
                    beams,
                    ImageEncodingType::RAW_uint8_1,
                    bearings,
                    0,
                    range,
                    range / (float)resolution,
                    cauv::now()
                )
            );
        sim_node->send(msg, UNRELIABLE_MSG);
    }
}

}
