#ifndef CAUV_SIMCAMERA_H
#define CAUV_SIMCAMERA_H

#include <generated/types/CameraID.h>
#include <osg/Node>
#include <osg/Image>
#include <osgViewer/Viewer>
#include <common/cauv_node.h>
#include "FixedNodeTrackerManipulator.h"
#include "attenuator.h"

namespace cauv {

class SimCamera {
    public:
    SimCamera (osg::Node *track_node,
               osg::Vec3d translation,
               osg::Vec3d axis1, float angle1,
               osg::Vec3d axis2, float angle2,
               osg::Vec3d axis3, float angle3,
               std::string window_title,
               unsigned int width,
               cauv::CauvNode *sim_node, cauv::CameraID::e id,
               unsigned int max_rate);
    void tick(double timestamp);
    void setup(osg::Node *root);
    static const unsigned int node_mask = 0x1;
    private:
    cauv::CauvNode *sim_node;
    cauv::CameraID::e cam_id;
    std::string window_title;
    unsigned int width, height;
    RateLimiter output_limit;
    osg::ref_ptr<osgViewer::Viewer> viewer;
    osg::ref_ptr<osg::Image> image;
    osg::ref_ptr<osg::Camera> camera;
    osg::ref_ptr<cauv::FixedNodeTrackerManipulator> fixed_manip;
    osg::ref_ptr<Attenuator> attenuator;
};

}

#endif
