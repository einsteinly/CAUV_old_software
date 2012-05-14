/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include <iostream>
#include <sstream>
#include <stdint.h>
#include <cmath>
#include <boost/make_shared.hpp>
#include <debug/cauv_debug.h>

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgGA/NodeTrackerManipulator>
#include <osg/PositionAttitudeTransform>

#include <osgViewer/View>
#include <osgViewer/Viewer>

#include <osgDB/WriteFile>

#include <osg/FrameStamp>

#include <osg/Image>
#include <osg/ShapeDrawable>
#include <osg/BlendFunc>

#include <common/cauv_node.h>
#include <common/msg_classes/north_east_depth.h>
#include <common/msg_classes/wgs84_coord.h>
#include <generated/types/SimPositionMessage.h>
#include <generated/message_observers.h>
#include <utility/ratelimit.h>

#include "FixedNodeTrackerManipulator.h"

#include <sstream>

#include <opencv/cv.h>
#include <generated/types/ImageMessage.h>
#include <common/msg_classes/image.h>

using namespace cauv;

class SimCamera {
    public:
    SimCamera (osg::Node *track_node,
               osg::Vec3d translation,
               osg::Vec3d axis1, float angle1,
               osg::Vec3d axis2, float angle2,
               osg::Vec3d axis3, float angle3,
               unsigned int width, unsigned int height,
               CauvNode *sim_node, CameraID::e id,
               unsigned int max_rate);
    void tick(double timestamp);
    void set_up(osg::Node *root);
    private:
    CauvNode *sim_node;
    CameraID::e cam_id;
    unsigned int width, height;
    RateLimiter output_limit;
    osg::ref_ptr<osgViewer::Viewer> viewer;
    osg::ref_ptr<osg::Image> image;
    osg::ref_ptr<osg::Camera> camera;
    osg::ref_ptr<cauv::FixedNodeTrackerManipulator> fixed_manip;
};

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

void SimCamera::set_up(osg::Node *root) {
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

class SimObserver : public MessageObserver {
    public:
    SimObserver(void) : datum(WGS84Coord::fromDatum("river cam")) {};
    virtual void onSimPositionMessage(SimPositionMessage_ptr m) {
        WGS84Coord loc = m->location();
        NorthEastDepth pos = loc - datum;
        position.x() = pos.east();
        position.y() = pos.north();
        position.z() = -pos.depth();
        
        quat rot = m->quaternion();
        attitude.x() = rot.q0;
        attitude.y() = rot.q1;
        attitude.z() = rot.q2;
        attitude.w() = rot.q3;
    };
    WGS84Coord datum;
    osg::Vec3d position;
    osg::Quat attitude;
};


namespace po = boost::program_options;

class SimNode : public CauvNode {
    public:
    SimNode() : CauvNode("sim") {};
    virtual void onRun (void);
    virtual void addOptions(po::options_description& desc,
                            po::positional_options_description& pos);
    private:
    unsigned int max_rate;
    std::string resource_dir;
};

void SimNode::addOptions(po::options_description& desc,
                          po::positional_options_description& pos)
{
    CauvNode::addOptions(desc, pos);
    desc.add_options()
        ("max_rate,r", po::value<unsigned int>(&max_rate)->default_value(20), "Maximum rate to send camera image messages")
        ("resource_dir,d", po::value<std::string>(&resource_dir)->default_value(""), "OSG resource directory to use")
    ;
}

class BuoyNode : public osg::Geode {
    public:
    BuoyNode(float radius) : 
        osg::Geode(),
        sphere(new osg::Sphere(osg::Vec3f(), radius)),
        drawable(new osg::ShapeDrawable(sphere.get())) {
        drawable->setColor(osg::Vec4f(1,1,0,1));
        addDrawable(drawable);
    };

    private:
    osg::ref_ptr<osg::Sphere> sphere;
    osg::ref_ptr<osg::ShapeDrawable> drawable;
};

class RedHerringNode : public osg::Geode {
    public:
    RedHerringNode(void) :
        osg::Geode(),
        box(new osg::Box(osg::Vec3f(), 0.2, 1, 0.2)),
        front_marker(new osg::Box(osg::Vec3f(0,0.5,0),0.1, 0.1, 0.1)),
        drawable(new osg::ShapeDrawable(box.get())),
        front_drawable(new osg::ShapeDrawable(front_marker.get())) {
        drawable->setColor(osg::Vec4f(1,0,0,1));
        addDrawable(drawable);
        addDrawable(front_drawable);
    }

    private:
    osg::ref_ptr<osg::Box> box;
    osg::ref_ptr<osg::Box> front_marker;
    osg::ref_ptr<osg::ShapeDrawable> drawable;
    osg::ref_ptr<osg::ShapeDrawable> front_drawable;
};

class WaterSurfaceNode : public osg::Geode {
    public:
    WaterSurfaceNode(void) :
        osg::Geode(),
        plane(new osg::Box(osg::Vec3f(0,0,-10),80,80,20)),
        drawable(new osg::ShapeDrawable(plane.get())) {
        addDrawable(drawable);
        drawable->setColor(osg::Vec4f(1,0.1,1,0.3));
        osg::StateSet* set = getOrCreateStateSet();
        set->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        osg::ref_ptr<osg::BlendFunc> blend = new osg::BlendFunc(GL_SRC_ALPHA ,GL_ONE_MINUS_SRC_ALPHA);
        set->setAttributeAndModes(blend, osg::StateAttribute::ON);
    }
    private:
    osg::ref_ptr<osg::Box> plane;
    osg::ref_ptr<osg::ShapeDrawable> drawable;
};

void SimNode::onRun(void) {
    osg::ref_ptr<osg::Group> root_group = new osg::Group();

    osg::ref_ptr<osg::Node> buoy = new BuoyNode(0.2);
    osg::ref_ptr<osg::PositionAttitudeTransform> buoy_pos = new osg::PositionAttitudeTransform();
    buoy_pos->addChild(buoy);
    root_group->addChild(buoy_pos);
    buoy_pos->setPosition(osg::Vec3f(10,0,0));
  
    osg::ref_ptr<osg::Node> vehicle = new RedHerringNode();
    osg::ref_ptr<osg::PositionAttitudeTransform> vehicle_pos = new osg::PositionAttitudeTransform();
    vehicle_pos->addChild(vehicle);
    root_group->addChild(vehicle_pos);

    osg::ref_ptr<osg::Node> surface = new WaterSurfaceNode();
    root_group->addChild(surface);

    osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer();
    viewer->setUpViewInWindow(0,0,640,320);
    osg::ref_ptr<osgGA::NodeTrackerManipulator> trackballmanip = new osgGA::NodeTrackerManipulator();
    trackballmanip->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER);
    trackballmanip->setTrackNode(vehicle);
    viewer->setCameraManipulator(trackballmanip.get()); 

    SimCamera forward(vehicle.get(),
              osg::Vec3d(0,0.6,0),
              osg::Vec3d(1,0,0), M_PI/2,
              osg::Vec3d(0,0,0), M_PI/2,
              osg::Vec3d(0,0,0), 0,
              512, 512,
              this, CameraID::Forward,
              max_rate);

    viewer->setSceneData(root_group);
    viewer->realize();
    forward.set_up(root_group);

    subMessage(SimPositionMessage());
    boost::shared_ptr<SimObserver> obs = boost::make_shared<SimObserver>();
    addMessageObserver(obs);

    RateLimiter framerate_limit(1,40);
    while(!viewer->done()){
        double simTime = viewer->getFrameStamp()->getSimulationTime();
        vehicle_pos->setPosition(obs->position);
        vehicle_pos->setAttitude(obs->attitude);
        viewer->frame(simTime);
        viewer->advance();
        forward.tick(simTime);
        framerate_limit.click(true);
    }
}

int main(int argc, char** argv) {
    //work around fun NVIDIA bug with fork() in threads using OpenGL...
    mkUID();
    SimNode node;
    if (node.parseOptions(argc,argv)) {
        return 1;
    }
    node.run(false);
    return 0;
}
