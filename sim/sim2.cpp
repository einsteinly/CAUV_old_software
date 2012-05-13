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

#include "FixedNodeTrackerManipulator.h"

#include <sstream>

class SimCamera {
    public:
    SimCamera (osg::Node *node,
               osg::Vec3d translation,
               osg::Vec3d axis1, float angle1,
               osg::Vec3d axis2, float angle2,
               osg::Vec3d axis3, float angle3,
               unsigned int width, unsigned int height) :
       viewer(new osgViewer::Viewer()),
       image(new osg::Image()),
       camera(viewer->getCamera()),
       fixed_manip(new cauv::FixedNodeTrackerManipulator)
    {
        viewer->setUpViewInWindow(0,0,width,height);
        image->allocateImage(width, height, 24, GL_RGB, GL_UNSIGNED_BYTE);
        //camera->attach(osg::Camera::COLOR_BUFFER0, image);
        fixed_manip->setTrackNode(node);
        fixed_manip->setRotation(axis1, angle1, axis2, angle2, axis3, angle3);
        fixed_manip->setTranslation(translation);
        viewer->setCameraManipulator(fixed_manip.get());
    }

    osg::ref_ptr<osgViewer::Viewer> viewer;
    osg::ref_ptr<osg::Image> image;
    private:
    osg::ref_ptr<osg::Camera> camera;
    osg::ref_ptr<cauv::FixedNodeTrackerManipulator> fixed_manip;
};

using namespace cauv;

class SimObserver : public MessageObserver {
    public:
    SimObserver(void) : datum(WGS84Coord::fromDatum("river cam")) {};
    virtual void onSimPositionMessage(SimPositionMessage_ptr m) {
        WGS84Coord loc = m->location();
        NorthEastDepth pos = loc - datum;
        debug() << *m;
        debug() << pos.north() << pos.east() << pos.depth();
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

class SimNode : public CauvNode {
    public:
    SimNode() : CauvNode("sim") {};
    virtual void onRun (void);
};

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

    //forward
    SimCamera c1(vehicle.get(),
              osg::Vec3d(0,0.6,0),
              osg::Vec3d(1,0,0), M_PI/2,
              osg::Vec3d(0,0,0), M_PI/2,
              osg::Vec3d(0,0,0), 0,
              512, 512);

    viewer->setSceneData(root_group);
    viewer->realize();
    c1.viewer->setSceneData(root_group);
    c1.viewer->realize();

    subMessage(SimPositionMessage());
    boost::shared_ptr<SimObserver> obs = boost::make_shared<SimObserver>();
    addMessageObserver(obs);

    while(!viewer->done()){
        double simTime = viewer->getFrameStamp()->getSimulationTime();
        vehicle_pos->setPosition(obs->position);
        vehicle_pos->setAttitude(obs->attitude);
        viewer->frame(simTime);
        viewer->advance();
        c1.viewer->frame(simTime);
        c1.viewer->advance();
        info() << "scene redrawn at " << simTime;

        //std::stringstream str;
        //str << "frames/u" << simTime << ".png";
        //osgDB::writeImageFile(*imag,str.str());//renders images looking up
         
        //std::stringstream str2;
        //str2 << "frames/d" << simTime << ".png";
        //osgDB::writeImageFile(*imag2,str2.str());//renders images lookig down
        
        //std::stringstream str3;
        //str3 << "frames/f" <<simTime << ".png";
        //osgDB::writeImageFile(*imag3,str3.str());//renders images looking forward
        
    }
}

int main(void)
{
    SimNode node;
    node.run(false);
    return 0;
}
