/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include <iostream>
#include <stdint.h>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>

#include <osgDB/ReadFile>
#include <osgGA/NodeTrackerManipulator>
#include <osg/PositionAttitudeTransform>
#include <osg/FrameStamp>

#include <osgViewer/Viewer>

#include <debug/cauv_debug.h>
#include <common/cauv_node.h>
#include <common/msg_classes/north_east_depth.h>
#include <common/msg_classes/wgs84_coord.h>
#include <generated/types/SimPositionMessage.h>
#include <generated/types/GeminiControlMessage.h>
#include <generated/message_observers.h>
#include <utility/ratelimit.h>
#include <utility/uid.h>

#include "sim_camera.h"
#include "sim_sonar.h"
#include "objects/barracuda.h"
#include "objects/buoy.h"
#include "objects/water.h"
#include "objects/nodetrail.h"

using namespace cauv;

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
    unsigned int trail_length;
    std::string env_file;
    bool sim_sonar;
};

void SimNode::addOptions(po::options_description& desc,
                          po::positional_options_description& pos)
{
    CauvNode::addOptions(desc, pos);
    desc.add_options()
        ("max_rate,r", po::value<unsigned int>(&max_rate)->default_value(20), "Maximum rate to send camera image messages")
        ("trail_length,t", po::value<unsigned int>(&trail_length)->default_value(100), "length of trails to draw")
        ("env_file,f", po::value<std::string>(&env_file)->default_value(""), "Simulation environment .osgt file to use")
        ("sim_sonar,g", po::value<bool>(&sim_sonar)->default_value(false)->zero_tokens(), "Simulate sonar data")
    ;
}

void SimNode::onRun(void) {
    static const unsigned int view_mask = 0x4;

    osg::ref_ptr<osg::Group> root_group = new osg::Group();
    osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer();
    viewer->setUpViewInWindow(0,0,640,320);
    viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

#if 0
    for (int i = 0; i != 3; i++) {
        osg::ref_ptr<osg::Node> buoy = new BuoyNode(0.2);
        osg::ref_ptr<osg::PositionAttitudeTransform> buoy_pos = new osg::PositionAttitudeTransform();
        buoy_pos->addChild(buoy);
        root_group->addChild(buoy_pos);
        buoy_pos->setPosition(osg::Vec3f(i*10,0,0));
    }
#endif
  
    osg::ref_ptr<osg::Node> vehicle = new BarracudaNode();
    osg::ref_ptr<osg::PositionAttitudeTransform> vehicle_pos = new osg::PositionAttitudeTransform();
    vehicle->setNodeMask(view_mask);
    vehicle_pos->addChild(vehicle);
    root_group->addChild(vehicle_pos);

    osg::ref_ptr<osg::Node> surface = new WaterNode();
    surface->setNodeMask(view_mask | SimCamera::node_mask);
    root_group->addChild(surface);

    osg::ref_ptr<NodeTrail> front_trail;
    osg::ref_ptr<NodeTrail> back_trail;
    if (trail_length) {
        front_trail = new NodeTrail(vehicle, osg::Vec3(0,0.3,0),
                                    osg::Vec3(1,0,0), trail_length);
        back_trail = new NodeTrail(vehicle, osg::Vec3(0,-0.3,0),
                                    osg::Vec3(0,0,1), trail_length);
        front_trail->setNodeMask(view_mask);
        root_group->addChild(front_trail);
        back_trail->setNodeMask(view_mask);
        root_group->addChild(back_trail);
    }
    
    osg::ref_ptr<osg::Node> environment_node = osgDB::readNodeFile(env_file);
    environment_node->setNodeMask(view_mask | SimCamera::node_mask | SimSonar::node_mask);
    root_group->addChild(environment_node);

    osg::ref_ptr<osgGA::NodeTrackerManipulator> trackballmanip = new osgGA::NodeTrackerManipulator();
    trackballmanip->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER);
    trackballmanip->setTrackNode(vehicle);
    viewer->setCameraManipulator(trackballmanip.get()); 

    SimCamera forward(vehicle.get(),
              osg::Vec3d(0,0.8,0),
              osg::Vec3d(1,0,0), M_PI/2,
              osg::Vec3d(0,0,0), 0,
              osg::Vec3d(0,0,0), 0,
              "Forward Camera", 512,
              this, CameraID::Forward,
              max_rate);

    SimCamera down(vehicle.get(),
              osg::Vec3d(0,0,-0.2),
              osg::Vec3d(0,0,0), M_PI/2,
              osg::Vec3d(0,0,0), M_PI/2,
              osg::Vec3d(0,0,0), 0,
              "Down Camera", 512,
              this, CameraID::Down,
              max_rate);

    SimCamera up(vehicle.get(),
              osg::Vec3d(0,0,0.2),
              osg::Vec3d(1,0,0), -M_PI,
              osg::Vec3d(0,0,0), 0,
              osg::Vec3d(0,0,0), 0,
              "Up camera", 512,
              this, CameraID::Up,
              max_rate);

    boost::shared_ptr<SimSonar> sonar;
    if(sim_sonar) {
        sonar = boost::make_shared<SimSonar>(
                  vehicle.get(),
                  osg::Vec3d(0,0.8,0.2),
                  osg::Vec3d(1,0,0), M_PI/2,
                  osg::Vec3d(0,0,0), 0,
                  osg::Vec3d(0,0,0), 0,
                  300,
                  this,
                  max_rate);

        subMessage(GeminiControlMessage());
        addMessageObserver(sonar);
    }

    viewer->setSceneData(root_group);
    viewer->realize();
    forward.setup(root_group);
    down.setup(root_group);
    up.setup(root_group);
    if (sonar) {
        sonar->setup(root_group);
    }

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
        down.tick(simTime);
        up.tick(simTime);
        if (back_trail) {
            back_trail->tick();
        }
        if (front_trail) {
            front_trail->tick();
        }
        if (sonar) {
            sonar->tick(simTime);
        }
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
