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

#include "simulator.h"

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#include <utility/foreach.h>

#include <model/auv_model.h>
#include <model/auv_controller.h>

#include <osg/Node>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgDB/WriteFile>

#include "validators.h"

#include "sensors/camera.h"

#include "visuals/pipscreen.h"

#include "auvs/redherring.h"

using namespace cauv;
using namespace cauv::sim;


Simulator::Simulator() : CauvNode("CauvSim"),
m_auv(boost::make_shared<AUV>()),
m_auv_controller(boost::make_shared<AUVController>(m_auv)),
m_simulated_auv(boost::make_shared<RedHerring>(this, m_auv)),
m_root(new osg::Group()),
m_viewer(new osgViewer::Viewer())
{
    joinGroup("control");
    joinGroup("telemetry");

    // graphics context for off screen rendering
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = 300;
    traits->height = 200;
    traits->red = 8;
    traits->green = 8;
    traits->blue = 8;
    traits->alpha = 8;
    traits->windowDecoration = false;
    traits->pbuffer = true;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;

    osg::ref_ptr<osg::GraphicsContext> pbuffer = osg::GraphicsContext::createGraphicsContext(traits.get());


    if(!pbuffer) {
        error() << __FILE__ << " - " << __FUNCTION__ << " - Failed to create pbuffer.";
        exit(1);
    }

    m_viewer->setSceneData(m_root);


    //
    // sort out the AUVs cameras
    foreach(boost::shared_ptr<sim::Camera> simulatedCam, m_simulated_auv->getCameras()){
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        m_viewer->addSlave(camera.get(), osg::Matrixd(), osg::Matrixd());
        camera->setGraphicsContext(pbuffer.get());
        camera->setViewport(new osg::Viewport(0,0,300,200));
        camera->attach(osg::Camera::COLOR_BUFFER, simulatedCam.get());
        camera->setViewMatrixAsLookAt(osg::Vec3f(0,-20,2), osg::Vec3f(0, 0, 0), osg::Vec3f(0,0,1));
        info() <<"Added simulated camera";
    }
}


osg::ref_ptr<WorldModel> Simulator::getWorldModel(){
    return m_world_model;
}


void Simulator::onRun()
{
    CauvNode::onRun();

    m_viewer->setUpViewInWindow( 0,0,1024,768, 0 );

    m_viewer->realize();
    while(!m_viewer->done()){
        double simTime = m_viewer->getFrameStamp()->getSimulationTime();
        m_viewer->frame(simTime);
        m_viewer->advance();
        info() << "scene redrawn at " << simTime;

        m_simulated_auv->propagateTicks(simTime);
        info() << "Simulation tick completed";
    }
}


void Simulator::addOptions(boost::program_options::options_description& desc,
                           boost::program_options::positional_options_description& pos)
{
    namespace po = boost::program_options;

    desc.add_options()
            ("viewer", "Launch a view into the simulation")
            ("windDirection", po::value<osg::Vec2f>(), "Wind direction <X> <Y>.")
            ("windSpeed", po::value<float>()->default_value(12.f), "Wind speed.")
            ("depth", po::value<float>()->default_value(10000.f), "Depth.")
            ("oceanSurfaceHeight", po::value<float>()->default_value(0), "The height of the ocean surface.")
            ("isChoppy", po::value<bool>()->default_value(true), "Are the waves choppy.")
            ("choppyFactor", po::value<float>()->default_value(2.5f), "How choppy the waves are.")
            ("crestFoamHeight", po::value<float>()->default_value(2.2f), "How high the waves need to be before foam forms on the crest.")
            ("reflectionDamping", po::value<float>()->default_value(0.35f), "Reflection Damping.")
            ("waveScale", po::value<float>()->default_value(1e-8), "Wave Scale.")
            ("sunPosition", po::value<osg::Vec3f>(), "Sun position.")
            ("sunDiffuse", po::value<osg::Vec3f>(), "Sun diffuse colour.")
            ;
    CauvNode::addOptions(desc, pos);
}



void Simulator::launchViewer(){
    info() << "Viewer opened";

    // TODO: can't get multiple cameras on the same window at the moment
    //auvView->getCamera()->setGraphicsContext(userView->getCamera()->getGraphicsContext());
    //auvView->getCamera()->setViewport(30, 30, 200, 200);
    // so show it in it's own window, not really ideal, would be better as picture in picture

    osgViewer::ViewerBase::Views views;
    m_viewer->getViews(views, false);

    foreach (osgViewer::View * i, views){
        i->setUpViewInWindow( 0,0,1024,768, 0 );
        //i->setUpViewOnSingleScreen(0);
        //m_world_model->getOceanSceneModel()->getOceanScene()->setScreenDims(osg::Vec2s(1920, 1280));
    }

    m_viewer->run();

    info() << "Viewer closed";
}


int Simulator::useOptionsMap(boost::program_options::variables_map& vm, boost::program_options::options_description& desc)
{

    osg::Vec3f sunPosition(520.f, 1900.f, 550.f );
    if(vm.count("sunPosition"))
        sunPosition = vm["sunPosition"].as<osg::Vec3f>();

    osg::Vec3f sunDiffuse(251.f/255.f, 251.f/255.f, 161.f/255.f );
    if(vm.count("sunDiffuse"))
        sunDiffuse = vm["sunDiffuse"].as<osg::Vec3f>();

    osg::Vec2f windDirection(1.0f, 1.0f);
    if(vm.count("windDirection"))
        windDirection = vm["windDirection"].as<osg::Vec2f>();


    // these options all have default values so we don't need
    // extra check around them
    float windSpeed = vm["windSpeed"].as<float>();
    bool choppy = vm["isChoppy"].as<bool>();
    float choppyFactor = vm["choppyFactor"].as<float>();
    float waveScale = vm["waveScale"].as<float>();
    float depth = vm["depth"].as<float>();
    float crestFoamHeight = vm["crestFoamHeight"].as<float>();
    float reflectionDamping = vm["reflectionDamping"].as<float>();
    float oceanHeight = vm["oceanSurfaceHeight"].as<float>();


    // we've got enough information to build the world model now
    m_world_model = new WorldModel(windDirection, windSpeed, depth, reflectionDamping, waveScale, choppy, choppyFactor, crestFoamHeight);
    m_world_model->setSunPosition(sunPosition);
    m_world_model->setSunDiffuse(sunDiffuse);
    m_world_model->setOceanSurfaceHeight(oceanHeight);

    m_root->addChild(m_world_model);

    osgDB::Registry::instance()->getDataFilePathList().push_back("/home/andy/dev/libs/openscenegraph/OpenSceneGraph-Data");
    const std::string filename = "cow.osg";
    osg::ref_ptr<osg::Node> cow = osgDB::readNodeFile(filename);

    m_world_model->addChild(cow);

    return CauvNode::useOptionsMap(vm, desc);
}
