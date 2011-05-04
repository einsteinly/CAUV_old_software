#include "simulator.h"

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#include <model/auv_model.h>
#include <model/auv_controller.h>

#include <osg/Node>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>

#include "validators.h"

#include "sensors/camera.h"

#include "visuals/pipscreen.h"

#include "auvs/redherring.h"

using namespace cauv;
using namespace cauv::sim;

Simulator::Simulator() : CauvNode("CauvSim"),
m_auv(boost::make_shared<AUV>()),
m_auv_controller(boost::make_shared<AUVController>(m_auv)),
m_simulated_auv(boost::make_shared<RedHerring>(m_auv)),
m_root(new osg::Group()),
m_viewer(new osgViewer::CompositeViewer())
{
    joinGroup("control");
    joinGroup("telemetry");

    //
    // the users camera
    osgViewer::Viewer * userView = new osgViewer::Viewer();
    userView->setSceneData(m_root.get());
    userView->getCamera()->setViewMatrixAsLookAt(osg::Vec3f(0,-20,0), osg::Vec3f(0, 0, 0), osg::Vec3f(0,0,1));
    userView->getCamera()->setViewport(0, 0, 1024, 768);

    // mouse handling
    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition( osg::Vec3f(0.f,-20.f,0.f), osg::Vec3f(0.f,0.f,0.f), osg::Vec3f(0,0,1) );
    userView->setCameraManipulator( tb );

    m_viewer->addView(userView);

    //
    // the auv cameras
    foreach(osg::ref_ptr<sim::Camera > camera, m_simulated_auv->getCameras()){

        camera->setSceneData(m_root.get());

        camera->getCamera()->setViewMatrixAsLookAt(osg::Vec3f(0,0,0), osg::Vec3f(0, 20, 0), osg::Vec3f(0,0,1));

        // mouse handling
        osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
        tb->setHomePosition( osg::Vec3f(0.f,-20.f,0.f), osg::Vec3f(0.f,0.f,0.f), osg::Vec3f(0,0,1) );
        camera->setCameraManipulator( tb );

        // mouse handling
        m_viewer->addView(camera);
    }
}


osg::ref_ptr<WorldModel> Simulator::getWorldModel(){
    return m_world_model;
}


void Simulator::onRun()
{
    CauvNode::onRun();
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
        //i->setUpViewInWindow( 0,0,1920,1280, 0 );
        i->setUpViewOnSingleScreen(0);
        m_world_model->getOceanSceneModel()->getOceanScene()->setScreenDims(osg::Vec2s(1920, 1280));
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


    // see if the user wants a window into the world...
    if(vm.count("viewer")) {
        boost::thread(boost::bind(&Simulator::launchViewer, this));
    } else {
        m_viewer->realize();
        while(!m_viewer->done()){
            m_viewer->advance();
            m_viewer->eventTraversal();
            m_viewer->updateTraversal();
            m_viewer->renderingTraversals();
        }
    }

    return CauvNode::useOptionsMap(vm, desc);
}
