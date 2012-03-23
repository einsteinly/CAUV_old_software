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
#include <debug/cauv_debug.h>

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgGA/NodeTrackerManipulator>
#include <osg/PositionAttitudeTransform>

//#include <osgOcean/ShaderManager>
//#include <osgOcean/OceanScene>

#include <osgViewer/View>
#include <osgViewer/Viewer>

#include <osgDB/WriteFile>

#include <osg/FrameStamp>

#include <osg/Image>

#include "simulator.h"
#include "worldmodel.h"
#include "FixedNodeTrackerManipulator.h"

#include <sstream>

int main(int argc, char** argv)
{

    osgDB::Registry::instance()->getDataFilePathList().push_back("/home/alex/osg/OpenSceneGraph-Data-3.0.0");

    osgDB::Registry::instance()->getDataFilePathList().push_back("/home/alex/repos/software.hg/sim/resources");


    const std::string filename = "cow.osg";
    osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer();
    osg::ref_ptr<osgViewer::Viewer> viewer2 = new osgViewer::Viewer();
    osg::ref_ptr<osgViewer::Viewer> viewer3 = new osgViewer::Viewer();
    
    viewer->setUpViewInWindow(800,0,640,320);
    viewer2->setUpViewInWindow(0,0,640,320);
    viewer3->setUpViewInWindow(400,400,640,320);


    osg::ref_ptr<osg::Camera> camera1 = viewer->getCamera();
    osg::ref_ptr<osg::Camera> camera2 = viewer2->getCamera();
    osg::ref_ptr<osg::Camera> camera3 = viewer3->getCamera();
    
    osg::ref_ptr<osg::Group> root_group = new osg::Group();
    osg::ref_ptr<osg::Node > cow = osgDB::readNodeFile(filename);
    osg::ref_ptr<osg::Node > cow2 = osgDB::readNodeFile(filename);
    osg::ref_ptr<osg::Node > cow3 = osgDB::readNodeFile(filename);
    osg::ref_ptr<osg::Node > cow4 = osgDB::readNodeFile(filename);
  
    osg::Image* imag = new osg::Image();
    imag->allocateImage(640, 320, 24, GL_RGB, GL_UNSIGNED_BYTE);
    
    osg::Image* imag2 = new osg::Image();
    imag2->allocateImage(640, 320, 24, GL_RGB,GL_UNSIGNED_BYTE);
    
    
    osg::Image* imag3 = new osg::Image();
    imag3->allocateImage(640, 320, 24, GL_RGB,GL_UNSIGNED_BYTE);
    
    camera1->attach(osg::Camera::COLOR_BUFFER0, imag ); 
    camera2->attach(osg::Camera::COLOR_BUFFER0, imag2 ); 
    camera3->attach(osg::Camera::COLOR_BUFFER0, imag3 ); 
    
    //osg::ref_ptr<osgGA::TrackballManipulator> trackballmanip = new osgGA::TrackballManipulator();
    //viewer->setCameraManipulator(trackballmanip.get()); 
    
    root_group->addChild(cow.get());
    osg::PositionAttitudeTransform * pat = new osg::PositionAttitudeTransform();
    pat->addChild(cow2.get());
    root_group->addChild(pat);
    
    osg::PositionAttitudeTransform * pat3 = new osg::PositionAttitudeTransform();
    pat3->addChild(cow4.get());
    root_group->addChild(pat3);

    osg::PositionAttitudeTransform * pat2 = new osg::PositionAttitudeTransform();
    pat2->addChild(cow3.get());
    root_group->addChild(pat2);

    //osg::ref_ptr<osgGA::NodeTrackerManipulator> fixed_manip = new osgGA::NodeTrackerManipulator;
    osg::ref_ptr<cauv::FixedNodeTrackerManipulator> fixed_manip = new cauv::FixedNodeTrackerManipulator;
    fixed_manip->setTrackNode(cow.get());
    fixed_manip->setRotation(osg::Vec3d(0,0,1),M_PI/2, osg::Vec3d(0,0,0), 0.0, osg::Vec3d(0,1,0), M_PI);
    fixed_manip->setTranslation(0,0,-100);//multiply by -1 to move translation to see auv for debugging purposes
    viewer->setCameraManipulator(fixed_manip.get());

    osg::ref_ptr<cauv::FixedNodeTrackerManipulator> fixed_manip2 = new cauv::FixedNodeTrackerManipulator;
    fixed_manip2->setTrackNode(cow.get());
    fixed_manip2->setRotation(osg::Vec3d(0,0,1),M_PI/2, osg::Vec3d(0,0,0), 0.0, osg::Vec3d(0,0,0), 0.0);
    fixed_manip2->setTranslation(0,0,100);
    viewer2->setCameraManipulator(fixed_manip2.get());

    osg::ref_ptr<cauv::FixedNodeTrackerManipulator> fixed_manip3 = new cauv::FixedNodeTrackerManipulator;
    fixed_manip3->setTrackNode(cow.get());
    fixed_manip3->setRotation(osg::Vec3d(0,1,0),-M_PI/2, osg::Vec3d(1,0,0), M_PI/2, osg::Vec3d(0,0,0), 0.0);
    fixed_manip3->setTranslation(5,0,0);
    viewer3->setCameraManipulator(fixed_manip3.get());


    viewer->setSceneData(root_group);
    viewer2->setSceneData(root_group);
    viewer3->setSceneData(root_group);

    /*
    osg::Vec3f sunPosition(520.f, 1900.f, 550.f );
    osg::Vec3f sunDiffuse(251.f/255.f, 251.f/255.f, 161.f/255.f );
    osg::Vec2f windDirection(1.0f, 1.0f);
    cauv::sim::WorldModel *m_world_model = new cauv::sim::WorldModel(windDirection, 12, 1000, 0.35, 1e-8, true, 2.5, 2.2);
    m_world_model->setSunPosition(sunPosition);
    m_world_model->setSunDiffuse(sunDiffuse);
    m_world_model->setOceanSurfaceHeight(0);
    osg::PositionAttitudeTransform * pat = new osg::PositionAttitudeTransform();
    pat->setPosition(osg::Vec3f(0,0,0));
    pat->addChild(m_world_model);
    */

    //root_group->addChild(m_world_model);

    viewer->realize();
    viewer2->realize();
    viewer3->realize();
    //viewer->run(); 

    while(!viewer->done()){
        double simTime = viewer->getFrameStamp()->getSimulationTime();
        pat->setPosition(osg::Vec3f(simTime,0,0));
        pat2->setPosition(osg::Vec3f(0,simTime,0));
        pat3->setPosition(osg::Vec3f(0,0,simTime));
        viewer->frame(simTime);
        viewer2->frame(simTime);
        viewer3->frame(simTime);
        viewer->advance();
        viewer2->advance();
        viewer3->advance();
        info() << "scene redrawn at " << simTime;

        std::stringstream str;
        str << "frames/u" << simTime << ".png";
        osgDB::writeImageFile(*imag,str.str());//renders images looking up
         
        std::stringstream str2;
        str2 << "frames/d" << simTime << ".png";
        osgDB::writeImageFile(*imag2,str2.str());//renders images lookig down
        
        std::stringstream str3;
        str3 << "frames/f" <<simTime << ".png";
        osgDB::writeImageFile(*imag3,str3.str());//renders images looking forward
        
    }
    return 0;
}
