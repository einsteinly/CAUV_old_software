#include <iostream>
#include <sstream>
#include <stdint.h>

#include <debug/cauv_debug.h>

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osg/PositionAttitudeTransform>

#include <osgOcean/ShaderManager>
#include <osgOcean/OceanScene>

#include <osgViewer/Viewer>

#include <osgDB/WriteFile>

#include <osg/Image>

#include "simulator.h"

#include <sstream>

int main(int argc, char** argv)
{


    osgDB::Registry::instance()->getDataFilePathList().push_back("/home/andy/dev/libs/openscenegraph/OpenSceneGraph-Data");
    const std::string filename = "cow.osg";
    osg::ref_ptr<osg::Node> cow = osgDB::readNodeFile(filename);


    // graphics context for off screen rendering
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits();
    traits->x = 0; // viewport of camera
    traits->y = 0;
    traits->width = 1024;
    traits->height = 768;
    traits->windowDecoration = false;
    traits->doubleBuffer = false;
    traits->sharedContext = NULL;
    traits->pbuffer = true;

    osg::GraphicsContext *graphicsContext = osg::GraphicsContext::createGraphicsContext(traits.get());

    if(!graphicsContext) {
        osg::notify(osg::NOTICE) << __FILE__ << " - " << __FUNCTION__ << " - Failed to create pbuffer." << std::endl;
        exit(1);
    }


    osgViewer::Viewer * viewer = new osgViewer::Viewer();
    viewer->setSceneData(cow);
    viewer->getCamera()->setGraphicsContext(graphicsContext);
    viewer->getCamera()->setViewport(new osg::Viewport(0,0,1024,768));

    viewer->getCamera()->setViewMatrixAsLookAt(osg::Vec3f(0,-20,0), osg::Vec3f(0, 0, 0), osg::Vec3f(0,0,1));

    int frames = 0;

    osg::Image* shot = new osg::Image();
    shot->allocateImage(1024, 768, 24, GL_RGB, GL_UNSIGNED_BYTE);
    viewer->getCamera()->attach(osg::Camera::COLOR_BUFFER, shot);


    viewer->realize();
    while(!viewer->done()){
        viewer->frame();
        info() << "scene redrawn";

        std::stringstream str;
        str << "frames/" << frames++ << ".png";
        osgDB::writeImageFile(*shot,str.str());
    }










/*
    osgDB::Registry::instance()->getDataFilePathList().push_back("resources/island");
    osgDB::Registry::instance()->getDataFilePathList().push_back("sim/resources");

    cauv::sim::Simulator sim;

    int ret = sim.parseOptions(argc, argv);
    if(ret != 0) return ret;

    //osgDB::Registry::instance()->getDataFilePathList().push_back("/home/andy/dev/libs/osg/OpenSceneGraph-Data");
    //osgDB::Registry::instance()->getDataFilePathList().push_back("/home/andy/dev/libs/osgOcean/osgOcean");
    //const std::string filename = "cow.osg";
    //osg::ref_ptr<osg::Node> ces = osgDB::readNodeFile(filename);

    //if(!ces)
    //    error() << "cessna could not be loaded";

    //osg::StateSet* cesState = ces->getOrCreateStateSet();

    //osg::ref_ptr<osgOcean::OceanScene> scene = sim.getWorldModel()->getOceanSceneModel()->getOceanScene();
    //ces->setNodeMask( 0 );


    //osg::Program* someProgram = new osg::Program;
    //osg::Shader* cesVertexObject = new osg::Shader( osg::Shader::VERTEX );
    //osg::Shader* cesFragmentObject = new osg::Shader( osg::Shader::FRAGMENT );

    //cesVertexObject->setShaderSource("void main(void){ gl_TexCoord[0].st = gl_MultiTexCoord0.st; gl_Position = ftransform(); }");
    //cesFragmentObject->setShaderSource("uniform sampler2D pTexture; void main(void){ gl_FragData[0] = texture2D( pTexture, gl_TexCoord[0].st ); gl_FragData[1] = vec4(0.0); }");

    //someProgram->addShader( cesFragmentObject );
    //someProgram->addShader( cesVertexObject );

    //cesState->setAttributeAndModes(someProgram, osg::StateAttribute::ON);

    //osg::PositionAttitudeTransform * pat = new osg::PositionAttitudeTransform();
    //pat->setPosition(osg::Vec3f(0,0,0));
    //pat->addChild(ces);

    //scene->addChild(pat);
    //sim.getWorldModel()->addChild(sim.getWorldModel()->getOceanSceneModel()->getScene());

    //sim.getWorldModel()->addChild(pat);

    sim.run();
*/
    return 0;
}
