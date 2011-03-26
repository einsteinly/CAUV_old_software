#include <iostream>
#include <sstream>
#include <stdint.h>

#include <debug/cauv_debug.h>

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osg/PositionAttitudeTransform>

#include <osgOcean/ShaderManager>
#include <osgOcean/OceanScene>

#include "simulator.h"

int main(int argc, char** argv)
{

    osgDB::Registry::instance()->getDataFilePathList().push_back("resources/island");
    osgDB::Registry::instance()->getDataFilePathList().push_back("sim/resources");

    cauv::sim::Simulator sim;

    int ret = sim.parseOptions(argc, argv);
    if(ret != 0) return ret;

    osgDB::Registry::instance()->getDataFilePathList().push_back("/home/andy/dev/libs/osg/OpenSceneGraph-Data");
    osgDB::Registry::instance()->getDataFilePathList().push_back("/home/andy/dev/libs/osgOcean/osgOcean");
    const std::string filename = "cow.osg";
    osg::ref_ptr<osg::Node> ces = osgDB::readNodeFile(filename);

    if(!ces)
        error() << "cessna could not be loaded";

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

    osg::PositionAttitudeTransform * pat = new osg::PositionAttitudeTransform();
    pat->setPosition(osg::Vec3f(0,0,0));
    pat->addChild(ces);

    //scene->addChild(pat);
    //sim.getWorldModel()->addChild(sim.getWorldModel()->getOceanSceneModel()->getScene());

    //sim.getWorldModel()->addChild(pat);

    sim.run();

    return 0;
}
