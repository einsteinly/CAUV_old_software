
#include <iostream>
#include <sstream>
#include <stdint.h>

#include <debug/cauv_debug.h>

#include <osgDB/ReadFile>

#include "simulator.h"

int main(int argc, char** argv)
{

    cauv::Simulator sim;

    int ret = sim.parseOptions(argc, argv);
    if(ret != 0) return ret;

    //osgDB::Registry::instance()->getDataFilePathList().push_back("/home/andy/dev/libs/openscenegraph/OpenSceneGraph-Data");
    //osgDB::Registry::instance()->getDataFilePathList().push_back("/home/andy/dev/libs/osgOcean/osgOcean");
    osgDB::Registry::instance()->getDataFilePathList().push_back("sim");
    //const std::string filename = "cessnafire.osg";
    //osg::ref_ptr<osg::Node> ces = osgDB::readNodeFile(filename);

    //sim.getWorldModel()->addChild(ces.get());

    sim.run();

    return 0;
}
