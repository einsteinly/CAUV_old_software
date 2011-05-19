#include "img-pipeline.h"

#include <iostream>
#include <sstream>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <generated/messages.h>

#include <common/spread/spread_rc_mailbox.h>
#include <common/spread/mailbox_monitor.h>
#include <common/spread/msgsrc_mb_observer.h>

#include <boost/program_options.hpp>

#include "imageProcessor.h"

using namespace std;
using namespace cauv::imgproc;
namespace po = boost::program_options;

ImagePipelineNode::ImagePipelineNode()
    : CauvNode("img-pipe"), m_pipeline_name("default"),
      m_pipeline(new ImageProcessor(mailbox()))
{
}

void ImagePipelineNode::onRun()
{
    info() << "starting pipeline, name: \"" << m_pipeline_name << "\"";
    m_pipeline->start(m_pipeline_name);
    
    joinGroup("image");
    joinGroup("pipeline");
    joinGroup("pl_gui");
    joinGroup("sonarout");
    addMessageObserver(m_pipeline);
}

void ImagePipelineNode::addOptions(po::options_description& desc,
                                   po::positional_options_description& pos)
{
    CauvNode::addOptions(desc, pos);
   
    desc.add_options()
        ("name,n", po::value<std::string>()->default_value("default"), "Pipeline name (e.g. sonar-processing)")
    ;

    pos.add("name", 1);
}

int ImagePipelineNode::useOptionsMap(po::variables_map& vm,
                                     po::options_description& desc)
{
    int ret = CauvNode::useOptionsMap(vm, desc);
    if (ret != 0) return ret;
    
    m_pipeline_name = vm["name"].as<std::string>();

    return 0;
}

static ImagePipelineNode* node;

void cleanup()
{
    info() << "Cleaning up...";
    cauv::CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    info() << "Clean up done.";
}

void interrupt(int sig)
{
    info() << BashColour::Red << "Interrupt caught!";
    cleanup();
    signal(SIGINT, SIG_DFL);
    raise(sig);
}

int main(int argc, char** argv)
{
    signal(SIGINT, interrupt);
    node = new ImagePipelineNode();
    
    int ret = 0;
    try{
        ret = node->parseOptions(argc, argv);
    }catch(std::exception& e){
        std::cout << e.what() << std::endl;
        return -1;
    }
    if(ret != 0)
        return ret;

    node->run();
    cleanup();
    return 0;
}
