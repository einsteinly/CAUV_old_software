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

#include "img-pipeline.h"

#include <iostream>
#include <sstream>

#include <boost/program_options.hpp>

#include <common/mailbox.h>

#include <generated/types/MembershipChangedMessage.h>

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
