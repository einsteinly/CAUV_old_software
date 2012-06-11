/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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
#include <boost/ref.hpp>

#include <common/mailbox.h>

#include <generated/message_observers.h>
#include <generated/types/MembershipChangedMessage.h>
#include <generated/types/AddNodeMessage.h>
#include <generated/types/GraphRequestMessage.h>

#include "imageProcessor.h"
#include "scheduler.h"

using namespace std;
using namespace cauv::imgproc;
namespace po = boost::program_options;


ImagePipelineNode::ImagePipelineNode()
    : CauvNode("img-pipe"),
      MessageObserver(),
      m_pipeline_name_root("default"),
      m_pipelines(),
      m_scheduler(new Scheduler())
{
    mailbox()->subMessage(AddNodeMessage());
    mailbox()->subMessage(GraphRequestMessage());
}

void ImagePipelineNode::spawnNewPipeline(std::string const& with_name)
{
    if(!m_pipelines.count(with_name)){
        info() << "starting pipeline, name: \"" << with_name << "\"";
        boost::shared_ptr<ImageProcessor> p = boost::make_shared<ImageProcessor>(mailbox(), m_scheduler);
        p->start(with_name);
        addMessageObserver(p);
        m_pipelines[with_name] = p;
    }
}

void ImagePipelineNode::onAddNodeMessage(AddNodeMessage_ptr m)
{
    spawnNewPipeline(m->pipelineName());
}

void ImagePipelineNode::onRemoveNodeMessage(RemoveNodeMessage_ptr m)
{
    spawnNewPipeline(m->pipelineName());
}

void ImagePipelineNode::onGraphRequestMessage(GraphRequestMessage_ptr m)
{
    spawnNewPipeline(m->pipelineName());
}

void ImagePipelineNode::onClearPipelineMessage(ClearPipelineMessage_ptr m)
{
    spawnNewPipeline(m->pipelineName());
}

void ImagePipelineNode::onRun()
{
    spawnNewPipeline(m_pipeline_name_root);

    m_scheduler->start();
}

void ImagePipelineNode::addOptions(po::options_description& desc,
                                   po::positional_options_description& pos)
{
    CauvNode::addOptions(desc, pos);
   
    desc.add_options()
        ("name,n", po::value<std::string>()->default_value("default"),
            "Pipeline name (e.g. ai) The pipeline will respond to, and "
            "create if necessary, sub-pipelines with any name starting with "
            "this prefix followed by '/' *in addition* to the main pipeline. "
            "For example, if name is 'ai', 'ai/skynet', 'ai' and 'ai/hal' "
            "would both be valid pipeline names that this process will "
            "respond to.")
    ;

    pos.add("name", 1);
}

int ImagePipelineNode::useOptionsMap(po::variables_map& vm,
                                     po::options_description& desc)
{
    int ret = CauvNode::useOptionsMap(vm, desc);
    if (ret != 0) return ret;
    
    m_pipeline_name_root = vm["name"].as<std::string>();

    return 0;
}

static boost::shared_ptr<ImagePipelineNode> node;

void cleanup()
{
    info() << "Cleaning up...";
    node.reset();
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
    node = boost::make_shared<ImagePipelineNode>();

    // observes it's own mailbox
    node->addMessageObserver(node);
    
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
