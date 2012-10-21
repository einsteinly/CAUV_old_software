/* Copyright 2012 Cambridge Hydronautics Ltd.
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

#include "messaging.h"
#include "fluiditynode.h"

#include <gui/core/model/nodes/groupingnode.h>

using namespace cauv;
using namespace cauv::gui;


FluiditySubscribeObserver::FluiditySubscribeObserver() {
    qRegisterMetaType<MessageType::e>("MessageType::e");
}
FluiditySubscribeObserver::~FluiditySubscribeObserver() {}

void FluiditySubscribeObserver::onSubscribed(MessageType::e messageType){
    Q_EMIT onSubscriptionConfirmation(messageType);
}


FluidityMessageObserver::FluidityMessageObserver(boost::shared_ptr<Node> parent) : m_parent(parent){
}

FluidityMessageObserver::~FluidityMessageObserver(){
}


void FluidityMessageObserver::onPipelineDiscoveryResponseMessage(PipelineDiscoveryResponseMessage_ptr m) {
    info() << mkStr(*m).lengthLimit(120).str();
    addPipeline(m->pipelineName());
}

void FluidityMessageObserver::onGraphDescriptionMessage(GraphDescriptionMessage_ptr m) {
    info() << mkStr(*m).lengthLimit(120).str();
    addPipeline(m->pipelineName());
}

void FluidityMessageObserver::onNodeAddedMessage(NodeAddedMessage_ptr m){
    info() << mkStr(*m).lengthLimit(120).str();
    addPipeline(m->pipelineName());
}

void FluidityMessageObserver::addPipeline(std::string const& name) {
    boost::shared_ptr<GroupingNode> pipelines = m_parent->findOrCreate<GroupingNode>("pipelines");

    QString full_pipeline_name = QString::fromStdString(name);
    QStringList basename_subname = full_pipeline_name.split('/');
    if(basename_subname.size() == 1){
        // no sub-name
        pipelines->findOrCreate<GroupingNode>(name);

    }
    else if(basename_subname.size() == 2){
        boost::shared_ptr<GroupingNode> pipeline_group = pipelines->findOrCreate<GroupingNode>(
                    std::string(basename_subname[0].toUtf8().data())
                    );
        pipeline_group->findOrCreate<FluidityNode>(
                    std::string(basename_subname[1].toUtf8().data())
                    );
    } else {
        error() << "invalid pipeline ID:" << name;
    }
}


