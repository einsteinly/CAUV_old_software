#include "pipelineMessageObserver.h"

#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>

#include "pipelineWidget.h"
#include "renderable/node.h"
#include "renderable/imgNode.h"
#include "renderable/arc.h"

using namespace pw;


void DBGLevelObserver::onDebugLevelMessage(DebugLevelMessage_ptr m)
{
    debug::setLevel(m->level());
}


PipelineGuiMsgObs::PipelineGuiMsgObs(PipelineWidget *p)
    : m_widget(p){
    setDoubleBuffered(MessageType::GuiImage, true);
}

void PipelineGuiMsgObs::onNodeAddedMessage(NodeAddedMessage_ptr m){
    debug(2) << BashColour::Green << "PiplineGuiMsgObs:" << __func__ << *m;
    switch(m->nodeType()){
        case NodeType::Invalid:
            break;
        case NodeType::GuiOutput:
            m_widget->addImgNode(boost::make_shared<ImgNode>(m_widget, m_widget, m));
            break;
        default:
            m_widget->addNode(boost::make_shared<Node>(m_widget, m_widget, m));
            break;
    }
}

void PipelineGuiMsgObs::onNodeRemovedMessage(NodeRemovedMessage_ptr m){
    debug(2) << BashColour::Green << "PiplineGuiMsgObs:" << __func__ << *m;
    m_widget->remove(m_widget->node(m->nodeId()));
}

void PipelineGuiMsgObs::onNodeParametersMessage(NodeParametersMessage_ptr m){
    debug(2) << BashColour::Green << "PiplineGuiMsgObs:" << __func__ << *m;
    if(node_ptr_t np = m_widget->node(m->nodeId()))
        np->setParams(m);
}

void PipelineGuiMsgObs::onArcAddedMessage(ArcAddedMessage_ptr m){
    debug(2) << BashColour::Green << "PiplineGuiMsgObs:" << __func__ << *m;
    m_widget->addArc(m->from().node, m->from().output,
                     m->to().node, m->to().input);
}

void PipelineGuiMsgObs::onArcRemovedMessage(ArcRemovedMessage_ptr m){
    debug(2) << BashColour::Green << "PiplineGuiMsgObs:" << __func__ << *m;
    m_widget->removeArc(m->from().node, m->from().output,
                        m->to().node, m->to().input);
}

void PipelineGuiMsgObs::onGraphDescriptionMessage(GraphDescriptionMessage_ptr m){
    debug(2) << BashColour::Green << "PiplineGuiMsgObs:" << __func__ << *m;

    typedef std::map<node_id, NodeType::e> node_type_map_t;
    typedef std::map<node_id, std::map<std::string, NodeOutput> > node_input_map_t;
    typedef std::map<node_id, std::map<std::string, std::vector<NodeInput> > > node_output_map_t;
    typedef std::map<node_id, std::map<std::string, NodeParamValue> > node_param_map_t;

    // remove nodes that shouldn't exist
    const std::vector<node_ptr_t> current_nodes = m_widget->nodes();
    std::vector<node_ptr_t>::const_iterator j;
    for(j = current_nodes.begin(); j != current_nodes.end(); j++)
        if(!m->nodeTypes().count((*j)->id())){
            m_widget->remove(*j);
        }

    // make sure all nodes exist with the correct inputs and outputs
    node_type_map_t::const_iterator i;
    for(i = m->nodeTypes().begin(); i != m->nodeTypes().end(); i++){
        node_ptr_t n;
        if(n = m_widget->node(i->first)); else{
            switch(i->second){
                case NodeType::Invalid:
                    break;
                case NodeType::GuiOutput:
                    m_widget->addImgNode(boost::make_shared<ImgNode>(m_widget, m_widget, i->first, i->second));
                    break;
                default:
                    m_widget->addNode(boost::make_shared<Node>(m_widget, m_widget, i->first, i->second));
                    break;
            }
            n = m_widget->node(i->first);
        }
        if(!n){
            error() << "couldn't add node";
            continue;
        }
        n->setType(i->second);

        node_output_map_t::const_iterator oi = m->nodeOutputs().find(i->first);
        if(oi == m->nodeOutputs().end())
            error() << __func__ << __LINE__;
        else
            n->setOutputs(oi->second);

        node_param_map_t::const_iterator pi = m->nodeParams().find(i->first);
        if(pi == m->nodeParams().end())
            error() << __func__ << __LINE__;
        else
            n->setParams(pi->second);

        node_input_map_t::const_iterator ii = m->nodeInputs().find(i->first);
        if(ii == m->nodeInputs().end())
            error() << __func__ << __LINE__;
        else
            n->setInputs(ii->second);
    }

    // now actually add arcs
    for(i = m->nodeTypes().begin(); i != m->nodeTypes().end(); i++){
        node_ptr_t n = m_widget->node(i->first);
        if(!n){
            error() << __func__ << __LINE__ <<  "required node not present";
            continue;
        }

        node_output_map_t::const_iterator oi = m->nodeOutputs().find(i->first);
        if(oi == m->nodeOutputs().end())
            error() << __func__ << __LINE__;
        else
            n->setOutputLinks(oi->second);

        node_input_map_t::const_iterator ii = m->nodeInputs().find(i->first);
        if(ii == m->nodeInputs().end())
            error() << __func__ << __LINE__;
        else{
            n->setParamLinks(ii->second);
            n->setInputLinks(ii->second);
        }
    }
}

void PipelineGuiMsgObs::onStatusMessage(StatusMessage_ptr m){
    debug(2) << BashColour::Green << "PiplineGuiMsgObs:" << __func__ << *m;
    if(node_ptr_t np = m_widget->node(m->nodeId()))
        np->status(m->status());
}

void PipelineGuiMsgObs::onInputStatusMessage(InputStatusMessage_ptr m){
    debug(2) << BashColour::Green << "PiplineGuiMsgObs:" << __func__ << *m;
    if(node_ptr_t np = m_widget->node(m->nodeId()))
        np->inputStatus(m->inputId(), m->status());
}

void PipelineGuiMsgObs::onOutpuStatusMessage(OutputStatusMessage_ptr m){
    debug(2) << BashColour::Green << "PiplineGuiMsgObs:" << __func__ << *m;
    if(node_ptr_t np = m_widget->node(m->nodeId()))
        np->outputStatus(m->outputId(), m->status());
}

void PipelineGuiMsgObs::onGuiImageMessageBuffered(GuiImageMessage_ptr m){
    debug(2) << BashColour::Green << "PiplineGuiMsgObs:" << __func__ << *m;
    if(imgnode_ptr_t np = m_widget->imgNode(m->nodeId()))
        np->display(m->image());
}
