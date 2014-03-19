#include <fluidity/model.h>
#include <fluidity/fNode.h>
#include <fluidity/manager.h>

using namespace cauv;
using namespace cauv::gui::f;
using namespace cauv::pipeline_model;


// ----------- GuiNodeModel -------------

GuiNodeModel::GuiNodeModel(const std::string type, GuiPipelineModel &pipeline) :
    NodeModel(type, pipeline),
    m_node(nullptr) {}

GuiNodeModel::GuiNodeModel(const NodeModelType& type_, GuiPipelineModel &pipeline_) :
    NodeModel(type_, pipeline_),
    m_node(nullptr) {}
    
void GuiNodeModel::setFNode(FNode* node){
    m_node = node;
}

FNode* GuiNodeModel::getFNode(){
    return m_node;
}

void GuiNodeModel::connectOutput(const std::string output, InputModel& input){
    NodeModel::connectOutput(output, input);
    auto to = boost::static_pointer_cast<GuiNodeModel>(input.node);
    m_node->constructArcTo(output, *(to->getFNode()), input.name);
}
//void connectInput(const std::string input, OutputModel&);

void GuiNodeModel::disconnectOutput(const std::string output){
    //this might be called after we've removed the assoicated FNode, in which case do nothing
    if (!m_node){ return; }
    for (InputModel& link : getOutput(output).outputs){
        auto to = boost::static_pointer_cast<GuiNodeModel>(link.node);
        m_node->destructArcTo(output, *(to->getFNode()), link.name);
    }
    NodeModel::disconnectOutput(output);
}

void GuiNodeModel::disconnectInput(const std::string input){
    if (!m_node){ return; }
    OutputModel* output = getInput(input).input;
    if (output){
        auto from = boost::static_pointer_cast<GuiNodeModel>(output->node);
        from->getFNode()->destructArcTo(output->name, *m_node, input);
        NodeModel::disconnectInput(input);
    }
}

// ----------- GuiPipelineModel -------------

GuiPipelineModel::GuiPipelineModel(const std::string& pipeline_name, Manager* manager) :
    PipelineModel(pipeline_name),
    m_manager(manager){}

boost::shared_ptr<NodeModel> GuiPipelineModel::constructNode(const std::string type) {
    return boost::make_shared<GuiNodeModel>(type, *this);
}
    
boost::shared_ptr<NodeModel> GuiPipelineModel::addNode(const std::string type){
    auto node = boost::static_pointer_cast<GuiNodeModel>(PipelineModel::addNode(type));
    m_manager->constructFNode(node);
    return node;
}

void GuiPipelineModel::delNode(const std::string &name){
    m_manager->destructFNode(boost::static_pointer_cast<GuiNodeModel>(getNode(name)));
    PipelineModel::delNode(name);
}

