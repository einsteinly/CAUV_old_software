/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "manager.h"

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QTimer>

#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>
#include <utility/bash_cout.h>

#include <liquid/layout.h>
#include <liquid/itemFridge.h>

#include <model/node.h>
#include <model/nodeItemModel.h>
#include <nodescene.h>
#include <drag/graphDropHandler.h>

#include "fNode.h"
//#include "imageSource.h"

using namespace cauv;
using namespace cauv::gui::f;
using namespace cauv::pipeline_model;

// - Static helper functions
// static bool isImageNode(NodeType::e t){
//     return t == NodeType::GuiOutput;
// }
// 
// static bool isInvalid(NodeType::e t){
//     return t == NodeType::Invalid;
// }
// 
// static bool isInvalid(NodeId const& id){
//     return id == 0;
// }

// - General Public Implementation:
Manager::Manager(NodeScene *scene,
                 boost::shared_ptr<Node> model_parent,
                 const std::string& pipeline_name)
    : QObject(),
      //DropHandlerInterface<QGraphicsItem*>(),
      boost::enable_shared_from_this<Manager>(),
      m_pipeline_model(pipeline_name, this),
      m_scene(scene),
//       m_nodes(),
//       m_pipeline_name(pipeline_name),
//       m_image_sources(), 
      m_focus_scenepos(0,0),
      m_layout_soon_timer(new QTimer()),
      m_model_parent(model_parent),
      m_item_model(new gui::NodeItemModel(model_parent->getRoot())){

    m_animation_permitted.push(true);
    m_scene->installEventFilter(new FocusPositionForwarder(*this));
//     m_layout_soon_timer->setSingleShot(true);
//     connect(m_layout_soon_timer, SIGNAL(timeout()), this, SLOT(updateLayoutNow()));
}

Manager::~Manager(){
    CAUV_LOG_DEBUG(1, "~Manager()");
    teardown();
    delete m_layout_soon_timer;
}


void Manager::init(){
    m_scene->registerDropHandler(boost::make_shared<gui::GraphingDropHandler>(m_item_model));
}

void Manager::teardown(){

}

// fnode_ptr Manager::lookup(node_id_t const& id){
//     node_id_map_t::right_const_iterator i = m_nodes.find(id);
//     if(i != m_nodes.right().end())
//         return i->left;
//     return nullptr;
// }

// void Manager::sendMessage(boost::shared_ptr<const Message> m) const{
//     m_cauv_node->send(m);
// }

// const std::string& Manager::pipelineName() const{
//     return m_pipeline_name;
// }

boost::shared_ptr<gui::Node> Manager::model() const {
    return m_model_parent;
}

gui::NodeItemModel * Manager::itemModel() const {
    return m_item_model.get();
}

bool Manager::animationPermitted() const{
   return m_animation_permitted.top();
}

void Manager::pushAnimationPermittedState(bool permitted){
    m_animation_permitted.push(permitted);
}

void Manager::popAnimationPermittedState(){
    m_animation_permitted.pop();
}

void Manager::considerUpdatingLayout(){
    _layoutSoonIfNothingHappens();
}

void Manager::delayLayout(){
    if(m_layout_soon_timer->isActive())
        _layoutSoonIfNothingHappens();
}

void Manager::setFocusPosition(QPointF p){
    m_focus_scenepos = QPointF(qRound(p.x()), qRound(p.y()));
}
/*
// - DropHandlerInterface Implementation
bool Manager::accepts(boost::shared_ptr<cauv::gui::Node> const& node){
    debug() << "Manager drop enter from" << node->nodePath();
    return false;
}

QGraphicsItem* Manager::handle(boost::shared_ptr<cauv::gui::Node> const& node){
    debug() << "Manager drop from" << node->nodePath();
    return NULL;
}*/

QList<QGraphicsItem*> Manager::rootNodes() const{
    QList<QGraphicsItem*> r;
#warning
//     node_id_map_t::right_const_iterator i;
//     for(i = m_nodes.right().begin(); i != m_nodes.right().end(); i++)
//         r << i->left;
    return r;
}

// // - Message Handling Slots:
// void Manager::onGraphDescription(GraphDescriptionMessage_ptr m){
//     debug(7) << BashColour::Green << "Manager::" << __func__ << *m;
// 
//     typedef std::map<node_id_t, NodeType::e> node_type_map_t;
//     typedef std::map<node_id_t, FNode::msg_node_input_map_t> node_input_map_t;
//     typedef std::map<node_id_t, FNode::msg_node_output_map_t> node_output_map_t;
//     typedef std::map<node_id_t, FNode::msg_node_param_map_t> node_param_map_t;
//     
//     AnimationPermittedState(*this, false);
// 
//     typedef QMap<QGraphicsView*,QGraphicsView::ViewportUpdateMode> view_updatestate_qmap;
//     view_updatestate_qmap saved_view_update_states;
//     for (QGraphicsView* view : m_scene->views()){
//         saved_view_update_states.insert(view, view->viewportUpdateMode());
//         view->setViewportUpdateMode(QGraphicsView::NoViewportUpdate);
//     }
// 
//     // remove nodes that shouldn't exist
//     node_id_map_t::right_iterator i;
//     node_type_map_t::const_iterator j;
// 
//     bool inconsistency_detected = false;
//     std::vector<node_id_t> nodes_for_removal;
//     for(i = m_nodes.right().begin(); i != m_nodes.right().end(); i++){
//         j = m->nodeTypes().find(i->right);
//         if(j == m->nodeTypes().end())
//             nodes_for_removal.push_back(i->right);
//         else if(j->second != i->left->nodeType())
//             inconsistency_detected = true;
//     }
//     if(!inconsistency_detected){
//         for (node_id_t id : nodes_for_removal)
//             removeNode(id);
//     
//         // !!! TODO: remove arcs that shouldn't exist
//         warning() << "removing dead arcs is not yet implemented";
//     }
// 
//     if(inconsistency_detected)
//         clearNodes();
//     
//     // destructive operations on nodes and arcs are complete, now make sure all
//     // nodes exist with the right inputs and outputs:
//     
//     for(j = m->nodeTypes().begin(); j != m->nodeTypes().end(); j++){
//         const node_id_t id = j->first;
//         const NodeType::e type = j->second;
//         i = m_nodes.right().find(id); 
//         if(i == m_nodes.right().end()){
//             addNode(type, id);
//             i = m_nodes.right().find(j->first);
//         }
//         if(i == m_nodes.right().end())
//             continue;
//         fnode_ptr node = i->left;
//         
//         auto inputs_it = m->nodeInputs().find(id);
//         auto outputs_it = m->nodeOutputs().find(id);
//         auto params_it = m->nodeParams().find(id);
// 
//         if(inputs_it == m->nodeInputs().end() ||
//            outputs_it == m->nodeOutputs().end() ||
//            params_it == m->nodeParams().end()){
//             error() << "inconsistent graph description for node" << id;
//             continue;
//         }
//         
//         node->setParams(params_it->second);        
//         node->setInputs(inputs_it->second);
//         node->setOutputs(outputs_it->second);
//         
//         _checkAddImageSource(id);
//     }
// 
//     for(j = m->nodeTypes().begin(); j != m->nodeTypes().end(); j++){
//         const node_id_t id = j->first;
//         i = m_nodes.right().find(id);
//         fnode_ptr node = i->left;
//         
//         auto inputs_it = m->nodeInputs().find(id);
//         auto outputs_it = m->nodeOutputs().find(id);
// 
//         node->setInputLinks(inputs_it->second);
//         node->setParamLinks(inputs_it->second);
//         node->setOutputLinks(outputs_it->second);
// 
//         //liquid::ItemFridge<FNode>* p = dynamic_cast<liquid::ItemFridge<FNode>*>(node);
//         //if(p)
//         //    p->freeze();
//     }
// 
//     _layoutSoonIfNothingHappens();
// 
//     view_updatestate_qmap::const_iterator mi = saved_view_update_states.begin();
//     for(; mi != saved_view_update_states.end(); mi++)
//         mi.key()->setViewportUpdateMode(mi.value());
// }
// 
// void Manager::onNodeParameters(NodeParametersMessage_ptr m){
//     node_id_map_t::right_iterator i = m_nodes.right().find(m->nodeId());
//     if(i != m_nodes.right().end())
//         i->left->setParams(m->params());
//     else
//         warning() << "unknown node" << m->nodeId();
// }
// 
// void Manager::onNodeAdded(NodeAddedMessage_ptr m){
//     _animAutoDisableTick();
//     addNode(m);
//     _checkAddImageSource(m->nodeId());
// }
// 
// void Manager::onNodeRemoved(NodeRemovedMessage_ptr m){
//     _animAutoDisableTick();
//     removeNode(m->nodeId());
// }
// 
// void Manager::onArcAdded(ArcAddedMessage_ptr m){
//     _animAutoDisableTick();
//     fnode_ptr from = lookup(m->from().node);
//     fnode_ptr to = lookup(m->to().node);
//     if(from && to)
//         from->connectOutputTo(m->from().output, to, m->to().input);
//     _layoutSoonIfNothingHappens();
// }
// 
// void Manager::onArcRemoved(ArcRemovedMessage_ptr m){
//     _animAutoDisableTick();
//     fnode_ptr from = lookup(m->from().node);
//     fnode_ptr to = lookup(m->to().node);
//     if(from && to)
//         from->disconnectOutputFrom(m->from().output, to, m->to().input);
//     _layoutSoonIfNothingHappens();
// }
// 
// void Manager::onGuiImage(GuiImageMessage_ptr m){
//     id_imgsrc_map_t::const_iterator i = m_image_sources.find(m->nodeId());
//     if(i != m_image_sources.end()){
//         i->second->emitImage(m);
//     }else{
//         warning() << "node" << m->nodeId() << "does not display images";
//     }
// }
// 
// void Manager::onStatus(StatusMessage_ptr m){
//     node_id_map_t::right_iterator i = m_nodes.right().find(m->nodeId());
//     if(i != m_nodes.right().end()){
//         if(((m->status() & cauv::NodeStatus::Bad) &&
//             (i->left->liquid::LiquidNode::status() != liquid::LiquidNode::NotOK)) ||
//            !(m->status() & cauv::NodeStatus::Bad)
//         ){
//             if(m->status() & cauv::NodeStatus::Bad)
//                 i->left->status(liquid::LiquidNode::NotOK, "Could Not Execute");
//             else
//                 i->left->status(liquid::LiquidNode::OK, m->throughput(), m->frequency(), m->timeTaken(), m->timeRatio());
//         }
//     }else{
//         error() << "no such node:" << m->nodeId();
//     }
// }
// 
// 
// // - Slot Implementations
// void Manager::requestArc(NodeOutput from, NodeInput to){
//     debug() << BashColour::Brown << BashIntensity::Bold << "requestArc" << from << to;
//     m_cauv_node->send(boost::make_shared<AddArcMessage>(m_pipeline_name, from, to));
// }
// 
// void Manager::requestRemoveArc(NodeOutput from, NodeInput to){
//     debug() << BashColour::Brown << "requestRemoveArc" << from << to;
//     m_cauv_node->send(boost::make_shared<RemoveArcMessage>(m_pipeline_name, from, to));
// }
// 
// void Manager::requestNode(NodeType::e const& type,
//                           std::vector<NodeInputArc> const& inputs,
//                           std::vector<NodeOutputArc> const& outputs){
//     debug() << BashColour::Brown << BashIntensity::Bold << "requestAddNode" << type;
//     m_cauv_node->send(boost::make_shared<AddNodeMessage>(
//         m_pipeline_name, type, inputs, outputs
//     ));
// }
// 
void Manager::removeNode(FNode &node) {
    CAUV_LOG_DEBUG(0, "removing node " << node.getName());
    //update the model
    m_pipeline_model.delNode(node.getName());
}

// void Manager::requestRefresh(){
//     m_cauv_node->send(boost::make_shared<GraphRequestMessage>(m_pipeline_name));
// }

// void Manager::requestForceExec(node_id_t const& id){
//     m_cauv_node->send(boost::make_shared<ForceExecRequestMessage>(m_pipeline_name, id));
// }

// // - General Protected Implementations
// void Manager::removeNode(node_id_t const& id){
//     debug() << BashColour::Red << "removeNode:" << id;
//     node_id_map_t::right_iterator i = m_nodes.right().find(id);
//     if(i != m_nodes.right().end()){
//         i->left->fadeAndRemove();
//         m_nodes.right().erase(i);
//         _layoutSoonIfNothingHappens();
//     }else{
//         error() << "no such node:" << id;
//     }
// }
// 
// fnode_ptr Manager::addNode(NodeType::e const& type, node_id_t const& id){
//     debug() << BashColour::Green << "addNode:" << type << id;
//     fnode_ptr r = nullptr;
//     if(isInvalid(type)){
//         warning() << "ignoring invalid node type" << type;
//     }else if(isInvalid(id)){
//         warning() << "ignoring invalid node id" << id;
//     }else if(m_nodes.right().find(id) != m_nodes.right().end()){
//         error() << "node" << id << "already exists";
//     }else{
//         //r = new liquid::ItemFridge<FNode>(*this, id, type);
//         r = new FNode(*this, id, type);
//         m_nodes.insert(node_id_map_t::value_type(r, id));
//         m_scene->addItem(r);
//         r->setPos(m_focus_scenepos + QPointF(qrand()%250-125, qrand()%250-125));
//         // this is a bit annoying
//         //_layoutSoonIfNothingHappens();
//     }
//     return r;
// }
// 
// fnode_ptr Manager::addNode(NodeAddedMessage_ptr m){
//     const NodeType::e type = m->nodeType();
//     const node_id_t id = m->nodeId();
//     fnode_ptr r = nullptr;
//     debug() << BashColour::Green << "addNode:" << type << id;
//     if(isInvalid(type)){
//         warning() << "ignoring invalid node type" << type;
//     }else if(isInvalid(id)){
//         warning() << "ignoring invalid node id" << id;
//     }else if(m_nodes.right().find(id) != m_nodes.right().end()){
//         error() << "node" << id << "already exists";
//     }else{
//         //r = new liquid::ItemFridge<FNode>(*this, m);
//         r = new FNode(*this, m);
//         m_nodes.insert(node_id_map_t::value_type(r, id));
//         m_scene->addItem(r);
//         r->setPos(m_focus_scenepos + QPointF(qrand()%250-125, qrand()%250-125));
//         // this is a bit annoying        
//         //_layoutSoonIfNothingHappens();
//     }
//     return r;
// }

void Manager::addNode(const std::string type) {
    m_pipeline_model.addNode(type);
};

FNode* Manager::constructFNode(boost::shared_ptr<GuiNodeModel> node){
    auto fNode = new FNode(node, *this);
    node->setFNode(fNode);
    m_scene->addItem(fNode);
    return fNode;
}

void Manager::destructFNode(boost::shared_ptr<GuiNodeModel> node){
    auto fNode = node->getFNode();
    //remove and delete the fluidity node
    fNode->remove();
    fNode->deleteLater();
    node->setFNode(nullptr);
}

bool Manager::createArc(boost::shared_ptr<pipeline_model::NodeModel> from, const std::string output,
               pipeline_model::InputModel& input){
    from->connectOutput(output, input);
    return true;
}

// void Manager::clearNodes(){
//     debug() << BashColour::Red << "clearNodes";
//     node_id_map_t::right_iterator i;
//     for(i = m_nodes.right().begin(); i != m_nodes.right().end(); i++)
//         i->left->fadeAndRemove();
//     m_nodes.clear();
// }

void Manager::updateLayoutNow(){
    liquid::LayoutItems::updateLayout(m_scene);
}

// !!! FIXME this is a bit untidy and not entirely the right way to do things:
void Manager::_animAutoDisableTick(){
    // only worry if animation isn't already disabled - prevent creating a
    // milllion timers
    if(animationPermitted()){
        TimeStamp tick = now();
            
        float sdiff = tick.secs - m_last_anim_auto_disable_check.secs;
        float musdiff = tick.musecs - m_last_anim_auto_disable_check.musecs; 

        if(sdiff + 1e-6*musdiff < 0.2){
            pushAnimationPermittedState(false);
            QTimer::singleShot(1000, this, SLOT(popAnimationPermittedState()));
        }
        
        m_last_anim_auto_disable_check = tick;
    }
}

void Manager::_layoutSoonIfNothingHappens(){
    m_layout_soon_timer->start(500);
}
/*
void Manager::_checkAddImageSource(node_id_t id){
    if(!isInvalid(id) && m_nodes.count(id) &&
       isImageNode(NodeType::e(m_nodes[id]->nodeType()))){
        boost::shared_ptr<ImageSource> image_src = boost::make_shared<ImageSource>();
        // !!! TODO: include information about which inputs images are associated with
        m_nodes[id]->addImageDisplayOnInput("image_in", image_src);
        m_image_sources[id] = image_src;
    }
}*/
