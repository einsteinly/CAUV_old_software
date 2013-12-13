/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "plugin.h"

#include <stdexcept>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/node_handle.h>
#include <ros/master.h>

#include <debug/cauv_debug.h>
#include <utility/string.h>

#include <model/registry.h>

#include <nodescene.h>

#include "fluiditynode.h"


using namespace cauv;
using namespace cauv::gui;

GENERATE_SIMPLE_NODE(NewPipelineNode)

//TODO merge add and setup pipeline functions

class FluidityDropHandler: public DropHandlerInterface<QGraphicsItem*> {
    public:
    FluidityDropHandler(boost::shared_ptr<NodeItemModel> model,
                        boost::weak_ptr<CauvMainWindow> window)
        : m_model(model),
          m_window(window){
    }

    //this drop handler should deal with creating new and existing pipelines
    virtual bool accepts(boost::shared_ptr<Node> const& node){
        return (node->type == nodeType<FluidityNode>() ||
                node->type == nodeType<NewPipelineNode>());
    }

    //create new pipeline/liquidfluidity node as appropriate
    //TODO neaten up 
    virtual QGraphicsItem * handle(boost::shared_ptr<Node> const& node){
        if(node->type == nodeType<FluidityNode>()){
            if(ConnectedNode * n = LiquidFluidityNode::nodeFor(node)){
                return n;
            }
        }
        if (node->type == nodeType<NewPipelineNode>()) {
            auto pipeline = node->getClosestParentOfType<GroupingNode>();

            size_t nPipelines = pipeline->countChildrenOfType<FluidityNode>();
            pipeline->findOrCreate<FluidityNode>(MakeString() << "pipeline" << (nPipelines + 1));
        }
        return nullptr;
    }

    protected:
    boost::shared_ptr<NodeItemModel> m_model;
    boost::weak_ptr<CauvMainWindow> m_window;
};



FluidityPlugin::FluidityPlugin() : m_parent(VehicleRegistry::instance()){
}

const QString FluidityPlugin::name() const{
    return QString("Fluidity");
}

void FluidityPlugin::initialise(){
    
    if (!is_first) {
        is_first = true;
    } else {
        throw std::runtime_error("only one FluidityPlugin may be initialised");
    }
    
    ros::NodeHandle ros_node;
    
    boost::shared_ptr<GroupingNode> pipelines = m_parent->findOrCreate<GroupingNode>("pipelines");
    connect(pipelines.get(), SIGNAL(childAdded(boost::shared_ptr<Node>)),
            this, SLOT(setupPipeline(boost::shared_ptr<Node>)));

    m_actions->scene->registerDropHandler(
                boost::make_shared<FluidityDropHandler>(m_actions->root, m_actions->window)
                );

    //subscriptions must occur last, as may use other components
    m_new_pipeline_sub = ros_node.subscribe("pipelines/new", 100, &FluidityPlugin::onNewPipeline, this);
    //typedef of std::vector<TopicInfo>
    ros::master::V_TopicInfo topics;
    //loop through topics, those in the pipelines/updates correspond to pipeline names
    if (ros::master::getTopics(topics)){
        foreach(ros::master::TopicInfo topic, topics){
            QString name = QString::fromStdString(topic.name);
            if(name.startsWith("pipelines/updates")){
                addPipeline(name.remove(0,17));
            }
        }
    } else {
        CAUV_LOG_ERROR("Failed to get topic list from master.");
    }
}

void FluidityPlugin::setupPipeline(boost::shared_ptr<Node> node){
    if(boost::dynamic_pointer_cast<GroupingNode>(node)){
        node->findOrCreate<NewPipelineNode>("new");
        
        connect(node.get(), SIGNAL(childAdded(boost::shared_ptr<Node>)),
                this, SLOT(setupPipeline(boost::shared_ptr<Node>)));
        foreach(boost::shared_ptr<Node> child, node->getChildren()){
            setupPipeline(child);
        }
    } else if(boost::shared_ptr<FluidityNode> fn = boost::dynamic_pointer_cast<FluidityNode>(node)){
        m_actions->scene->addItem(new LiquidFluidityNode(fn, m_actions->window));
    }
}

void FluidityPlugin::addPipeline(const std::string& name){
    QString full_pipeline_name = QString::fromStdString(name);
    addPipeline(full_pipeline_name);
}

void FluidityPlugin::addPipeline(QString& full_pipeline_name){
    //Create gui nodes
    boost::shared_ptr<GroupingNode> pipelines = m_parent->findOrCreate<GroupingNode>("pipelines");

    QStringList basename_subname = full_pipeline_name.split('/');
    //note that this relies on the process_name/slot_name structure
    if(basename_subname.size() == 1){
        // no sub-name
        pipelines->findOrCreate<GroupingNode>(full_pipeline_name.toUtf8().data());
    }
    else if(basename_subname.size() == 2){
        boost::shared_ptr<GroupingNode> pipeline_group = pipelines->findOrCreate<GroupingNode>(
                    std::string(basename_subname[0].toUtf8().data())
                    );
        pipeline_group->findOrCreate<FluidityNode>(
                    std::string(basename_subname[1].toUtf8().data())
                    );
    } else {
        CAUV_LOG_ERROR("invalid pipeline ID:" << full_pipeline_name.toUtf8().data());
    }
    //Subscribe to update messages ? shold this be in liquidfluiditynode
    /*if(boost::shared_ptr<ros::NodeHandle> node = m_action->node.lock()){
        m_pipeline_updates_subs.push_back(node.subscribe("imaging/pipelines/updates/" << name,
                                                     100, &FluidityPlugin::onNewPipeline, this))
    } else error() << "Failed to lock ROSNode while adding pipeline:" << name;*/
}

void FluidityPlugin::onNewPipeline(const std_msgs::String::ConstPtr& msg){
    addPipeline(msg->data);
}

Q_EXPORT_PLUGIN2(cauv_fluidityplugin, FluidityPlugin)
