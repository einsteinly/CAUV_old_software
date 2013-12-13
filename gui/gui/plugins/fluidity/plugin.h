/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_FUIDITY_PLUGIN_H__
#define __CAUV_FUIDITY_PLUGIN_H__

#include <QObject>

#include <ros/subscriber.h>

#include <std_msgs/String.h>

#include <cauvbasicplugin.h>

#include <model/node.h>
#include <model/nodes/groupingnode.h>
#include <model/nodes/vehiclenode.h>

#include <liquid/node.h>

namespace cauv{

namespace gui{

class FluidityNode;

class NodeChildrenExclusionFilter;

/* Only one FluidityPlugin per process. An exception will be thrown from the
 * initialise function if more than one FluidityPlugin is created and
 * initialised in the same process.
 * (It would be possible to relax this constraint a little)
 */
class FluidityPlugin: public QObject,
        public CauvBasicPlugin
{
    Q_OBJECT
    Q_INTERFACES(cauv::gui::CauvInterfacePlugin)

public:
    FluidityPlugin();
    virtual const QString name() const;
    virtual void initialise();

public Q_SLOTS:
    void setupPipeline(boost::shared_ptr<Node> node);

private:
    friend class LiquidFluidityNode;

    // this is set by the first instance of a FluidityPlugin to be initialised.
    static bool is_first;
    boost::shared_ptr<Node> m_parent;
    
    ros::Subscriber m_new_pipeline_sub;
    std::vector<boost::shared_ptr<ros::Subscriber>> m_pipeline_update_subs;
    void addPipeline(const std::string& name);
    void addPipeline(QString& full_pipeline_name);
    void onNewPipeline(const std_msgs::String::ConstPtr& msg);


};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_FUIDITY_PLUGIN_H__
