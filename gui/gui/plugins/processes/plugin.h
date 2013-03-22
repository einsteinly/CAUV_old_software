/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef PROCESSPLUGIN_H
#define PROCESSPLUGIN_H

#include <common/zeromq/zeromq_mailbox.h>

#include <cauvbasicplugin.h>
#include <model/node.h>

#include <boost/thread.hpp>

#include <QObject>

#include <liquid/node.h>

namespace cauv {
namespace gui {

class NodeChildrenExclusionFilter;

class ProcessPlugin : public QObject, public CauvBasicPlugin, public SubscribeObserver
{
    Q_OBJECT
    Q_INTERFACES(cauv::gui::CauvInterfacePlugin)

public:
    ProcessPlugin();
    virtual const QString name() const;
    virtual void initialise();

public Q_SLOTS:
    void setupProcess(boost::shared_ptr<Node> node);
    void setupHost(boost::shared_ptr<Node> node);
    void setupVehicle(boost::shared_ptr<Node> vnode);
    void reloadProcesses();
    void onSubscribed(MessageType::e messageType);

protected:
    boost::shared_ptr<NodeChildrenExclusionFilter> m_filter;
    typedef boost::mutex mutex_t;
    typedef boost::unique_lock<mutex_t> lock_t;
    mutex_t m_processLock;
};


} // namespace gui
} // namespace cauv

#endif // PROCESSPLUGIN_H
