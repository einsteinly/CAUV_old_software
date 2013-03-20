/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef PROCESSPLUGIN_H
#define PROCESSPLUGIN_H

#include <common/zeromq/zeromq_mailbox.h>

#include <cauvbasicplugin.h>
#include <model/node.h>

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
    void setupVehicle(boost::shared_ptr<Node> node);
    void reloadProcesses();
    void onSubscribed(MessageType::e messageType);

protected:
    boost::shared_ptr<NodeChildrenExclusionFilter> m_filter;

};


} // namespace gui
} // namespace cauv

#endif // PROCESSPLUGIN_H
