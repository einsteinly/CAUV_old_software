/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef AIPLUGIN_H
#define AIPLUGIN_H

#include <common/zeromq/zeromq_mailbox.h>

#include <cauvbasicplugin.h>
#include <model/node.h>

#include <QObject>

#include <liquid/node.h>

namespace cauv {
namespace gui {

class NodeChildrenExclusionFilter;

class AiPlugin : public QObject, public CauvBasicPlugin, public SubscribeObserver
{
    Q_OBJECT
    Q_INTERFACES(cauv::gui::CauvInterfacePlugin)

public:
    AiPlugin();
    virtual const QString name() const;
    virtual void initialise();

    typedef liquid::LiquidNode LiquidNode;

public Q_SLOTS:
    void setupTask(boost::shared_ptr<Node> node);
    void setupCondition(boost::shared_ptr<Node> node);
    void setupVehicle(boost::shared_ptr<Node> node);
    void reloadAi();
    void nodeClosed(LiquidNode * node);
    void onSubscribed(MessageType::e messageType);

protected Q_SLOTS:
    void resetTask();
    void stopTask();
    void startTask();
    void pauseAi();
    void resumeAi();
    void toggleAi();
    void saveAi();
    void keyPressed(int,Qt::KeyboardModifiers);

protected:
    boost::shared_ptr<NodeChildrenExclusionFilter> m_filter;
    bool m_aiRunning;

};


} // namespace gui
} // namespace cauv

#endif // AIPLUGIN_H
