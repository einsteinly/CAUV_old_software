/* Copyright 2011 Cambridge Hydronautics Ltd.
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

#ifndef AIPLUGIN_H
#define AIPLUGIN_H

#include <common/zeromq/zeromq_mailbox.h>

#include <gui/core/cauvbasicplugin.h>
#include <gui/core/model/node.h>

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
    void keyPressed(int,Qt::KeyboardModifiers);

protected:
    boost::shared_ptr<NodeChildrenExclusionFilter> m_filter;
    bool m_aiRunning;

};


} // namespace gui
} // namespace cauv

#endif // AIPLUGIN_H
