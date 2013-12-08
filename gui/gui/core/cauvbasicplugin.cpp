/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "cauvbasicplugin.h"

#include <debug/cauv_debug.h>

#include <connectednode.h>

using namespace cauv;
using namespace cauv::gui;

CauvBasicPlugin::~CauvBasicPlugin(){
    CAUV_LOG_DEBUG(2, "~CauvBasicPlugin()");
}

void CauvBasicPlugin::initialise(boost::shared_ptr<GuiActions> const& actions, ConnectedNodeMap* m) {
    m_actions = actions;

    ConnectedNode::setMap(m);

    initialise();
}

void CauvBasicPlugin::shutdown(){

}
