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

#include "cauvbasicplugin.h"

#include <debug/cauv_debug.h>

#include <connectednode.h>

using namespace cauv;
using namespace cauv::gui;

CauvBasicPlugin::~CauvBasicPlugin(){
    debug(2) << "~CauvBasicPlugin()";
}

void CauvBasicPlugin::initialise(boost::shared_ptr<GuiActions> const& actions, ConnectedNodeMap* m) {
    m_actions = actions;

    ConnectedNode::setMap(m);

    initialise();
}

void CauvBasicPlugin::shutdown(){

}
