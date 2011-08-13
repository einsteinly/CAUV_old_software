#include "cauvbasicplugin.h"

using namespace cauv;
using namespace cauv::gui;

void CauvBasicPlugin::initialise(boost::shared_ptr<GuiActions> actions) {
    m_auv = actions->auv;
    m_node = actions->node;
    m_actions = actions;

    initialise();
}
