#include "cauvbasicplugin.h"

#include <debug/cauv_debug.h>

using namespace cauv;
using namespace cauv::gui;

CauvBasicPlugin::~CauvBasicPlugin(){
    debug(2) << "~CauvBasicPlugin()";
}

void CauvBasicPlugin::initialise(boost::shared_ptr<GuiActions> actions) {
    m_auv = actions->auv;
    m_actions = actions;

    initialise();
}

void CauvBasicPlugin::shutdown(){

}
