#include "cauvbasicplugin.h"

using namespace cauv;

void CauvBasicPlugin::initialise(boost::shared_ptr<AUV> auv, boost::shared_ptr<CauvNode> node) {
    m_auv = auv;
    m_node = node;
}

const QMap<QDockWidget*, Qt::DockWidgetArea> &CauvBasicPlugin::getDockWidgets() const {
    return m_docks;
}

const QList<QWidget* > &CauvBasicPlugin::getCentralWidgets() const {
    return m_tabs;
}
