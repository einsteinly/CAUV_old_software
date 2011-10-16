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
