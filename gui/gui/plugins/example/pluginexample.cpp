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

#include "pluginexample.h"

#include <QDockWidget>
#include <QPushButton>
#include <QString>

#include <debug/cauv_debug.h>

using namespace cauv;

PluginExample::PluginExample()
{
    m_tabs.append(new QPushButton("Hello, World."));
}

const QString PluginExample::name() const{
    return QString("my name");
}

const QList<QString> PluginExample::getGroups() const{
    QList<QString> groups;
    groups.push_back(QString("gui"));
    return groups;
}

void PluginExample::initialise(boost::shared_ptr<AUV> auv, boost::shared_ptr<CauvNode> node){
    info() << "test plugin initisalised";
    CauvBasicPlugin::initialise(auv, node);
}


Q_EXPORT_PLUGIN2(cauv_testplugin, PluginExample)
