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
