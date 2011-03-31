#ifndef PLUGINEXAMPLE_H
#define PLUGINEXAMPLE_H

#include <cauvinterfaceplugin.h>
#include <debug/cauv_debug.h>

#include <QObject>

namespace cauv {

    class PluginExample : public QObject, public CauvInterfacePlugin
    {
        Q_OBJECT
        Q_INTERFACES(cauv::CauvInterfacePlugin)


    public:
        PluginExample();

   /* virtual QMap<QDockWidget, Qt::DockWidgetArea> &getDockWidgets() const {
        return QMap<QDockWidget, Qt::DockWidgetArea>();
    }

    virtual QList<QWidget> &getCentralWidgets() const {
        return QList<QWidget>();
    }

    virtual QString &name() const{
        return QString("my name");
    }*/

        std::vector<std::string> getGroups(){
            std::vector<std::string> groups;
            groups.push_back("gui");
            return groups;
        }

       virtual void initialise(boost::shared_ptr<AUV>, boost::shared_ptr<CauvNode> node){
            info() << "test plugin initisalised";
        }

    };

} // namespace cauv

#endif // PLUGINEXAMPLE_H
