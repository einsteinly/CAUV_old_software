#ifndef CAUVINTERFACEPLUGIN_H
#define CAUVINTERFACEPLUGIN_H

#include <boost/shared_ptr.hpp>

#include <vector>

#include <QMap>
#include <QList>
#include <Qt>
#include <QtPlugin>


class QWidget;
class QDockWidget;
class QString;


namespace cauv {

    class AUV;
    class CauvNode;

    class CauvInterfacePlugin {

    public:
        virtual ~CauvInterfacePlugin() {}

        //virtual QMap<QDockWidget, Qt::DockWidgetArea> &getDockWidgets() const = 0;
        //virtual QList<QWidget> &getCentralWidgets() const = 0;
        //virtual QString &name() const;
        virtual std::vector<std::string> getGroups() = 0;

        virtual void initialise(boost::shared_ptr<AUV>, boost::shared_ptr<CauvNode> node) = 0;

    };

} // namespace cauv

Q_DECLARE_INTERFACE(cauv::CauvInterfacePlugin, "CauvInterfacePlugin/1.0")


#endif // CAUVINTERFACEPLUGIN_H
