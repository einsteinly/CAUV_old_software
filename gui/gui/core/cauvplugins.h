#ifndef CAUVINTERFACEPLUGIN_H
#define CAUVINTERFACEPLUGIN_H

#include <boost/shared_ptr.hpp>

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

        virtual const QString name() const = 0;
        virtual const QList<QString> getGroups() const = 0;

        virtual void initialise(boost::shared_ptr<AUV>, boost::shared_ptr<CauvNode> node) = 0;

        virtual const QMap<QDockWidget* , Qt::DockWidgetArea> &getDockWidgets() const = 0;
        virtual const QList<QWidget* > &getCentralWidgets() const = 0;

    };

} // namespace cauv

Q_DECLARE_INTERFACE(cauv::CauvInterfacePlugin, "CauvInterfacePlugin/1.0")

#endif // CAUVINTERFACEPLUGIN_H
