#ifndef PLUGINEXAMPLE_H
#define PLUGINEXAMPLE_H

#include <gui/core/cauvbasicplugin.h>

#include <QObject>

namespace cauv {

    class PluginExample : public QObject, public CauvBasicPlugin
    {
        Q_OBJECT
        Q_INTERFACES(cauv::CauvInterfacePlugin)

    public:
        PluginExample();

        virtual const QString name() const;
        virtual const QList<QString> getGroups() const;
        virtual void initialise(boost::shared_ptr<AUV>, boost::shared_ptr<CauvNode> node);

    };

} // namespace cauv

#endif // PLUGINEXAMPLE_H
