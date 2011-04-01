#ifndef CAUVBASICPLUGIN_H
#define CAUVBASICPLUGIN_H

#include <gui/core/cauvplugins.h>

#include <QList>
#include <QMap>
#include <QString>

namespace cauv {

    class CauvBasicPlugin : public CauvInterfacePlugin
    {
        Q_INTERFACES(cauv::CauvInterfacePlugin)

    public:
        CauvBasicPlugin(){}
        virtual ~CauvBasicPlugin(){}

        virtual void initialise(boost::shared_ptr<AUV> auv, boost::shared_ptr<CauvNode> node);

        virtual const QMap<QDockWidget* , Qt::DockWidgetArea> &getDockWidgets() const;
        virtual const QList<QWidget* > &getCentralWidgets() const;

    protected:
        boost::shared_ptr<AUV> m_auv;
        boost::shared_ptr<CauvNode> m_node;

        QList<QWidget *> m_tabs;
        QMap<QDockWidget *, Qt::DockWidgetArea> m_docks;
    };
}

#endif // CAUVBASICPLUGIN_H
