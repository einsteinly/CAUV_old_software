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
