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

#ifndef GAMEPADPLUGIN_H
#define GAMEPADPLUGIN_H

#include <gui/core/cauvbasicplugin.h>

#include <QObject>

namespace cauv {

    class GamepadPlugin : public QObject, public CauvBasicPlugin
    {
        Q_OBJECT
        Q_INTERFACES(cauv::CauvInterfacePlugin)

    public:

        virtual const QString name() const;
        virtual const QList<QString> getGroups() const;
        virtual void initialise(boost::shared_ptr<AUV>, boost::shared_ptr<CauvNode> node);

    };

} // namespace cauv

#endif // GAMEPADPLUGIN_H
